from nova.core.exceptions import PlanTrajectoryFailed, LoadPlanFailed
from nova.gateway import ApiGateway
from nova.types.action import Action, CombinedActions
from nova.types.pose import Pose
from nova.types import (
    MovementController,
    LoadPlanResponse,
    InitialMovementStream,
    InitialMovementConsumer,
)
from loguru import logger
import wandelbots_api_client as wb

from nova.core.movement_controller import move_forward, MovementControllerContext

MAX_JOINT_VELOCITY_PREPARE_MOVE = 0.2
START_LOCATION_OF_MOTION = 0.0


class MotionGroup:
    def __init__(self, api_gateway: ApiGateway, cell: str, motion_group_id: str):
        self._api_gateway = api_gateway
        self._motion_api_client = api_gateway.motion_api
        self._cell = cell
        self._motion_group_id = motion_group_id
        self._current_motion: str | None = None

    @property
    def motion_group_id(self) -> str:
        return self._motion_group_id

    @property
    def current_motion(self) -> str:
        # if not self._current_motion:
        #    raise ValueError("No MotionId attached. There is no planned motion available.")
        return self._current_motion

    async def run(
        self,
        actions: list[Action] | Action,
        tcp: str,
        # collision_scene: dts.CollisionScene | None,
        response_rate_in_ms: int = 200,
        movement_controller: MovementController = move_forward,
        initial_movement_consumer: InitialMovementConsumer | None = None,
    ):
        if not isinstance(actions, list):
            actions = [actions]

        if len(actions) == 0:
            raise ValueError("No actions provided")

        # PLAN MOTION
        joint_trajectory = await self._plan(actions, tcp)

        # LOAD MOTION
        load_plan_response = await self._load_planned_motion(joint_trajectory)

        # MOVE TO START POSITION
        number_of_joints = await self._get_number_of_joints()
        joints_velocities = [MAX_JOINT_VELOCITY_PREPARE_MOVE] * number_of_joints
        movement_stream = await self.move_to_start_position(joints_velocities, load_plan_response)
        if initial_movement_consumer is not None:
            async for move_to_response in movement_stream:
                initial_movement_consumer(move_to_response)

        # EXECUTE MOTION
        movement_controller_context = MovementControllerContext(
            combined_actions=CombinedActions(items=actions), motion_id=load_plan_response.motion
        )
        _movement_controller = movement_controller(movement_controller_context)
        await self._api_gateway.motion_api.execute_trajectory(self._cell, _movement_controller)

    async def get_state(self, tcp: str) -> wb.models.MotionGroupStateResponse:
        response = await self._api_gateway.motion_group_infos_api.get_current_motion_group_state(
            cell=self._cell, motion_group=self.motion_group_id, tcp=tcp
        )
        return response

    async def joints(self, tcp: str) -> wb.models.Joints:
        state = await self.get_state(tcp=tcp)
        return state.state.joint_position

    async def tcp_pose(self, tcp: str) -> Pose:
        state = await self.get_state(tcp=tcp)
        return Pose(state.state.tcp_pose)

    async def _get_number_of_joints(self) -> int:
        spec = await self._api_gateway.motion_group_infos_api.get_motion_group_specification(
            cell=self._cell, motion_group=self.motion_group_id
        )
        return len(spec.mechanical_joint_limits)

    async def _get_optimizer_setup(self, tcp: str) -> wb.models.OptimizerSetup:
        return await self._api_gateway.motion_group_infos_api.get_optimizer_configuration(
            cell=self._cell, motion_group=self._motion_group_id, tcp=tcp
        )

    async def _plan(self, actions: list[Action], tcp: str) -> wb.models.JointTrajectory:
        current_joints = await self.joints(tcp=tcp)
        robot_setup = await self._get_optimizer_setup(tcp=tcp)
        motion_commands = CombinedActions(items=actions).to_motion_command()

        request = wb.models.PlanTrajectoryRequest(
            robot_setup=robot_setup,
            motion_group=self.motion_group_id,
            start_joint_position=current_joints.joints,
            motion_commands=motion_commands,
            tcp=tcp,
        )

        motion_api_client = self._api_gateway.motion_api
        plan_response = await motion_api_client.plan_trajectory(
            cell=self._cell, plan_trajectory_request=request
        )

        if isinstance(
            plan_response.response.actual_instance, wb.models.PlanTrajectoryFailedResponse
        ):
            failed_response = plan_response.response.actual_instance
            raise PlanTrajectoryFailed(failed_response)

        return plan_response.response.actual_instance

    async def _load_planned_motion(
        self, joint_trajectory: wb.models.JointTrajectory
    ) -> wb.models.PlanSuccessfulResponse:
        load_plan_response = await self._api_gateway.motion_api.load_planned_motion(
            cell=self._cell,
            planned_motion=wb.models.PlannedMotion(
                motion_group=self.motion_group_id,
                times=joint_trajectory.times,
                joint_positions=joint_trajectory.joint_positions,
                locations=joint_trajectory.locations,
                tcp="Flange",
            ),
        )

        if (
            load_plan_response.plan_failed_on_trajectory_response is not None
            or load_plan_response.plan_failed_on_trajectory_response is not None
        ):
            raise LoadPlanFailed(load_plan_response)

        return load_plan_response.plan_successful_response

    async def move_to_start_position(
        self, joint_velocities, load_plan_response: LoadPlanResponse
    ) -> InitialMovementStream:
        limit_override = wb.models.LimitsOverride()
        if joint_velocities is not None:
            limit_override.joint_velocity_limits = wb.models.Joints(joints=joint_velocities)

        move_to_trajectory_stream = (
            self._api_gateway.motion_api.stream_move_to_trajectory_via_joint_ptp(
                cell=self._cell, motion=load_plan_response.motion, location_on_trajectory=0
            )
        )
        return move_to_trajectory_stream

    async def stop(self):
        logger.debug(f"Stopping motion of {self}...")
        try:
            await self._motion_api_client.stop_execution(
                cell=self._cell, motion=self.current_motion
            )
            logger.debug(f"Motion {self.current_motion} stopped.")
        except ValueError as e:
            logger.debug(f"No motion to stop for {self}: {e}")
