import wandelbots_api_client as wb
from loguru import logger

from nova.actions import Action, CombinedActions, MovementController, MovementControllerContext
from nova.core.exceptions import LoadPlanFailed, PlanTrajectoryFailed
from nova.core.movement_controller import move_forward
from nova.gateway import ApiGateway
from nova.types import InitialMovementConsumer, InitialMovementStream, LoadPlanResponse
from nova.types.pose import Pose

MAX_JOINT_VELOCITY_PREPARE_MOVE = 0.2
START_LOCATION_OF_MOTION = 0.0


class MotionGroup:
    def __init__(
        self, api_gateway: ApiGateway, cell: str, motion_group_id: str, is_activated: bool = False
    ):
        self._api_gateway = api_gateway
        self._motion_api_client = api_gateway.motion_api
        self._cell = cell
        self._motion_group_id = motion_group_id
        self._current_motion: str | None = None
        self._optimizer_setup: wb.models.OptimizerSetup | None = None
        self.is_activated = is_activated

    async def __aenter__(self):
        await self._api_gateway.motion_group_api.activate_motion_group(
            cell=self._cell, motion_group=self._motion_group_id
        )
        return self

    async def __aexit__(self, exc_type, exc_val, exc_tb):
        await self._api_gateway.motion_group_api.deactivate_motion_group(
            cell=self._cell, motion_group=self._motion_group_id
        )

    @property
    def motion_group_id(self) -> str:
        return self._motion_group_id

    @property
    def current_motion(self) -> str | None:
        # if not self._current_motion:
        #    raise ValueError("No MotionId attached. There is no planned motion available.")
        return self._current_motion

    async def plan(self, actions: list[Action] | Action, tcp: str) -> wb.models.JointTrajectory:
        """Plan a trajectory for the given actions

        Args:
            actions (list[Action] | Action): The actions to be planned. Can be a single action or a list of actions.
                Only motion actions are considered for planning.
            tcp (str): The identifier of the tool center point (TCP)

        Returns:
            wb.models.JointTrajectory: The planned joint trajectory
        """
        if not isinstance(actions, list):
            actions = [actions]

        if len(actions) == 0:
            raise ValueError("No actions provided")

        motion_commands = CombinedActions(items=tuple(actions)).to_motion_command()  # type: ignore
        joints = await self.joints()
        robot_setup = await self._get_optimizer_setup(tcp=tcp)
        request = wb.models.PlanTrajectoryRequest(
            robot_setup=robot_setup,
            start_joint_position=list(joints),
            motion_commands=motion_commands,
        )

        plan_trajectory_response = await self._motion_api_client.plan_trajectory(
            cell=self._cell, plan_trajectory_request=request
        )
        if isinstance(
            plan_trajectory_response.response.actual_instance,
            wb.models.PlanTrajectoryFailedResponse,
        ):
            raise PlanTrajectoryFailed(plan_trajectory_response.response.actual_instance)
        return plan_trajectory_response.response.actual_instance

    async def execute(
        self,
        joint_trajectory: wb.models.JointTrajectory,
        tcp: str,
        actions: list[Action] | Action | None = None,
        # collision_scene: dts.CollisionScene | None,
        movement_controller: MovementController = move_forward,
        initial_movement_consumer: InitialMovementConsumer | None = None,
    ):
        """Execute a planned motion

        Args:
            joint_trajectory (wb.models.JointTrajectory): The planned joint trajectory
            tcp (str): The identifier of the tool center point (TCP)
            actions (list[Action] | Action | None): The actions to be executed. Defaults to None.
            movement_controller (MovementController): The movement controller to be used. Defaults to move_forward
            initial_movement_consumer (InitialMovementConsumer): A consumer for the initial movement
        """
        if actions is None:
            actions = []
        elif not isinstance(actions, list):
            actions = [actions]

        load_plan_response = await self._load_planned_motion(joint_trajectory, tcp)

        number_of_joints = await self._get_number_of_joints()
        joints_velocities = [MAX_JOINT_VELOCITY_PREPARE_MOVE] * number_of_joints
        movement_stream = await self.move_to_start_position(joints_velocities, load_plan_response)
        async for move_to_response in movement_stream:
            if initial_movement_consumer is not None:
                initial_movement_consumer(move_to_response)

        movement_controller_context = MovementControllerContext(
            combined_actions=CombinedActions(items=tuple(actions)),  # type: ignore
            motion_id=load_plan_response.motion,
        )
        _movement_controller = movement_controller(movement_controller_context)
        await self._api_gateway.motion_api.execute_trajectory(self._cell, _movement_controller)

    async def plan_and_execute(self, actions: list[Action] | Action, tcp: str):
        joint_trajectory = await self.plan(actions, tcp)
        await self.execute(joint_trajectory, tcp, actions=actions)

    async def _get_number_of_joints(self) -> int:
        spec = await self._api_gateway.motion_group_infos_api.get_motion_group_specification(
            cell=self._cell, motion_group=self.motion_group_id
        )
        return len(spec.mechanical_joint_limits)

    async def _get_optimizer_setup(self, tcp: str) -> wb.models.OptimizerSetup:
        if self._optimizer_setup is None:
            self._optimizer_setup = (
                await self._api_gateway.motion_group_infos_api.get_optimizer_configuration(
                    cell=self._cell, motion_group=self._motion_group_id, tcp=tcp
                )
            )
        return self._optimizer_setup

    async def _load_planned_motion(
        self, joint_trajectory: wb.models.JointTrajectory, tcp: str
    ) -> wb.models.PlanSuccessfulResponse:
        load_plan_response = await self._api_gateway.motion_api.load_planned_motion(
            cell=self._cell,
            planned_motion=wb.models.PlannedMotion(
                motion_group=self.motion_group_id,
                times=joint_trajectory.times,
                joint_positions=joint_trajectory.joint_positions,
                locations=joint_trajectory.locations,
                tcp=tcp,
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

    async def get_state(self, tcp: str | None = None) -> wb.models.MotionGroupStateResponse:
        """Get the current state of the motion group

        Args:
            tcp (str): The identifier of the tool center point (TCP) to be used for tcp_pose in response. If not set,
                the flange pose is returned as tcp_pose.
        """
        response = await self._api_gateway.motion_group_infos_api.get_current_motion_group_state(
            cell=self._cell, motion_group=self.motion_group_id, tcp=tcp
        )
        return response

    async def joints(self) -> tuple:
        """Get the current joint positions"""
        state = await self.get_state()
        return tuple(state.state.joint_position.joints)

    async def tcp_pose(self, tcp: str | None = None) -> Pose:
        """Get the current TCP pose"""
        state = await self.get_state(tcp=tcp)
        return Pose(state.state.tcp_pose)

    async def tcps(self) -> list[wb.models.RobotTcp]:
        """Get the available tool center points (TCPs)"""
        response = await self._api_gateway.motion_group_infos_api.list_tcps(
            cell=self._cell, motion_group=self.motion_group_id
        )
        return response.tcps

    async def tcp_names(self) -> list[str]:
        """Get the names of the available tool center points (TCPs)"""
        return [tcp.id for tcp in await self.tcps()]
