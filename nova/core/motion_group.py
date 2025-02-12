import asyncio
from typing import Callable

import wandelbots_api_client as wb
from loguru import logger

from nova.actions import Action, CombinedActions, MovementController, MovementControllerContext
from nova.core.exceptions import LoadPlanFailed, PlanTrajectoryFailed
from nova.core.movement_controller import motion_group_state_to_motion_state, move_forward
from nova.core.robot_cell import AbstractRobot
from nova.gateway import ApiGateway
from nova.types import InitialMovementStream, LoadPlanResponse, MotionState, Pose, RobotState
from nova.utils import StreamExtractor

MAX_JOINT_VELOCITY_PREPARE_MOVE = 0.2
START_LOCATION_OF_MOTION = 0.0


class MotionGroup(AbstractRobot):
    def __init__(self, api_gateway: ApiGateway, cell: str, motion_group_id: str):
        self._api_gateway = api_gateway
        self._motion_api_client = api_gateway.motion_api
        self._cell = cell
        self._motion_group_id = motion_group_id
        self._current_motion: str | None = None
        self._optimizer_setup: wb.models.OptimizerSetup | None = None
        super().__init__()

    async def open(self):
        await self._api_gateway.motion_group_api.activate_motion_group(
            cell=self._cell, motion_group=self._motion_group_id
        )
        return self

    async def close(self):
        # RPS-1174: when a motion group is deactivated, RAE closes all open connections
        #           this behaviour is not desired in some cases,
        #           so for now we will not deactivate for the user
        pass

    @property
    def motion_group_id(self) -> str:
        return self._motion_group_id

    @property
    def current_motion(self) -> str | None:
        # if not self._current_motion:
        #    raise ValueError("No MotionId attached. There is no planned motion available.")
        return self._current_motion

    async def _plan(self, actions: list[Action], tcp: str) -> wb.models.JointTrajectory:
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
            # TODO: handle partially executable path
            raise PlanTrajectoryFailed(plan_trajectory_response.response.actual_instance)
        return plan_trajectory_response.response.actual_instance

    async def _execute(
        self,
        joint_trajectory: wb.models.JointTrajectory,
        tcp: str,
        actions: list[Action],
        movement_controller: MovementController | None,
    ):
        if movement_controller is None:
            movement_controller = move_forward

        # Load planned trajectory
        load_plan_response = await self._load_planned_motion(joint_trajectory, tcp)

        # Move to start position
        number_of_joints = await self._get_number_of_joints()
        joints_velocities = [MAX_JOINT_VELOCITY_PREPARE_MOVE] * number_of_joints
        movement_stream = await self.move_to_start_position(joints_velocities, load_plan_response)

        # If there's an initial consumer, feed it the data
        async for move_to_response in movement_stream:
            # TODO: refactor
            if (
                move_to_response.state is None
                or move_to_response.state.motion_groups is None
                or len(move_to_response.state.motion_groups) == 0
                or move_to_response.move_response is None
                or move_to_response.move_response.current_location_on_trajectory is None
            ):
                continue

            # TODO: maybe 1-...
            motion_state = motion_group_state_to_motion_state(
                move_to_response.state.motion_groups[0],
                float(move_to_response.move_response.current_location_on_trajectory),
            )
            yield motion_state

        controller = movement_controller(
            MovementControllerContext(
                combined_actions=CombinedActions(items=tuple(actions)),  # type: ignore
                motion_id=load_plan_response.motion,
            )
        )
        def stop_condition(response: wb.models.ExecuteTrajectoryResponse) -> bool:
            instance = response.actual_instance
            # Stop when standstill indicates motion ended
            return (
                isinstance(instance, wb.models.Standstill)
                and instance.standstill.reason == wb.models.StandstillReason.REASON_MOTION_ENDED
            )
        
        execute_response_streaming_controller = StreamExtractor(controller, stop_condition)
        execution_task = asyncio.create_task(self._api_gateway.motion_api.execute_trajectory(self._cell, execute_response_streaming_controller))
        async for execute_resposne in execute_response_streaming_controller:
            yield execute_resposne
        await execution_task

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

    async def get_state(self, tcp: str | None = None) -> RobotState:
        response = await self._api_gateway.motion_group_infos_api.get_current_motion_group_state(
            cell=self._cell, motion_group=self.motion_group_id, tcp=tcp
        )
        return RobotState(
            pose=Pose(response.state.tcp_pose), joints=tuple(response.state.joint_position.joints)
        )

    async def joints(self) -> tuple:
        state = await self.get_state()
        if state.joints is None:
            raise ValueError(
                f"No joint positions available for motion group {self._motion_group_id}"
            )
        return state.joints

    async def tcp_pose(self, tcp: str | None = None) -> Pose:
        state = await self.get_state(tcp=tcp)
        return state.pose

    async def tcps(self) -> list[wb.models.RobotTcp]:
        """Get the available tool center points (TCPs)"""
        response = await self._api_gateway.motion_group_infos_api.list_tcps(
            cell=self._cell, motion_group=self.motion_group_id
        )
        return response.tcps

    async def tcp_names(self) -> list[str]:
        """Get the names of the available tool center points (TCPs)"""
        return [tcp.id for tcp in await self.tcps()]

    async def active_tcp(self) -> wb.models.RobotTcp:
        active_tcp = await self._api_gateway.motion_group_infos_api.get_active_tcp(
            cell=self._cell, motion_group=self.motion_group_id
        )
        return active_tcp

    async def active_tcp_name(self) -> str:
        active_tcp = await self.active_tcp()
        return active_tcp.id
