from typing import Callable

import wandelbots_api_client as wb
from loguru import logger

from nova.actions import (
    Action,
    CollisionFreeMotion,
    CombinedActions,
    Motion,
    MovementController,
    MovementControllerContext,
)
from nova.api import models
from nova.core.exceptions import InconsistentCollisionScenes, LoadPlanFailed, PlanTrajectoryFailed
from nova.core.movement_controller import motion_group_state_to_motion_state, move_forward
from nova.core.robot_cell import AbstractRobot
from nova.gateway import ApiGateway
from nova.types import InitialMovementStream, LoadPlanResponse, MotionState, Pose, RobotState

MAX_JOINT_VELOCITY_PREPARE_MOVE = 0.2
START_LOCATION_OF_MOTION = 0.0


def compare_collition_scenes(scene1: wb.models.CollisionScene, scene2: wb.models.CollisionScene):
    if scene1.colliders != scene2.colliders:
        return False

    # Compare motion groups
    if scene1.motion_groups != scene2.motion_groups:
        return False

    return True


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

    async def _plan(
        self,
        actions: list[Action],
        tcp: str,
        start_joint_position: tuple[float, ...] | None = None,
        optimizer_setup: wb.models.OptimizerSetup | None = None,
    ) -> wb.models.JointTrajectory:
        motion_commands = CombinedActions(items=tuple(actions)).to_motion_command()  # type: ignore

        collision_scenes: list[models.CollisionScene] = []
        motion_counter = 0
        for action in actions:
            if isinstance(action, Motion):
                motion_action: Motion = action
                motion_counter = motion_counter + 1
                if motion_action.collision_scene is not None:
                    collision_scenes.append(motion_action.collision_scene)

        # If a collision scene is provided, the same should be provided for all the collision scene
        # TODO: should we maybe use the first collision scene for all the motions? rather than giving error
        if len(collision_scenes) != 0 and len(collision_scenes) != motion_counter:
            raise InconsistentCollisionScenes("All actions must use the same collision scene")

        if len(collision_scenes) > 1:
            first_scene = collision_scenes[0]
            if not all(
                compare_collition_scenes(first_scene, scene) for scene in collision_scenes[1:]
            ):
                raise InconsistentCollisionScenes("All actions must use the same collision scene")

        if start_joint_position is None:
            start_joint_position = await self.joints()

        # Get optimizer setup
        robot_setup = optimizer_setup or await self._get_optimizer_setup(tcp=tcp)

        request = wb.models.PlanTrajectoryRequest(
            robot_setup=robot_setup,
            start_joint_position=list(start_joint_position),
            motion_commands=motion_commands,
        )

        # Only add collision scene data if available
        if collision_scenes and collision_scenes[0]:
            request.static_colliders = collision_scenes[0].colliders

            # Only add motion group if available
            if (
                collision_scenes[0].motion_groups
                and robot_setup.motion_group_type in collision_scenes[0].motion_groups
            ):
                request.collision_motion_group = collision_scenes[0].motion_groups[
                    robot_setup.motion_group_type
                ]

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

    # TODO: add this to the abstract robot
    async def _plan_collision_free(
        self,
        action: CollisionFreeMotion,
        tcp: str,
        start_joint_position: list[float],
        optimizer_setup: wb.models.OptimizerSetup | None = None,
    ) -> wb.models.JointTrajectory:
        """Plan collision-free PTP action.

        Args:
            action: The collision-free motion action to plan
            robot_setup: Robot optimizer configuration
            start_joints: Starting joint positions

        Returns:
            Planned joint trajectory
        """
        target_request = wb.models.PlanCollisionFreePTPRequestTarget(
            action.target._to_wb_pose2() if isinstance(action.target, Pose) else action.target
        )

        # Get optimizer setup
        robot_setup = optimizer_setup or await self._get_optimizer_setup(tcp=tcp)

        request: wb.models.PlanCollisionFreePTPRequest = wb.models.PlanCollisionFreePTPRequest(
            robot_setup=robot_setup,
            start_joint_position=start_joint_position,
            target=target_request,
        )

        # Only add collision scene data if available
        if action.collision_scene and action.collision_scene.colliders:
            collision_scene = action.collision_scene
            request.static_colliders = collision_scene.colliders

            # Only add motion group if available
            if (
                collision_scene.motion_groups
                and robot_setup.motion_group_type in collision_scene.motion_groups
            ):
                request.collision_motion_group = collision_scene.motion_groups[
                    robot_setup.motion_group_type
                ]

        plan_result = await self._motion_api_client.plan_collision_free_ptp(
            cell=self._cell, plan_collision_free_ptp_request=request
        )

        if isinstance(plan_result.response.actual_instance, wb.models.PlanTrajectoryFailedResponse):
            raise PlanTrajectoryFailed(plan_result.response.actual_instance)
        return plan_result.response.actual_instance

    async def _plan_combined(
        self,
        actions: list[Action | CollisionFreeMotion],
        tcp: str,
        start_joint_position: list[float],
        optimizer_setup: wb.models.OptimizerSetup | None = None,
    ) -> wb.models.JointTrajectory:
        if not actions:
            raise ValueError("No actions provided")

        current_joints: tuple[float, ...] = tuple(
            start_joint_position if start_joint_position is not None else await self.joints()
        )
        robot_setup = await self._get_optimizer_setup(tcp=tcp)

        # Separate CollisionFreePTP actions from other actions
        current_batch: list[Action] = []
        all_trajectories: list[wb.models.JointTrajectory] = []

        for action in actions:
            if isinstance(action, CollisionFreeMotion):
                # Plan current batch if not empty
                if current_batch:
                    trajectory = await self._plan(
                        current_batch, tcp, current_joints, optimizer_setup=optimizer_setup
                    )
                    all_trajectories.append(trajectory)
                    current_joints = tuple(trajectory.joint_positions[-1].joints)
                    current_batch = []

                # Plan collision-free action
                trajectory = await self._plan_collision_free(action, tcp, list(current_joints))
                all_trajectories.append(trajectory)
                current_joints = tuple(trajectory.joint_positions[-1].joints)
            else:
                current_batch.append(action)

        # Plan remaining batch if not empty
        if current_batch:
            trajectory = await self._plan(current_batch, tcp, current_joints, robot_setup)
            all_trajectories.append(trajectory)

        # Combine all trajectories
        final_trajectory = all_trajectories[0]
        current_end_time = final_trajectory.times[-1]
        current_end_location = final_trajectory.locations[-1]

        for trajectory in all_trajectories[1:]:
            # Shift times and locations to continue from last endpoint
            shifted_times = [t + current_end_time for t in trajectory.times[1:]]  # Skip first point
            shifted_locations = [
                location + current_end_location for location in trajectory.locations[1:]
            ]  # Skip first point

            final_trajectory.times.extend(shifted_times)
            final_trajectory.joint_positions.extend(trajectory.joint_positions[1:])
            final_trajectory.locations.extend(shifted_locations)

            current_end_time = final_trajectory.times[-1]
            current_end_location = final_trajectory.locations[-1]

        return final_trajectory

    async def _execute(
        self,
        joint_trajectory: wb.models.JointTrajectory,
        tcp: str,
        actions: list[Action],
        on_movement: Callable[[MotionState | None], None],
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
            on_movement(motion_state)

        controller = movement_controller(
            MovementControllerContext(
                combined_actions=CombinedActions(items=tuple(actions)),  # type: ignore
                motion_id=load_plan_response.motion,
            ),
            on_movement=on_movement,
        )

        await self._api_gateway.motion_api.execute_trajectory(self._cell, controller)

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
