import asyncio
from typing import AsyncIterable, cast

import wandelbots_api_client as wb

from nova.actions import Action, CombinedActions, MovementController, MovementControllerContext
from nova.actions.motions import CollisionFreeMotion, Motion
from nova.api import models
from nova.core import logger
from nova.core.exceptions import InconsistentCollisionScenes, LoadPlanFailed, PlanTrajectoryFailed
from nova.core.gateway import ApiGateway
from nova.core.movement_controller import move_forward
from nova.core.robot_cell import AbstractRobot
from nova.types import InitialMovementStream, LoadPlanResponse, MovementResponse, Pose, RobotState
from nova.utils import StreamExtractor

MAX_JOINT_VELOCITY_PREPARE_MOVE = 0.2
START_LOCATION_OF_MOTION = 0.0


def compare_collision_scenes(scene1: wb.models.CollisionScene, scene2: wb.models.CollisionScene):
    if scene1.colliders != scene2.colliders:
        return False

    # Compare motion groups
    if scene1.motion_groups != scene2.motion_groups:
        return False

    return True


# TODO: when collision scene is different in different motions
#  , we should plan them separately
def split_actions_into_batches(actions: list[Action]) -> list[list[Action]]:
    """
    Splits the list of actions into batches of actions and collision free motions.
    Actions are sent to plan_trajectory API and collision free motions are sent to plan_collision_free_ptp API.
    """
    batches: list[list[Action]] = []
    for action in actions:
        if (
            # Start a new batch if:
            not batches  # first action no batches yet
            or isinstance(action, CollisionFreeMotion)
            or isinstance(batches[-1][-1], CollisionFreeMotion)
        ):
            batches.append([action])
        else:
            batches[-1].append(action)
    return batches


def combine_trajectories(
    trajectories: list[wb.models.JointTrajectory],
) -> wb.models.JointTrajectory:
    """
    Combines multiple trajectories into one trajectory.
    """
    final_trajectory = trajectories[0]
    current_end_time = final_trajectory.times[-1]
    current_end_location = final_trajectory.locations[-1]

    for trajectory in trajectories[1:]:
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


class MotionGroup(AbstractRobot):
    def __init__(self, api_gateway: ApiGateway, cell: str, motion_group_id: str):
        self._api_gateway = api_gateway
        self._motion_api_client = api_gateway.motion_api
        self._cell = cell
        self._motion_group_id = motion_group_id
        self._current_motion: str | None = None
        self._optimizer_setup: wb.models.OptimizerSetup | None = None
        super().__init__(id=motion_group_id)

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

    async def _plan_with_collision_check(
        self,
        actions: list[Action],
        tcp: str,
        start_joint_position: tuple[float, ...] | None = None,
        optimizer_setup: wb.models.OptimizerSetup | None = None,
    ) -> wb.models.JointTrajectory:
        """
        This method plans a trajectory and checks for collisions.
        The collision check only happens if the actions have collision scene data.

        You must provide the exact same collision data into all the actions.
        Because the underlying API supports collision checks for the whole trajectory only.

        Raises:
            InconsistentCollisionScenes: If the collision scene data is not consistent across all actions

            Your actions should follow below rules to be considered consistent:
            1- They all should have the same collision scene data
            2- They all should have no collision data

            PlanTrajectoryFailed: If the trajectory planning failed including the collision check

        For more information about this API, please refer to the plan_trajectory in the API documentation.

        Args:
            actions: list of actions to plan, current supported actions are Motion and WriteActions
                     WriteAction you specify on your path is handled in a performant way.
                     Please check execute_trajectory.motion_command.set_io for more information.
            tcp:     The tool to use
            start_joint_position: The starting joint position, if none provided, current position of the robot is used
            optimizer_setup: The optimizer setup

        Returns: planned joint trajectory

        """
        # PREPARE THE REQUEST
        collision_scenes = self._validate_collision_scenes(actions)
        start_joint_position = start_joint_position or await self.joints()
        robot_setup = optimizer_setup or await self._get_optimizer_setup(tcp=tcp)

        motion_commands = CombinedActions(items=tuple(actions)).to_motion_command()  # type: ignore

        static_colliders = None
        collision_motion_group = None
        if collision_scenes and len(collision_scenes) > 0:
            static_colliders = collision_scenes[0].colliders

            motion_group_type = robot_setup.motion_group_type
            if (
                collision_scenes[0].motion_groups
                and motion_group_type in collision_scenes[0].motion_groups
            ):
                collision_motion_group = collision_scenes[0].motion_groups[motion_group_type]

        request = wb.models.PlanTrajectoryRequest(
            robot_setup=robot_setup,
            start_joint_position=list(start_joint_position),
            motion_commands=motion_commands,
            static_colliders=static_colliders,
            collision_motion_group=collision_motion_group,
        )

        # EXECUTE THE API CALL
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

    def _validate_collision_scenes(self, actions: list[Action]) -> list[models.CollisionScene]:
        motion_count = len([action for action in actions if isinstance(action, Motion)])
        collision_scenes = [
            action.collision_scene
            for action in actions
            if isinstance(action, CollisionFreeMotion) and action.collision_scene is not None
        ]

        if len(collision_scenes) != 0 and len(collision_scenes) != motion_count:
            raise InconsistentCollisionScenes(
                "Only some of the actions have collision scene. Either specify it for all or none."
            )

        # If a collision scene is provided, the same should be provided for all the collision scene
        if len(collision_scenes) > 1:
            first_scene = collision_scenes[0]
            if not all(
                compare_collision_scenes(first_scene, scene) for scene in collision_scenes[1:]
            ):
                raise InconsistentCollisionScenes(
                    "All actions must use the same collision scene but some are different"
                )

        return collision_scenes

    # TODO: we get the optimizer setup from as an input because
    #  it has a velocity setting which is used in collision free movement, I need to double check this
    async def _plan_collision_free(
        self,
        action: CollisionFreeMotion,
        tcp: str,
        start_joint_position: list[float],
        optimizer_setup: wb.models.OptimizerSetup | None = None,
    ) -> wb.models.JointTrajectory:
        """
        This method plans a trajectory and avoids collisions.
        This means if there is a collision along the way to the target pose or joint positions,
        It will adjust the trajectory to avoid the collision.

        The collision check only happens if the action have collision scene data.

        For more information about this API, please refer to the plan_collision_free_ptp in the API documentation.

        Args:
            action: The target pose or joint positions to reach
            tcp:     The tool to use
            start_joint_position: The starting joint position, if none provided, current position of the robot is used
            optimizer_setup: The optimizer setup

        Returns: planned joint trajectory


        """
        target = wb.models.PlanCollisionFreePTPRequestTarget(
            **action.model_dump(exclude_unset=True)
        )
        robot_setup = optimizer_setup or await self._get_optimizer_setup(tcp=tcp)

        static_colliders = None
        collision_motion_group = None
        collision_scene = action.collision_scene
        if collision_scene and collision_scene.colliders:
            static_colliders = collision_scene.colliders

            if (
                collision_scene.motion_groups
                and robot_setup.motion_group_type in collision_scene.motion_groups
            ):
                collision_motion_group = collision_scene.motion_groups[
                    robot_setup.motion_group_type
                ]

        request: wb.models.PlanCollisionFreePTPRequest = wb.models.PlanCollisionFreePTPRequest(
            robot_setup=robot_setup,
            start_joint_position=start_joint_position,
            target=target,
            static_colliders=static_colliders,
            collision_motion_group=collision_motion_group,
        )

        plan_result = await self._motion_api_client.plan_collision_free_ptp(
            cell=self._cell, plan_collision_free_ptp_request=request
        )

        if isinstance(plan_result.response.actual_instance, wb.models.PlanTrajectoryFailedResponse):
            raise PlanTrajectoryFailed(plan_result.response.actual_instance)
        return plan_result.response.actual_instance

    async def _plan(
        self,
        actions: list[Action],
        tcp: str,
        start_joint_position: tuple[float, ...] | None = None,
        optimizer_setup: wb.models.OptimizerSetup | None = None,
    ) -> wb.models.JointTrajectory:
        if not actions:
            raise ValueError("No actions provided")

        current_joints = start_joint_position or await self.joints()
        robot_setup = optimizer_setup or await self._get_optimizer_setup(tcp=tcp)

        all_trajectories = []
        for batch in split_actions_into_batches(actions):
            if len(batch) == 0:
                raise ValueError("Empty batch of actions")

            if isinstance(batch[0], CollisionFreeMotion):
                motion: CollisionFreeMotion = cast(CollisionFreeMotion, batch[0])
                trajectory = await self._plan_collision_free(
                    action=motion,
                    tcp=tcp,
                    start_joint_position=list(current_joints),
                    optimizer_setup=robot_setup,
                )
                all_trajectories.append(trajectory)
                # the last joint position of this trajectory is the starting point for the next one
                current_joints = tuple(trajectory.joint_positions[-1].joints)
            else:
                trajectory = await self._plan_with_collision_check(
                    actions=batch,
                    tcp=tcp,
                    start_joint_position=current_joints,
                    optimizer_setup=robot_setup,
                )
                all_trajectories.append(trajectory)
                # the last joint position of this trajectory is the starting point for the next one
                current_joints = tuple(trajectory.joint_positions[-1].joints)

        return combine_trajectories(all_trajectories)

    # TODO: refactor and simplify code, tests are already there
    # TODO: split into batches when the collision scene changes in a batch of collision free motions

    async def _execute(
        self,
        joint_trajectory: wb.models.JointTrajectory,
        tcp: str,
        actions: list[Action],
        movement_controller: MovementController | None,
    ) -> AsyncIterable[MovementResponse]:
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

            yield move_to_response

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
        execution_task = asyncio.create_task(
            self._api_gateway.motion_api.execute_trajectory(
                self._cell, execute_response_streaming_controller
            )
        )
        async for execute_response in execute_response_streaming_controller:
            yield execute_response
        await execution_task

    async def _get_number_of_joints(self) -> int:
        spec = await self._api_gateway.motion_group_infos_api.get_motion_group_specification(
            cell=self._cell, motion_group=self.motion_group_id
        )
        return len(spec.mechanical_joint_limits)

    async def _get_optimizer_setup(self, tcp: str) -> wb.models.OptimizerSetup:
        # TODO: mypy failed on main branch, need to check
        if self._optimizer_setup is None or self._optimizer_setup.tcp != tcp:  # type: ignore
            self._optimizer_setup = (
                await self._api_gateway.motion_group_infos_api.get_optimizer_configuration(
                    cell=self._cell, motion_group=self._motion_group_id, tcp=tcp
                )
            )
        # TODO: mypy failed on code from main branch need to check
        return self._optimizer_setup  # type: ignore

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
