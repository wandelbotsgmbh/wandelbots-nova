import asyncio
import logging
from contextlib import aclosing
from typing import AsyncGenerator, cast

import numpy as np

from nova import api
from nova.actions import Action, CombinedActions, MovementController, MovementControllerContext
from nova.actions.mock import WaitAction
from nova.actions.motions import CollisionFreeMotion
from nova.core.gateway import ApiGateway
from nova.exceptions import LoadPlanFailed, PlanTrajectoryFailed
from nova.types import Pose, RobotState
from nova.types.state import MotionState, motion_group_state_to_motion_state
from nova.utils.collision_setup import (
    get_joint_position_limits_from_motion_group_setup,
    get_safety_collision_setup_from_motion_group_description,
    motion_group_setup_from_motion_group_description,
    validate_collision_setups,
)
from nova.utils.joint_trajectory import combine_trajectories
from nova.utils.motion_group_settings import update_motion_group_setup_with_motion_settings

from .movement_controller import move_forward
from .robot_cell import AbstractRobot

MAX_JOINT_VELOCITY_PREPARE_MOVE = 0.2
START_LOCATION_OF_MOTION = 0.0


logger = logging.getLogger(__name__)


# TODO: when collision scene is different in different motions
#  , we should plan them separately
def split_actions_into_batches(actions: list[Action]) -> list[list[Action]]:
    """
    Splits the list of actions into batches of actions, collision free motions and waits.
    Actions are sent to plan_trajectory API and collision free motions are sent to plan_collision_free_ptp API.
    Waits generate a trajectory with the same start and end position.
    """
    batches: list[list[Action]] = []
    for action in actions:
        if (
            # Start a new batch if:
            not batches  # first action no batches yet
            or isinstance(action, CollisionFreeMotion)
            or isinstance(batches[-1][-1], CollisionFreeMotion)
            or isinstance(action, WaitAction)
            or isinstance(batches[-1][-1], WaitAction)
        ):
            batches.append([action])
        else:
            batches[-1].append(action)
    return batches


def _find_shortest_distance(
    start_joint_positions: tuple[float, ...], solutions: list[tuple[float, ...]]
) -> tuple[float, ...]:
    smallest_distance = float("inf")
    for solution in solutions:
        distance = np.linalg.norm(np.array(solution) - np.array(start_joint_positions))
        logger.info(f"IK solution: {solution}, distance from start: {distance}")
        if distance < smallest_distance:
            smallest_distance = float(distance)
            target_joint_positions = solution
    return target_joint_positions


class MotionGroup(AbstractRobot):
    """Manages motion planning and execution within a specified motion group."""

    def __init__(self, api_client: ApiGateway, cell: str, controller_id: str, motion_group_id: str):
        """
        Initializes a new MotionGroup instance.

        Args:
            api_client (ApiGateway): The API gateway through which motion commands are sent.
            cell (str): The name or identifier of the robotic cell.
            motion_group_id (str): The identifier of the motion group.
        """
        self._api_client = api_client
        self._cell = cell
        self._controller_id = controller_id
        self._motion_group_id = motion_group_id
        self._current_motion: str | None = None
        super().__init__(id=motion_group_id)

    @property
    def id(self) -> str:
        """The unique identifier for this motion group in the shape "motion_group_id@controller_id" e.g. "0@ur10e".

        Returns:
            str: The unique identifier for this motion group.
        """
        return self._motion_group_id

    @property
    def current_motion(self) -> str | None:
        return self._current_motion

    # TODO: does this needs to be cached?
    async def _fetch_motion_group_description(self) -> api.models.MotionGroupDescription:
        return await self._api_client.motion_group_api.get_motion_group_description(
            cell=self._cell, controller=self._controller_id, motion_group=self.id
        )

    async def get_description(self) -> api.models.MotionGroupDescription:
        """Get the motion group description.

        Returns:
            api.models.MotionGroupDescription: The motion group description.
        """
        return await self._fetch_motion_group_description()

    async def get_model(self) -> str:
        """Get the motion group model.

        Returns:
            api.models.MotionGroupModel: The motion group model.
        """
        motion_group_description = await self._fetch_motion_group_description()
        return motion_group_description.motion_group_model.root

    async def get_setup(self, tcp_name: str | None = None) -> api.models.MotionGroupSetup:
        """Get the motion group setup.

        Args:
            tcp_name (str): The TCP to get the setup for.

        Returns:
            api.models.MotionGroupSetup: The motion group setup.
        """
        # TODO allow to specify payload
        motion_group_description = await self._fetch_motion_group_description()
        return motion_group_setup_from_motion_group_description(
            motion_group_description=motion_group_description, tcp_name=tcp_name
        )

    async def get_mounting(self) -> Pose | None:
        """Get the mounting of the motion group.

        Returns:
            Pose | None: The mounting of the motion group. None if not available.
        """
        motion_group_description = await self._fetch_motion_group_description()
        return (
            Pose(motion_group_description.mounting)
            if motion_group_description.mounting is not None
            else None
        )

    async def get_safety_collision_setup(self, tcp: str) -> api.models.CollisionSetup:
        """Get the safety collision setup of the motion group.

        Returns:
            api.models.CollisionSetup: The safety collision setup of the motion group.
        """
        motion_group_description = await self._fetch_motion_group_description()
        return get_safety_collision_setup_from_motion_group_description(
            motion_group_description=motion_group_description, tcp_name=tcp
        )

    async def get_default_collision_link_chain(self) -> api.models.LinkChain:
        description = await self._fetch_motion_group_description()
        collision_model = (
            await self._api_client.motion_group_models_api.get_motion_group_collision_model(
                motion_group_model=description.motion_group_model.root
            )
        )

        return api.models.LinkChain([api.models.Link(link) for link in collision_model])

    # TODO: check the response type, it is not easy to use
    # API returns list of list of list of float ( 3 inner lists )
    async def _inverse_kinematics(
        self,
        poses: list[Pose],
        tcp: str,
        motion_group_setup: api.models.MotionGroupSetup | None = None,
    ) -> list[list[tuple[float, ...]]]:
        """Do inverse kinematics for the given poses for the motion group.

        This API will return all found solutions for each requested pose in a list.
        This list can contain multiple joint positions for each pose or be empty if no solution is found.


        Args:
            poses (list[Pose]): The target poses for which to calculate joint positions.
            tcp (str): The TCP to use for the calculations.
            collision_setup (api.models.CollisionSetup | None): Collision setup to use for the inverse kinematics calculation.
                When provided, the calculation will avoid configurations that lead to collisions.
                If link_chain or tool are not specified in the collision setup, they will be automatically populated
                from the motion group's default collision setup to ensure robot and tool geometry are included.
                Check `nova.utils.collision_setup.motion_group_setup_from_motion_group_description` for default collision setups used for the inverse kinematics calculation.

        Returns:
            list[list[tuple[float, ...]]]: All found joint position solutions for each pose. The outer list corresponds to each pose,
                                            and the inner list contains each valid joint configuration solution found for that pose.
        """
        motion_group_setup = motion_group_setup or await self.get_setup(tcp)

        tcp_offset = await self.tcp_offset(tcp)
        motion_group_model = await self.get_model()
        mounting = await self.get_mounting()

        joint_position_limits = get_joint_position_limits_from_motion_group_setup(
            motion_group_setup
        )

        response = await self._api_client.kinematics_api.inverse_kinematics(
            cell=self._cell,
            inverse_kinematics_request=api.models.InverseKinematicsRequest(
                motion_group_model=api.models.MotionGroupModel(motion_group_model),
                tcp_poses=[pose.to_api_model() for pose in poses],
                tcp_offset=tcp_offset.to_api_model(),
                mounting=mounting.to_api_model() if mounting is not None else None,
                joint_position_limits=joint_position_limits,
                collision_setups=motion_group_setup.collision_setups,
            ),
        )
        return response.joints

    async def forward_kinematics(self, joints: list[tuple[float, ...]], tcp: str) -> list[Pose]:
        """Get the forward kinematics of the motion group.

        Returns:
            list[Pose]: The forward kinematics of the motion group. Empty list if not available.
        """
        if len(joints) == 0:
            raise ValueError("Provide at least one joint configuration")

        joint_positions = [api.models.DoubleArray(list(joint_config)) for joint_config in joints]

        tcp_offset = await self.tcp_offset(tcp)
        motion_group_model = await self.get_model()
        mounting = await self.get_mounting()

        response = await self._api_client.kinematics_api.forward_kinematics(
            cell=self._cell,
            forward_kinematics_request=api.models.ForwardKinematicsRequest(
                motion_group_model=api.models.MotionGroupModel(motion_group_model),
                joint_positions=joint_positions,
                tcp_offset=tcp_offset.to_api_model(),
                mounting=mounting.to_api_model() if mounting is not None else None,
            ),
        )

        if len(response.tcp_poses) == 0:
            raise ValueError("No TCP poses returned from forward kinematics")

        return [Pose(tcp_pose) for tcp_pose in response.tcp_poses]

    async def open(self):
        # TODO if there is no explicit motion group activation, what should we do here?
        # maybe we set the mode to control mode? But this is not needed (implicitly done by the trajectory execution)
        return self

    async def close(self):
        # RPS-1174: when a motion group is deactivated, RAE closes all open connections
        #           this behaviour is not desired in some cases,
        #           so for now we will not deactivate for the user
        pass

    # TODO: should we remove this until we fix it?
    async def stop(self):
        """Stop the motion group.

        Raises:
            ValueError: If no motion to stop.
        """
        logger.debug(f"Stopping motion of {self}...")
        try:
            if self._current_motion is None:
                raise ValueError("No motion to stop")
            await self._api_client.motion_api.stop_execution(
                cell=self._cell, motion=self._current_motion
            )
            logger.debug(f"Motion {self.current_motion} stopped.")
        except ValueError as e:
            logger.debug(f"No motion to stop for {self}: {e}")

    async def get_state(self, tcp: str | None = None) -> RobotState:
        """
        Returns the motion group state.
        Args:
            tcp (str | None): The reference TCP for the cartesian pose part of the robot state. Defaults to None.
                                        If None, the current active/selected TCP of the motion group is used.
        """
        motion_group_state = await self._fetch_state()
        if tcp is None or tcp == motion_group_state.tcp:
            tcp = motion_group_state.tcp
            pose = Pose(motion_group_state.tcp_pose)
        else:
            tcps = await self.tcps()
            tcp_offset = Pose(position=tcps[tcp].position, orientation=tcps[tcp].orientation)
            pose = Pose(motion_group_state.flange_pose) @ tcp_offset
        return RobotState(pose=pose, tcp=tcp, joints=tuple(motion_group_state.joint_position))

    async def stream_state(
        self, response_rate_msecs: int | None = None
    ) -> AsyncGenerator[api.models.MotionGroupState, None]:
        """
        Streams the motion group state continuously.

        This method provides a real-time stream of robot state information including
        joint positions and TCP pose data for the motion group.
        Args:
            response_rate_msecs (int | None): The rate at which state updates are streamed
                                             in milliseconds. Defaults to None for maximum rate.
        """
        response_stream = self._api_client.motion_group_api.stream_motion_group_state(
            cell=self._cell,
            controller=self._controller_id,
            motion_group=self.id,
            response_rate=response_rate_msecs,
        )

        async with aclosing(response_stream) as response_stream:
            async for response in response_stream:
                yield response

    async def joints(self) -> tuple[float, ...]:
        """Returns the current joint positions of the motion group."""
        return (await self.get_state()).joints

    async def tcp_pose(self, tcp: str | None = None) -> Pose:
        """
        Returns the current TCP pose of the motion group.
        Args:
            tcp (str | None): The reference TCP for the returned pose. Defaults to None.
                                If None, the current active/selected TCP of the motion group is used.
        """
        return (await self.get_state(tcp=tcp)).pose

    @staticmethod
    def _tcps_as_dict(
        tcps: dict[str, api.models.RobotTcp] | list[api.models.RobotTcp],
    ) -> dict[str, api.models.RobotTcp]:
        if isinstance(tcps, dict):
            return tcps
        return {tcp.id: tcp for tcp in tcps}

    async def tcp_offset(self, tcp: str) -> Pose:
        motion_group_description = await self._fetch_motion_group_description()
        tcps = motion_group_description.tcps
        if tcps is None:
            raise ValueError("No TCPs found in motion group description")
        return Pose(tcps[tcp].pose)

    async def tcps(self) -> dict[str, api.models.RobotTcp]:
        motion_group_description = await self._fetch_motion_group_description()
        tcps = motion_group_description.tcps
        if tcps is None:
            return {}
        return {
            tcp: api.models.RobotTcp(
                id=tcp,
                name=tcp_offset.name,
                position=tcp_offset.pose.position,
                # TODO: what is the correct rotation type here then?
                orientation=api.models.Orientation(tcp_offset.pose.orientation.root)
                if tcp_offset.pose.orientation is not None
                else None,
            )
            for tcp, tcp_offset in tcps.items()
            if tcp_offset.pose.position is not None
        }

    # TODO names?, ids?, both?, whatever? (probably ids atm)
    async def tcp_names(self) -> list[str]:
        tcps = await self.tcps()
        return list(tcps.keys())

    async def active_tcp_name(self) -> str | None:
        return (await self._fetch_state()).tcp

    async def active_tcp(self) -> api.models.RobotTcp | None:
        active_tcp_name = await self.active_tcp_name()
        if active_tcp_name is None:
            return None
        tcps = await self.tcps()
        return tcps.get(active_tcp_name)

    async def ensure_virtual_tcp(
        self, tcp: api.models.RobotTcp, timeout: int = 12
    ) -> api.models.RobotTcp:
        """
        Ensure that a virtual TCP with the expected configuration exists on this motion group.
        If it doesn't exist, it will be created. If it exists but has different configuration,
        it will be updated by recreating it.

        Args:
            tcp (models.RobotTcp): The expected TCP configuration

        Returns:
            models.RobotTcp: The TCP configuration
        """
        existing_tcps = self._tcps_as_dict(await self.tcps())

        existing_tcp = existing_tcps.get(tcp.id)
        if (
            existing_tcp
            and existing_tcp.position == tcp.position
            and existing_tcp.orientation == tcp.orientation
            and existing_tcp.orientation_type == tcp.orientation_type
        ):
            # if existing_tcp and existing_tcp.pose == Pose(tcp.orientation, tcp.position):
            return existing_tcp

        await self._api_client.virtual_controller_api.add_virtual_controller_tcp(
            cell=self._cell,
            controller=self._controller_id,
            motion_group=self._motion_group_id,
            tcp=tcp.id,
            robot_tcp_data=api.models.RobotTcpData(
                name=tcp.name,
                position=tcp.position,
                orientation=tcp.orientation,
                orientation_type=tcp.orientation_type,
            ),
        )

        # TODO: this is a workaround to wait for the TCP to be created (restart of the virtual controller?)
        t = timeout
        while t > 0:
            try:
                tcps = self._tcps_as_dict(await self.tcps())
                return tcps[tcp.id]
            except KeyError:
                await asyncio.sleep(1)
                t -= 1

        raise TimeoutError(f"Failed to create TCP '{tcp.id}' within {timeout} seconds")

    async def _fetch_state(self) -> api.models.MotionGroupState:
        return await self._api_client.motion_group_api.get_current_motion_group_state(
            cell=self._cell, controller=self._controller_id, motion_group=self.id
        )

    async def _load_planned_motion(
        self, joint_trajectory: api.models.JointTrajectory, tcp: str
    ) -> str:
        load_plan_response = await self._api_client.trajectory_caching_api.add_trajectory(
            cell=self._cell,
            controller=self._controller_id,
            add_trajectory_request=api.models.AddTrajectoryRequest(
                motion_group=self.id, trajectory=joint_trajectory, tcp=tcp
            ),
        )

        if load_plan_response.error is not None:
            raise LoadPlanFailed(load_plan_response.error)

        if load_plan_response.trajectory is None:
            raise ValueError("Trajectory is None")

        return load_plan_response.trajectory

    async def _plan_with_collision_check(
        self,
        actions: list[Action],
        tcp: str,
        motion_group_setup: api.models.MotionGroupSetup,
        start_joint_position: tuple[float, ...],
    ) -> api.models.JointTrajectory:
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
            motion_group_setup: The motion group setup

        Returns: planned joint trajectory

        """
        # PREPARE THE REQUEST
        collision_setups = validate_collision_setups(actions)
        first_collision_setup = collision_setups[0] if len(collision_setups) > 0 else None

        # this is bad for memory because collision scenes can be very large
        # but we do it for now anyway because we don't want to create side effect on the provided motion group setup
        motion_group_setup = motion_group_setup.model_copy(deep=True)
        if motion_group_setup.collision_setups is None:
            motion_group_setup.collision_setups = api.models.CollisionSetups({})

        if first_collision_setup is not None:
            motion_group_setup.collision_setups.root["collision-check"] = first_collision_setup

        motion_commands = CombinedActions(items=tuple(actions)).to_motion_command()  # type: ignore

        # Plan the trajectory
        plan_trajectory_response = await self._api_client.trajectory_planning_api.plan_trajectory(
            cell=self._cell,
            plan_trajectory_request=api.models.PlanTrajectoryRequest(
                motion_group_setup=motion_group_setup,
                start_joint_position=api.models.DoubleArray(list(start_joint_position)),
                motion_commands=motion_commands,
            ),
        )

        # If the plan trajectory failed, raise an exception
        if isinstance(plan_trajectory_response.response, api.models.PlanTrajectoryFailedResponse):
            # TODO: handle partially executable path

            raise PlanTrajectoryFailed(
                error=plan_trajectory_response.response, motion_group_id=self.id
            )

        return plan_trajectory_response.response

    async def _plan_collision_free(
        self,
        action: CollisionFreeMotion,
        tcp: str,
        motion_group_setup: api.models.MotionGroupSetup,
        start_joint_position: tuple[float, ...] | None = None,
    ) -> api.models.JointTrajectory:
        """
        Plan a collision free trajectory to the target pose or joint positions.
        Collision setup can be provided in the action or in the motion group setup.


        Args:
            action: The target pose or joint positions to reach
            tcp:     The tool to use
            start_joint_position: The starting joint position
            motion_group_setup: The motion group setup

        Returns: planned joint trajectory


        """
        if start_joint_position is None:
            raise RuntimeError("start_joint_position must be provided for CollisionFreeMotion")

        # this is bad for memory because collision scenes can be very large
        # but we do it for now anyway because we don't want to create side effect on the provided motion group setup
        motion_group_setup = motion_group_setup.model_copy(deep=True)
        if motion_group_setup.collision_setups is None:
            motion_group_setup.collision_setups = api.models.CollisionSetups({})

        if action.collision_setup is not None:
            motion_group_setup.collision_setups.root["collision-free-motion"] = (
                action.collision_setup
            )

        if isinstance(action.target, Pose):
            solutions = await self._inverse_kinematics(
                poses=[action.target], tcp=tcp, motion_group_setup=motion_group_setup
            )
            if len(solutions) == 0 or len(solutions[0]) == 0:
                raise ValueError(
                    f"No inverse kinematics solution found for target pose {action.target}"
                )

            target_joint_positions = _find_shortest_distance(start_joint_position, solutions[0])
        elif isinstance(action.target, tuple):
            target_joint_positions = action.target
        else:
            raise ValueError("Invalid target type for CollisionFreeMotion")

        # Update the collision setup with user data
        if action.settings is not None:
            update_motion_group_setup_with_motion_settings(
                motion_group_setup=motion_group_setup, settings=action.settings
            )

        request: api.models.PlanCollisionFreeRequest = api.models.PlanCollisionFreeRequest(
            motion_group_setup=motion_group_setup,
            start_joint_position=api.models.DoubleArray(list(start_joint_position)),
            target=api.models.DoubleArray(list(target_joint_positions)),
            algorithm=action.algorithm,
        )

        response: api.models.PlanCollisionFreeResponse = (
            await self._api_client.trajectory_planning_api.plan_collision_free(
                cell=self._cell, plan_collision_free_request=request
            )
        )

        if isinstance(response.response, api.models.PlanCollisionFreeFailedResponse):
            raise PlanTrajectoryFailed(error=response.response, motion_group_id=self.id)

        if isinstance(response.response, api.models.JointTrajectory):
            return response.response

        raise ValueError("Unexpected response type from plan_collision_free API")

    async def _plan(
        self,
        actions: list[Action],
        tcp: str,
        start_joint_position: tuple[float, ...] | None = None,
        motion_group_setup: api.models.MotionGroupSetup | None = None,
    ) -> api.models.JointTrajectory:
        if not actions:
            raise ValueError("No actions provided")

        current_joints = start_joint_position or await self.joints()
        motion_group_setup = motion_group_setup or await self.get_setup(tcp)

        # TODO: can be done in parallel, would be a big performance boost
        all_trajectories = []
        for batch in split_actions_into_batches(actions):
            if len(batch) == 0:
                raise ValueError("Empty batch of actions")

            if isinstance(batch[0], CollisionFreeMotion):
                motion: CollisionFreeMotion = cast(CollisionFreeMotion, batch[0])
                trajectory = await self._plan_collision_free(
                    action=motion,
                    tcp=tcp,
                    start_joint_position=current_joints,
                    motion_group_setup=motion_group_setup,
                )
                all_trajectories.append(trajectory)
                # the last joint position of this trajectory is the starting point for the next one

                current_joints = tuple(trajectory.joint_positions[-1].root)
            elif isinstance(batch[0], WaitAction):
                # Waits generate a trajectory with the same joint position at each timestep
                # Use 50ms timesteps from 0 to wait_for_in_seconds
                wait_time = batch[0].wait_for_in_seconds
                timestep = 0.050  # 50ms timestep
                num_steps = max(2, int(wait_time / timestep) + 1)  # Ensure at least 2 points

                # Create equal-length arrays for positions, times, and locations
                joint_positions = [
                    api.models.Joints(list(current_joints)) for _ in range(num_steps)
                ]
                times = [i * timestep for i in range(num_steps)]
                # Ensure the last timestep is exactly the wait duration
                times[-1] = wait_time
                # Use the same location value for all points
                locations = [0] * num_steps

                trajectory = api.models.JointTrajectory(
                    joint_positions=joint_positions,
                    times=times,
                    locations=[api.models.Location(float(loc)) for loc in locations],
                )
                all_trajectories.append(trajectory)
                # the last joint position of this trajectory is the starting point for the next one
                current_joints = tuple(trajectory.joint_positions[-1])
            else:
                trajectory = await self._plan_with_collision_check(
                    actions=batch,
                    tcp=tcp,
                    start_joint_position=current_joints,
                    motion_group_setup=motion_group_setup,
                )
                all_trajectories.append(trajectory)
                # the last joint position of this trajectory is the starting point for the next one
                current_joints = tuple(trajectory.joint_positions[-1])

        return combine_trajectories(all_trajectories)

    # TODO: refactor and simplify code, tests are already there
    async def _execute(
        self,
        joint_trajectory: api.models.JointTrajectory,
        tcp: str,
        actions: list[Action],
        movement_controller: MovementController | None,
        start_on_io: api.models.StartOnIO | None = None,
    ) -> AsyncGenerator[MotionState, None]:
        if movement_controller is None:
            movement_controller = move_forward

        # Load planned trajectory
        trajectory_id = await self._load_planned_motion(joint_trajectory, tcp)

        controller = movement_controller(
            MovementControllerContext(
                combined_actions=CombinedActions(items=tuple(actions)),  # type: ignore
                motion_id=trajectory_id,
                start_on_io=start_on_io,
                motion_group_state_stream_gen=self.stream_state,
            )
        )

        class MotionGroupStateSentinel:
            pass

        states = asyncio.Queue[api.models.MotionGroupState | MotionGroupStateSentinel]()
        SENTINEL = MotionGroupStateSentinel()

        async def monitor_motion_group_state():
            async for motion_group_state in self.stream_state():
                if motion_group_state.execute:
                    states.put_nowait(motion_group_state)

        async def execution():
            try:
                await self._api_client.trajectory_execution_api.execute_trajectory(
                    cell=self._cell,
                    controller=self._controller_id,
                    client_request_generator=controller,
                )
            finally:
                states.put_nowait(SENTINEL)

        async with asyncio.TaskGroup() as tg:
            monitor_task = tg.create_task(monitor_motion_group_state())

            tg.create_task(execution(), name=f"execute_trajectory-{trajectory_id}-{self.id}")

            while (motion_group_state := await states.get()) is not SENTINEL:
                assert isinstance(motion_group_state, api.models.MotionGroupState)
                yield motion_group_state_to_motion_state(motion_group_state)

            # when the execution task finished
            # task group will still wait for the monitoring task
            # so we need to cancel it
            monitor_task.cancel()
