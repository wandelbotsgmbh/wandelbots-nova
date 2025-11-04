import asyncio
from functools import partial
from typing import AsyncIterable

from icecream import ic

from nova import api
from nova.actions import Action, CombinedActions, MovementController, MovementControllerContext
from nova.actions.mock import WaitAction
from nova.actions.motions import Motion
from nova.cell.robot_cell import AbstractRobot
from nova.config import ENABLE_TRAJECTORY_TUNING
from nova.core import logger
from nova.core.exceptions import InconsistentCollisionScenes
from nova.core.gateway import ApiGateway
from nova.core.movement_controller import move_forward
from nova.core.tuner import TrajectoryTuner
from nova.types import MovementResponse, Pose, RobotState, RobotTcp
from nova.types.state import MotionState, motion_group_state_to_motion_state
from nova.utils import StreamExtractor

MAX_JOINT_VELOCITY_PREPARE_MOVE = 0.2
START_LOCATION_OF_MOTION = 0.0


def motion_group_setup_from_motion_group_description(
    motion_group_description: api.models.MotionGroupDescription,
    tcp_name: str,
    payload: api.models.Payload | None = None,
) -> api.models.MotionGroupSetup:
    # TODO the function does multiple things not separated very well
    collision_scene = api.models.CollisionSetup(
        colliders=motion_group_description.safety_zones,
        link_chain=motion_group_description.safety_link_colliders,
        tool=motion_group_description.safety_tool_colliders,
        self_collision_detection=True,  # explicitly set here until we have a better understanding
    )
    # For the time being it is assumed that the auto limits are always present
    # We also assume that the motion player in RAE will scale corretly if the
    # planned trajectory is played back with different limits (due to a different robot mode)
    # than the one used for planning
    assert motion_group_description.operation_limits.auto_limits is not None
    limits = motion_group_description.operation_limits.auto_limits
    # TODO maybe we also want to give the user more control over the collision scene
    return api.models.MotionGroupSetup(
        motion_group_model=motion_group_description.motion_group_model,
        cycle_time=motion_group_description.cycle_time,
        mounting=motion_group_description.mounting,
        global_limits=limits,
        tcp_offset=motion_group_description.tcps[tcp_name].pose,
        payload=payload,
        collision_scene=collision_scene,
    )


def compare_collision_scenes(scene1: api.models.CollisionSetup, scene2: api.models.CollisionSetup):
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
    Splits the list of actions into batches of actions, collision free motions and waits.
    Actions are sent to plan_trajectory API and collision free motions are sent to plan_collision_free_ptp API.
    Waits generate a trajectory with the same start and end position.
    """
    batches: list[list[Action]] = []
    for action in actions:
        if (
            # Start a new batch if:
            not batches  # first action no batches yet
            or isinstance(action, WaitAction)
            or isinstance(batches[-1][-1], WaitAction)
        ):
            batches.append([action])
        else:
            batches[-1].append(action)
    return batches


def combine_trajectories(
    trajectories: list[api.models.JointTrajectory],
) -> api.models.JointTrajectory:
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


def validate_collision_scenes(actions: list[Action]) -> list[api.models.CollisionSetup]:
    """
    RAE V1 APIs provide two ways of planning actions.
    Collition free planning and collision checked planning.
    As the names suggest, collision free planning produces a joint trajectory with which the collision is avoided.
    But collision check planning checks for collision and if there is one, it will return an error.


    The action list python sdk takes from the user has a variety of types.
    There can be a variety of actions in the list.
    1. Collision free motions
    2. Normal motions
    3. Waits
    4. Write actions -> this is a special write on the path supported by the API


    This function checks that a given set of actions contains valid collision scenes data.
    It is assumed that the action list provided here is a sub-batch generated by the split_actions_into_batches function.
    """
    motion_count = len([action for action in actions if isinstance(action, Motion)])
    collision_scenes = [
        action.collision_scene
        for action in actions
        if isinstance(action, Motion) and action.collision_scene is not None
    ]

    if len(collision_scenes) != 0 and len(collision_scenes) != motion_count:
        raise InconsistentCollisionScenes(
            "Only some of the actions have collision scene. Either specify it for all or none."
        )

    # If a collision scene is provided, the same should be provided for all the collision scene
    if len(collision_scenes) > 1:
        first_scene = collision_scenes[0]
        if not all(compare_collision_scenes(first_scene, scene) for scene in collision_scenes[1:]):
            raise InconsistentCollisionScenes(
                "All actions must use the same collision scene but some are different"
            )

    return collision_scenes


class MotionGroup(AbstractRobot):
    """Manages motion planning and execution within a specified motion group."""

    def __init__(
        self, api_gateway: ApiGateway, cell: str, controller_id: str, motion_group_id: str
    ):
        """
        Initializes a new MotionGroup instance.

        Args:
            api_gateway (ApiGateway): The API gateway through which motion commands are sent.
            cell (str): The name or identifier of the robotic cell.
            motion_group_id (str): The identifier of the motion group.
        """
        self._api_gateway = api_gateway
        self._cell = cell
        self._controller_id = controller_id
        self._motion_group_id = motion_group_id
        self._current_motion: str | None = None
        super().__init__(id=motion_group_id)

    @property
    def motion_group_id(self) -> str:
        """
        Returns:
            str: The unique identifier for this motion group.
        """
        return self._motion_group_id

    @property
    def current_motion(self) -> str | None:
        # if not self._current_motion:
        #    raise ValueError("No MotionId attached. There is no planned motion available.")
        return self._current_motion

    async def open(self):
        # TODO if there is no explicit motion group activation, what should we do here?
        # maybe we set the mode to control mode? But this is not needed (implicitly done by the trajectory execution)
        return self

    async def close(self):
        # RPS-1174: when a motion group is deactivated, RAE closes all open connections
        #           this behaviour is not desired in some cases,
        #           so for now we will not deactivate for the user
        pass

    async def stop(self):
        logger.debug(f"Stopping motion of {self}...")
        try:
            if self._current_motion is None:
                raise ValueError("No motion to stop")
            await self._api_gateway.stop_motion(cell=self._cell, motion_id=self._current_motion)
            logger.debug(f"Motion {self.current_motion} stopped.")
        except ValueError as e:
            logger.debug(f"No motion to stop for {self}: {e}")

    async def get_description(self) -> api.models.MotionGroupDescription:
        return await self._fetch_description()

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
            tcp_offset = (await self.tcps())[tcp].pose
            pose = Pose(motion_group_state.flange_pose) @ tcp_offset
        return RobotState(pose=pose, tcp=tcp, joints=tuple(motion_group_state.joint_position))

    async def stream_state(
        self, response_rate_msecs: int | None = None
    ) -> AsyncIterable[api.models.MotionGroupState]:
        """
        Streams the motion group state continuously.

        This method provides a real-time stream of robot state information including
        joint positions and TCP pose data for the motion group.
        Args:
            response_rate_msecs (int | None): The rate at which state updates are streamed
                                             in milliseconds. Defaults to None for maximum rate.
        """
        response_stream = self._api_gateway.motion_group_api.stream_motion_group_state(
            cell=self._cell,
            controller=self._controller_id,
            motion_group=self.motion_group_id,
            response_rate=response_rate_msecs,
        )
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

    async def tcps(self) -> dict[str, RobotTcp]:
        return {
            tcp: RobotTcp(id=tcp, name=tcp_offset.name, pose=Pose(tcp_offset.pose))
            for tcp, tcp_offset in (await self._fetch_description()).tcps.items()
        }

    # TODO names?, ids?, both?, whatever? (probably ids atm)
    async def tcp_names(self) -> list[str]:
        return list((await self.tcps()).keys())

    async def active_tcp(self) -> RobotTcp:
        return (await self._fetch_state()).tcp

    async def active_tcp_name(self) -> str:
        return (await self.active_tcp()).name

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
        existing_tcps = await self.tcps()

        from icecream import ic

        existing_tcp = existing_tcps.get(tcp.id)
        ic(existing_tcp, tcp)
        if (
            existing_tcp
            and api.models.RobotTcp(
                id=existing_tcp.id,
                name=existing_tcp.name,
                position=existing_tcp.pose.position,
                orientation=existing_tcp.pose.orientation,
            )
            == tcp
        ):
            # if existing_tcp and existing_tcp.pose == Pose(tcp.orientation, tcp.position):
            return existing_tcp

        await self._api_gateway.virtual_controller_api.add_virtual_controller_tcp(
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
                return (await self.tcps())[tcp.id]
            except KeyError:
                await asyncio.sleep(1)
                t -= 1

        raise TimeoutError(f"Failed to create TCP '{tcp.id}' within {timeout} seconds")

    async def _get_setup(self, tcp: str) -> api.models.MotionGroupSetup:
        # TODO allow to specify payload
        motion_group_description = await self._fetch_description()
        return motion_group_setup_from_motion_group_description(
            motion_group_description=motion_group_description, tcp_name=tcp
        )

    async def _fetch_description(self) -> api.models.MotionGroupDescription:
        return await self._api_gateway.motion_group_api.get_motion_group_description(
            cell=self._cell, controller=self._controller_id, motion_group=self.motion_group_id
        )

    async def _fetch_state(self) -> api.models.MotionGroupState:
        return await self._api_gateway.motion_group_api.get_current_motion_group_state(
            cell=self._cell, controller=self._controller_id, motion_group=self.motion_group_id
        )

    async def _load_planned_motion(
        self, joint_trajectory: api.models.JointTrajectory, tcp: str
    ) -> str:
        return await self._api_gateway.load_planned_motion(
            cell=self._cell,
            controller_id=self._controller_id,
            motion_group_id=self.motion_group_id,
            joint_trajectory=joint_trajectory,
            tcp=tcp,
        )

    async def _plan_with_collision_check(
        self,
        actions: list[Action],
        tcp: str,
        start_joint_position: tuple[float, ...] | None = None,
        robot_setup: api.models.MotionGroupSetup | None = None,
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
            robot_setup: The robot setup

        Returns: planned joint trajectory

        """
        # PREPARE THE REQUEST
        collision_scenes = validate_collision_scenes(actions)
        start_joint_position = start_joint_position or await self.joints()
        robot_setup = robot_setup or await self._get_setup(tcp=tcp)

        motion_commands = CombinedActions(items=tuple(actions)).to_motion_command()  # type: ignore

        static_colliders = None
        collision_motion_group = None
        if collision_scenes and len(collision_scenes) > 0:
            static_colliders = collision_scenes[0].colliders

            motion_group_type = robot_setup.motion_group_model
            if (
                collision_scenes[0].motion_groups
                and motion_group_type in collision_scenes[0].motion_groups
            ):
                collision_motion_group = collision_scenes[0].motion_groups[motion_group_type]

        request = api.models.PlanTrajectoryRequest(
            motion_group_setup=robot_setup,
            start_joint_position=list(start_joint_position),
            motion_commands=motion_commands,
        )

        return await self._api_gateway.plan_trajectory(
            cell=self._cell, motion_group_id=self.motion_group_id, request=request
        )

    async def _plan(
        self,
        actions: list[Action],
        tcp: str,
        start_joint_position: tuple[float, ...] | None = None,
        robot_setup: api.models.MotionGroupSetup | None = None,
    ) -> api.models.JointTrajectory:
        if not actions:
            raise ValueError("No actions provided")

        current_joints = start_joint_position or await self.joints()
        robot_setup = robot_setup or await self._get_setup(tcp=tcp)

        all_trajectories = []
        for batch in split_actions_into_batches(actions):
            if len(batch) == 0:
                raise ValueError("Empty batch of actions")

            elif isinstance(batch[0], WaitAction):
                # Waits generate a trajectory with the same joint position at each timestep
                # Use 50ms timesteps from 0 to wait_for_in_seconds
                wait_time = batch[0].wait_for_in_seconds
                timestep = 0.050  # 50ms timestep
                num_steps = max(2, int(wait_time / timestep) + 1)  # Ensure at least 2 points

                # Create equal-length arrays for positions, times, and locations
                joint_positions = [
                    api.models.Joints(joints=list(current_joints)) for _ in range(num_steps)
                ]
                times = [i * timestep for i in range(num_steps)]
                # Ensure the last timestep is exactly the wait duration
                times[-1] = wait_time
                # Use the same location value for all points
                locations = [0] * num_steps

                trajectory = api.models.JointTrajectory(
                    joint_positions=joint_positions,
                    times=times,
                    locations=[float(loc) for loc in locations],
                )
                all_trajectories.append(trajectory)
                # the last joint position of this trajectory is the starting point for the next one
                current_joints = tuple(trajectory.joint_positions[-1].joints)
            else:
                trajectory = await self._plan_with_collision_check(
                    actions=batch,
                    tcp=tcp,
                    start_joint_position=current_joints,
                    robot_setup=robot_setup,
                )
                all_trajectories.append(trajectory)
                # the last joint position of this trajectory is the starting point for the next one
                current_joints = tuple(trajectory.joint_positions[-1])

        return combine_trajectories(all_trajectories)

    # TODO: refactor and simplify code, tests are already there
    # TODO: split into batches when the collision scene changes in a batch of collision free motions

    async def _execute(
        self,
        joint_trajectory: api.models.JointTrajectory,
        tcp: str,
        actions: list[Action],
        movement_controller: MovementController | None,
        start_on_io: api.models.StartOnIO | None = None,
    ) -> AsyncIterable[MotionState]:
        # This is the entrypoint for the trajectory tuning mode
        if ENABLE_TRAJECTORY_TUNING:
            logger.info("Entering trajectory tuning mode...")
            async for execute_response in self._tune_trajectory(joint_trajectory, tcp, actions):
                yield execute_response
            return

        if movement_controller is None:
            movement_controller = move_forward

        # Load planned trajectory
        trajectory_id = await self._load_planned_motion(joint_trajectory, tcp)
        ic(trajectory_id)

        controller = movement_controller(
            MovementControllerContext(
                combined_actions=CombinedActions(items=tuple(actions)),  # type: ignore
                motion_id=trajectory_id,
                start_on_io=start_on_io,
                motion_group_state_stream_gen=self.stream_state,
            )
        )
        states = asyncio.Queue[api.models.MotionGroupState | None]()
        SENTINEL = None

        async def monitor_motion_group_state():
            async for motion_group_state in self.stream_state():
                ic()
                # ic(motion_group_state)
                if motion_group_state.execute:
                    states.put_nowait(motion_group_state)

        async def execution():
            await self._api_gateway.trajectory_execution_api.execute_trajectory(
                cell=self._cell, controller=self._controller_id, client_request_generator=controller
            )
            states.put_nowait(SENTINEL)

        async with asyncio.TaskGroup() as tg:
            monitor_task = tg.create_task(monitor_motion_group_state())
            execution_task = tg.create_task(
                execution(), name=f"execute_trajectory-{trajectory_id}-{self.motion_group_id}"
            )

            while (motion_group_state := await states.get()) is not SENTINEL:
                # ic()
                # ic(motion_group_state)
                yield motion_group_state_to_motion_state(motion_group_state)
            ic()
            monitor_task.cancel()
            # async for motion_group_state in self.stream_state():
            #     if motion_group_state.execute:
            #         yield motion_group_state_to_motion_state(motion_group_state)

            # try:
            #     await execution_task
            #     await monitor_task
            # except asyncio.CancelledError:
            #     ic()

    async def _stream_jogging(self, tcp, movement_controller):
        controller = movement_controller(
            MovementControllerContext(
                combined_actions=CombinedActions(),  # type: ignore
                motion_id="DUMMY_MOTION_ID",  # TODO This is a dummy ID, not used in jogging
            )
        )

        def stop_condition(_response):
            return True

        execute_response_streaming_controller = StreamExtractor(controller, stop_condition)
        execution_task = asyncio.create_task(
            self._api_gateway.motion_group_jogging_api.execute_jogging(
                cell=self._cell, controller=self._controller_id, client_request_generator=controller
            )
        )
        # BEGIN TODO
        # This has just been copied during the last main merge and needs to be checked
        # MOTION_STATE_STREAM_RATE_MS = 100
        # motion_state_stream = self._api_gateway.motion_group_api.stream_motion_group_state(
        #    cell=self._cell,
        #    controller=self._controller_id,
        #    motion_group=self.motion_group_id,
        #    response_rate=MOTION_STATE_STREAM_RATE_MS,
        # )
        # execute_response_stream = stream.merge(
        #    execute_response_streaming_controller, motion_state_stream
        # )
        # async for execute_response in execute_response_stream.stream():
        #    # async for execute_response in execute_response_streaming_controller:
        #    yield execute_response
        # await execution_task
        # END TODO

        async for execute_response in execute_response_streaming_controller:
            yield execute_response
        await execution_task

    async def _tune_trajectory(
        self, joint_trajectory: api.models.JointTrajectory, tcp: str, actions: list[Action]
    ) -> AsyncIterable[MovementResponse]:
        start_joints = await self.joints()

        async def plan_fn(actions: list[Action]) -> tuple[str, api.models.JointTrajectory]:
            # we fix the start joints here because the tuner might call plan multiple times whilst tuning
            # and the start joints would change to the respective joint positions at the time of planning
            # which is not what we want
            joint_trajectory = await self._plan(actions, tcp, start_joints)
            load_planned_motion_response = await self._load_planned_motion(joint_trajectory, tcp)
            return load_planned_motion_response.motion, joint_trajectory

        execute_fn = partial(self._api_gateway.motion_api.execute_trajectory, cell=self._cell)
        tuner = TrajectoryTuner(actions, plan_fn, execute_fn)
        async for response in tuner.tune():
            yield response
