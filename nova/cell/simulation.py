import asyncio
import math
import time
from collections import defaultdict
from datetime import datetime
from typing import Any, AsyncIterable, Literal, SupportsIndex

import numpy as np
from scipy.spatial.transform import Rotation
from wandelbots_api_client import models

from nova import api
from nova.actions import Action, MovementController
from nova.actions.motions import CartesianPTP, Circular, JointPTP, Linear
from nova.cell.robot_cell import (
    AbstractController,
    AbstractRobot,
    AsyncCallableDevice,
    ConfigurablePeriphery,
    Device,
    IODevice,
    RobotCell,
    Timer,
)
from nova.core.io import ValueType
from nova.types import MotionState, MovementResponse, Pose, RobotState


def default_value():
    """
    NOTE:
        This method is needed because in order to create a Process with a function call the function should not contain
        any lambda functions
    """
    return "default_value"


class UnknownPose(ValueError):
    """A pose is requested from an SimulatedRobot without initial pose which have not moved so far"""


def naive_joints_to_pose(joints: tuple[float, ...]) -> Pose:
    """
    Very naive forward kinematics matching the naive IK in _plan:
      x = 1000 * j1
      y = 1000 * j2
      z = 1000 * j3
      rx = degrees(j4)
      ry = degrees(j5)
      rz = degrees(j6)

    Returns a Pose (position & orientation).
    """
    if len(joints) != 6:
        raise ValueError("This naive FK expects exactly 6 joints.")
    j1, j2, j3, j4, j5, j6 = joints

    # Position in mm if j1..j3 are in radians, but we just treat them as "some angle"
    x = 1000.0 * j1
    y = 1000.0 * j2
    z = 1000.0 * j3

    # Orientation in degrees
    rx = math.degrees(j4)
    ry = math.degrees(j5)
    rz = math.degrees(j6)

    return Pose((x, y, z, rx, ry, rz))


class SimulatedRobot(ConfigurablePeriphery, AbstractRobot):
    """A simulated robot cell without a camera"""

    class Configuration(ConfigurablePeriphery.Configuration):
        """The configuration of a simulated robot

        Args:
            initial_pose: The start pose of the robot, None means it is unknown
            step_size: the distance of the steps between MotionStates. The default value 0 means infinite steps (i.e.
                just start and end)
        """

        type: Literal["simulated_robot"] = "simulated_robot"
        id: str = "0@controller"
        initial_pose: Pose = Pose((0, 0, 0, 0, 0, 0))
        tools: dict[str, Pose] | None = None
        step_size: float = 0

    def __init__(self, configuration: Configuration = Configuration()):
        if not configuration.tools:
            configuration = configuration.model_copy(
                update={"tools": {"Flange": Pose((0, 0, 0, 0, 0, 0))}}
            )
        super().__init__(id=configuration.id, configuration=configuration)
        self._step_size = configuration.step_size if configuration.step_size else math.inf
        self._param = 1
        self._trajectory: list[MotionState] = (
            []
            if configuration.initial_pose is None
            else [
                MotionState(
                    motion_group_id=self.configuration.id,
                    path_parameter=0,
                    state=RobotState(pose=configuration.initial_pose, joints=None),
                )
            ]
        )
        # Added and used for tests of Wandelscript. In every planned_motion_iter() a motion trajectory is appended to
        # this list. Every motion trajectory corresponds to blocs of wandelscript code between sync commands.
        self.record_of_commands: list[list[Action]] = []

    async def get_optimizer_setup(self, tcp_name: str) -> api.models.OptimizerSetup:
        tcp_pos = api.models.Vector3d(x=0, y=0, z=0)
        tcp_pos.x, tcp_pos.y, tcp_pos.z = self.configuration.tools[tcp_name].position
        tcp_ori = api.models.Quaternion(w=1, x=0, y=0, z=0)
        tcp_ori.x, tcp_ori.y, tcp_ori.z, tcp_ori.w = Rotation.from_rotvec(
            self.configuration.tools[tcp_name].orientation
        ).as_quat()
        joint_position_limits = [
            api.models.PlanningLimitsLimitRange(lower_limit=-np.pi, upper_limit=np.pi)
        ] * 6
        joint_velocity_limits = [1.0] * 6
        joint_acceleration_limits = [1e8] * 6
        joint_torque_limits = [200.0] * 6
        limits = api.models.PlanningLimits(
            joint_position_limits=joint_position_limits,
            joint_velocity_limits=joint_velocity_limits,
            joint_acceleration_limits=joint_acceleration_limits,
            joint_torque_limits=joint_torque_limits,
            tcp_velocity_limit=500,
            tcp_acceleration_limit=1e8,
            tcp_orientation_velocity_limit=1,
            tcp_orientation_acceleration_limit=1e8,
            elbow_velocity_limit=1e8,
            elbow_acceleration_limit=1e8,
            elbow_force_limit=1e8,
        )
        setup = api.models.SafetyConfiguration(global_limits=limits)
        tcp = api.models.PlannerPose(position=tcp_pos, orientation=tcp_ori)
        mounting = api.models.PlannerPose(
            position=api.models.Vector3d(x=0, y=0, z=0),
            orientation=api.models.Quaternion(x=0, y=0, z=0, w=1),
        )
        motion_group_type = "FANUC_CRX25iA"
        payload = api.models.Payload(name="example", payload=0.0)
        return api.models.OptimizerSetup(
            motion_group_type=motion_group_type,
            mounting=mounting,
            tcp=tcp,
            safety_setup=setup,
            cycle_time=8,
            payload=payload,
        )

    async def get_mounting(self) -> Pose:
        mounting = (await self.get_optimizer_setup((await self.tcp_names())[0])).mounting
        if mounting is None or mounting.position is None or mounting.orientation is None:
            raise ValueError("Mounting is None")

        return Pose.from_position_and_quaternion(  # type: ignore
            [mounting.position.x, mounting.position.y, mounting.position.z],
            [
                mounting.orientation.w,
                mounting.orientation.x,
                mounting.orientation.y,
                mounting.orientation.z,
            ],
        )

    async def _plan(
        self,
        actions: list[Action] | Action,
        tcp: str,
        start_joint_position: tuple[float, ...] | None = None,
        optimizer_setup: api.models.OptimizerSetup | None = None,
    ) -> api.models.JointTrajectory:
        """
        A simple example planner that:
          1. Starts from [0, 0, 0, 0, 0, 0].
          2. For each action, determines the final joint configuration (very naive).
          3. Interpolates in joint space from the current to the final configuration.
          4. Accumulates the samples in JointTrajectory.joint_positions, times, locations.
        """

        # We assume 6-DOF for this example
        current_joints = (
            np.zeros(6, dtype=float)
            if start_joint_position is None
            else np.array(start_joint_position, dtype=float)
        )

        joint_positions: list[tuple[float, ...]] = []
        times = []
        locations = []

        # For demonstration, each motion block will have this many interpolation steps
        steps_per_action = 10

        # Keep track of the 'time' as we build the trajectory
        current_time = 0.0
        dt = 0.1  # step of 0.1s between samples

        # ---------------------------------------------------------------------
        # Helper function: Very naive approach to get final joint configuration
        # from a cartesian target. This is *not* a real IK solver!
        # ---------------------------------------------------------------------
        def naive_cartesian_to_joints(pose: Pose) -> np.ndarray:
            """
            Convert a pose (x, y, z, rx, ry, rz) to a 6-joint configuration, very naively.
            This is purely for demonstration. Replace with real IK if you have one.
            """
            # We'll just clamp each coordinate or orientation to a "reasonable" range
            # and pretend that's your joint angle in radians. This is purely illustrative!
            px, py, pz = pose.position.x, pose.position.y, pose.position.z
            ox, oy, oz = pose.orientation.x, pose.orientation.y, pose.orientation.z

            # Some made-up scaling to turn cartesian coords into "joint angles"
            # You could do anything that produces a 6-length array of angles
            j1 = np.clip(px / 1000.0, -2.0, 2.0)
            j2 = np.clip(py / 1000.0, -2.0, 2.0)
            j3 = np.clip(pz / 1000.0, -2.0, 2.0)
            j4 = np.radians(ox) % (2 * np.pi)
            j5 = np.radians(oy) % (2 * np.pi)
            j6 = np.radians(oz) % (2 * np.pi)

            return np.array([j1, j2, j3, j4, j5, j6], dtype=float)

        # ---------------------------------------------------------------------
        # Loop through actions and build the trajectory
        # ---------------------------------------------------------------------
        for i, action in enumerate(actions):
            if isinstance(action, JointPTP):
                # Directly use the target as the final joints
                final_joints = np.array(action.target, dtype=float)

            elif isinstance(action, (Linear, CartesianPTP, Circular)):
                # Very naive approach: do a "fake IK" from the pose
                if not isinstance(action.target, Pose):
                    # If user gave a vector or something else, you'd need to convert
                    # but from your code, CartesianPTP/Linear typically store Pose internally
                    raise ValueError(f"Expected Pose as target, got {type(action.target)}")

                final_joints = naive_cartesian_to_joints(action.target)

            else:
                # If there's any other custom action type, handle it here
                raise ValueError(f"Unsupported action type {type(action)}")

            # Interpolate from current_joints -> final_joints in N steps
            for step in range(steps_per_action):
                alpha = float(step) / (steps_per_action - 1)  # from 0 to 1
                # Linear interpolation in joint space
                interp_joints = (1 - alpha) * current_joints + alpha * final_joints

                joint_positions.append(tuple(interp_joints.tolist()))
                times.append(current_time)
                # "location" can be a float that indicates fraction of "action i"
                # E.g. i + alpha
                locations.append(i + alpha)

                # Move time forward
                current_time += dt

            # The final position of this action is the start of the next
            current_joints = final_joints

        return api.models.JointTrajectory(
            joint_positions=[api.models.Joints(joints=list(j)) for j in joint_positions],
            times=times,
            locations=locations,
        )

    async def _execute(
        self,
        joint_trajectory: models.JointTrajectory,
        tcp: str,
        actions: list[Action],
        movement_controller: MovementController | None,
    ) -> AsyncIterable[MovementResponse]:
        """
        Executes the given joint_trajectory by simulating the robot's motion.

        * self._trajectory must be a list[Pose].
        * For each step in the trajectory:
            - Optionally wait to match the 'times' array.
            - Compute the current Pose from the joint angles (fake forward kinematics).
            - Append that Pose to self._trajectory (while the robot is moving).
            - Create a RobotState(pose=..., joints=...).
            - Create a MotionState(path_parameter=..., state=...).
            - Call on_movement(motion_state).

        'path_parameter' is the 'location' in the trajectory, e.g.:
          - 0→1 for Action 1
          - 1→2 for Action 2
          - 2→3 for Action 3
        """

        self._trajectory = []
        self.record_of_commands.append(actions)

        # Start time for optional synchronization
        start_time = time.time()

        # Iterate over each interpolation step in the planned trajectory
        for joints, planned_time, location in zip(
            joint_trajectory.joint_positions, joint_trajectory.times, joint_trajectory.locations
        ):
            # Wait until the correct planned_time from the start (if needed)
            now = time.time()
            wait_secs = (start_time + float(planned_time)) - now
            if wait_secs > 0:
                await asyncio.sleep(wait_secs)

            # Send joint command to hardware or simulator (if a controller is provided)
            if movement_controller is not None:
                # e.g. movement_controller.send_joint_command(joints)
                pass

            # Compute the current Pose from these joint values
            current_pose = naive_joints_to_pose(tuple(joints.joints))
            motion_state = MotionState(
                motion_group_id=self.id,
                path_parameter=float(location),
                state=RobotState(pose=current_pose, joints=tuple(joints.joints)),
            )

            # Append this Pose to self._trajectory while moving
            self._trajectory.append(motion_state)

            yield api.models.ExecuteTrajectoryResponse(
                api.models.Movement(
                    movement=api.models.MovementMovement(
                        time_to_end=0,
                        current_location=0,
                        state=api.models.RobotControllerState(
                            controller="Simulated",
                            operation_mode="OPERATION_MODE_AUTO",
                            safety_state="SAFETY_STATE_NORMAL",
                            timestamp=datetime.now(),
                            sequence_number="0",
                            motion_groups=[
                                api.models.MotionGroupState(
                                    motion_group="0",
                                    controller="Simulated",
                                    tcp_pose=api.models.TcpPose(
                                        tcp="Flange",
                                        position=motion_state.state.pose.position.to_api_vector3d(),
                                        orientation=motion_state.state.pose.orientation.to_api_vector3d(),
                                    ),
                                    joint_velocity=api.models.Joints(joints=[0.0] * 6),
                                    velocity=api.models.MotionVector(
                                        linear=api.models.Vector3d(x=0, y=0, z=0)
                                    ),
                                    joint_limit_reached=api.models.MotionGroupStateJointLimitReached(
                                        limit_reached=[False]
                                    ),
                                    joint_position=api.models.Joints(
                                        joints=list(motion_state.state.joints)  # type: ignore
                                    ),
                                    sequence_number="0",
                                )
                            ],
                        ),
                    )
                )
            )

    async def tcps(self) -> list[api.models.RobotTcp]:
        return [
            api.models.RobotTcp(
                id=name,
                readable_name=name,
                position=tool_pose.position.model_dump(),
                rotation=tool_pose.orientation.model_dump(),
            )
            for name, tool_pose in self.configuration.tools.items()
        ]

    async def tcp_names(self) -> list[str]:
        return list(self.configuration.tools.keys())

    async def active_tcp(self) -> api.models.RobotTcp:
        tcps = await self.tcps()
        return next(iter(tcps))

    async def active_tcp_name(self) -> str:
        return next(iter(self.configuration.tools))

    async def get_state(self, tcp: str | None = None) -> RobotState:
        if not self._trajectory:
            raise UnknownPose
        flange2robot = self._trajectory[-1]
        # TODO: calculate tool offset
        # if tcp:
        #    tcp2flange = pose_to_versor((await self.get_tcps())[tcp])
        #    tcp2robot = flange2robot & tcp2flange
        # else:
        #    tcp2robot = flange2robot

        # tcp_pose = Pose.from_position_and_quaternion(*tcp2robot.to_pose())
        return flange2robot.state

    async def joints(self) -> tuple:
        if not self._trajectory:
            raise UnknownPose
        return self._trajectory[-1].state.joints  # type: ignore

    async def tcp_pose(self, tcp: str | None = None) -> Pose:
        if tcp is None:
            tcp = "Flange"
        state = await self.get_state(tcp)
        return state.pose

    async def stop(self):
        pass

    def release(self):
        pass

    def set_status(self, active: bool) -> None:
        print(f"set status: {active}")


class SimulatedIO(ConfigurablePeriphery, Device, IODevice):
    """A simulated IO by a default dict

    Example:
    >>> import asyncio
    >>> async def example():
    ...     device = SimulatedIO(silent=True)
    ...     await device.write("a", 2)
    ...     return await device.read("a")
    >>> asyncio.run(example())
    2
    """

    class Configuration(ConfigurablePeriphery.Configuration):
        type: Literal["simulated_io"] = "simulated_io"
        id: str = "io"

    def __init__(self, configuration: Configuration = Configuration(id="io"), silent=False):
        super().__init__(configuration=configuration)
        self._silent = silent
        self._io: dict[str, Any] = defaultdict(default_value)

    async def read(self, key: str) -> ValueType:
        if not self._silent:
            print(f"Get {self._io[key]} from '{key}'")
        return self._io[key]

    async def write(self, key: str, value: ValueType) -> None:
        if not self._silent:
            print(f"Set '{key}' to {value}")
        self._io[key] = value

    async def wait_for_bool_io(self, key: str, value: bool) -> None:
        while await self.read(key) != value:
            await asyncio.sleep(0.1)


class SimulatedController(ConfigurablePeriphery, AbstractController):
    """A simulated controller"""

    class Configuration(ConfigurablePeriphery.Configuration):
        type: Literal["simulated_controller"] = "simulated_controller"
        id: str = "controller"
        robots: list[SimulatedRobot.Configuration] | None = None
        raises_on_open: bool = False

    def __init__(self, configuration: Configuration = Configuration(id="controller")):
        super().__init__(configuration=configuration)
        if configuration.robots is None:
            robot_configurations = get_simulated_robot_configs(configuration.id, 2)
        else:
            robot_configurations = configuration.robots
        self._robots: dict[str, AbstractRobot] = {
            robot.id: robot for robot in map(SimulatedRobot, robot_configurations)
        }
        self._simulated_io = SimulatedIO()

    def get_motion_groups(self) -> dict[str, AbstractRobot]:
        return self._robots

    def __getitem__(self, item):
        return self._robots[f"{item}@{self.id}"]

    async def read(self, key: str) -> ValueType:
        return await self._simulated_io.read(key)

    async def write(self, key: str, value: ValueType) -> None:
        await self._simulated_io.write(key, value)

    async def wait_for_bool_io(self, key: str, value: bool) -> None:
        await self._simulated_io.wait_for_bool_io(key, value)

    async def open(self):
        if self.configuration.raises_on_open:
            raise RuntimeError("RaisingRobotCell")


class SimulatedTimer(Timer):
    """A simulated timer (doing logging only)"""

    class Configuration(Timer.Configuration):
        type: Literal["simulated_timer"] = "simulated_timer"
        id: str = "timer"

    def __init__(self, configuration: Configuration = Configuration()):
        super().__init__(configuration=configuration)

    async def __call__(self, duration: float):
        print(f"Wait for {duration} ms")


class SimulatedAsyncCallable(ConfigurablePeriphery, AsyncCallableDevice):
    """A simulated callable of an external function or service

    Example:
    >>> import asyncio
    >>> async def example():
    ...     device = SimulatedAsyncCallable()
    ...     async with device:
    ...         return await device("method_name", 1, 2)
    >>> asyncio.run(example())
    ('method_name', (1, 2))
    """

    class Configuration(ConfigurablePeriphery.Configuration):
        type: Literal["simulated_callable"] = "simulated_callable"
        id: str = "callable"

    # TODO the id is only "sensor" to make it work for the current RobotCell
    def __init__(self, configuration: Configuration = Configuration(id="sensor")):
        super().__init__(configuration=configuration)

    async def _call(self, key, *args):
        return key, args


class SimulatedRobotCell(RobotCell):
    """A robot cell fully simulated (on default)"""

    def __init__(self, **kwargs):
        defaults = {
            "timer": SimulatedTimer(),
            "controller": SimulatedController(),
            "sensor": SimulatedAsyncCallable(),
        }
        for key, value in defaults.items():
            if key not in kwargs:
                kwargs[key] = value
        super().__init__(**kwargs)


def get_simulated_robot_configs(
    controller_id: str = "controller", num_robots: SupportsIndex = 2
) -> list[SimulatedRobot.Configuration]:
    return [
        SimulatedRobot.Configuration(
            id=f"{i}@{controller_id}", tools={"Flange": Pose((0, 0, 0, 0, 0, 0))}
        )
        for i in range(num_robots)
    ]


def get_robot_controller(
    controller_id="controller", num_robots=1, raises_on_open=False
) -> SimulatedController:
    """Get a simulated controller"""
    return SimulatedController(
        SimulatedController.Configuration(
            id=controller_id,
            robots=get_simulated_robot_configs(controller_id=controller_id, num_robots=num_robots),
            raises_on_open=raises_on_open,
        )
    )


def get_robot_cell() -> SimulatedRobotCell:
    """Get a simulated robot cell"""
    return SimulatedRobotCell(controller=get_robot_controller())
