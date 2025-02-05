from typing import final, Union, Protocol, runtime_checkable, AsyncIterable, Callable
from abc import ABC, abstractmethod
import asyncio
from contextlib import AsyncExitStack
from loguru import logger
import aiostream
from collections import defaultdict
import asyncstdlib
from dataclasses import dataclass

from typing import Any, ClassVar, Literal, get_origin, get_type_hints
from nova.types import Pose, MotionState
from nova.actions import Action, MovementController
import pydantic
import anyio
from functools import reduce
from nova.api import models


class ConfigurablePeriphery:
    """A device which is configurable"""

    all_classes: ClassVar[dict] = {}

    def __init_subclass__(cls, is_abstract=False):
        super().__init_subclass__()
        if not is_abstract:
            assert (
                hasattr(cls, "Configuration")
                and get_origin(get_type_hints(cls.Configuration)["type"]) is Literal
            ), f"{cls.__name__} has no type literal"
            assert ConfigurablePeriphery.Configuration is not cls.Configuration
            cls.all_classes[cls.Configuration] = cls

    class Configuration(pydantic.BaseModel):
        """Minimum configuration of a configurable periphery

        Args:
            identifier: A unique identifier to reference the periphery
        """

        model_config = pydantic.ConfigDict(frozen=True)

        type: str
        identifier: str

    _configuration: Configuration

    def __init__(self, configuration: Configuration, **kwargs):
        super().__init__(**kwargs)
        self._configuration = configuration

    @property
    def configuration(self):
        return self._configuration

    @property
    def identifier(self):
        return self.configuration.identifier

    @classmethod
    def from_dict(cls, data):
        """Constructs a new configurable periphery from a dict

        Returns:
            cls: the newly created ConfigurablePeriphery object

        """
        return cls(cls.Configuration(**data))

    def to_dict(self) -> dict[str, Any]:
        """Creates a json dict from the configurable periphery parameters which can be transformed to a json string

        Returns:
            Dict[str, Any]: a json string
        """
        return self._configuration.model_dump()


class Device(ABC):
    """A device that takes care of lifetime management"""

    def __init__(self, **kwargs):
        super().__init__(**kwargs)
        self._is_active = False

    async def open(self) -> None:
        """Allocates the external hardware resource (i.e. establish a connection)"""
        self._is_active = True

    async def close(self):
        """Release the external hardware (i.e. close connection or set mode of external hardware back)"""
        self._is_active = False

    async def restart(self):
        if self._is_active:
            await self.close()
        await self.open()

    @final
    async def __aenter__(self):
        await self.open()
        return self

    @final
    async def __aexit__(self, exc_type, exc_val, exc_tb):
        await self.close()


ValueType = Union[int, str, bool, float, Pose]


@runtime_checkable
class InputDevice(Protocol):
    """A device which supports reading from"""

    async def read(self, key: str) -> ValueType:
        """Read a value given its key"""


@runtime_checkable
class OutputDevice(Protocol):
    """A device which supports writing to"""

    async def write(self, key: str, value: ValueType) -> None:
        """Write a value given its key and the new value"""


@runtime_checkable
class IODevice(InputDevice, OutputDevice, Protocol):
    """A device which supports reading and writing"""


class AbstractDeviceState(Protocol):
    """A state of a device"""

    def __eq__(self, other: "AbstractDeviceState") -> bool:
        """Check if the state is equal to another state"""


@runtime_checkable
class StoppableDevice(Protocol):
    """A device that can be stopped"""

    async def stop(self) -> None:
        """Stop the device"""


@runtime_checkable
class StateStreamingDevice(Protocol):
    """A device which supports streaming its state"""

    def state_stream(self, rate: int) -> AsyncIterable[AbstractDeviceState]:
        """Read a value given its key
        Args:
            rate: The rate at which the state should be streamed
        """


class AbstractRobot(Device):
    """An interface for real and simulated robots"""

    def __init__(self, **kwargs):
        super().__init__(**kwargs)
        self._motion_recording: list[list[MotionState]] = []
        self._execution_duration = 0.0
        self._counter = 0

    def recorded_trajectories(self) -> list[list[MotionState]]:
        """Return the recorded motions of a robot. Each list is collected from sync to sync."""
        return self._motion_recording

    def execution_duration(self) -> float:
        """Return the time to execute the movement"""
        return self._execution_duration

    @abstractmethod
    async def _plan(self, actions: list[Action], tcp: str) -> models.JointTrajectory:
        """Plan a trajectory for the given actions

        Args:
            actions (list[Action] | Action): The actions to be planned. Can be a single action or a list of actions.
                Only motion actions are considered for planning.
            tcp (str): The identifier of the tool center point (TCP)

        Returns:
            wb.models.JointTrajectory: The planned joint trajectory
        """

    async def plan(self, actions: list[Action] | Action, tcp: str) -> models.JointTrajectory:
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

        return await self._plan(actions, tcp)

    @abstractmethod
    async def _execute(
        self,
        joint_trajectory: models.JointTrajectory,
        tcp: str,
        actions: list[Action],
        on_movement: Callable[[MotionState], None],
        movement_controller: MovementController | None,
    ):
        """Execute a planned motion

        Args:
            joint_trajectory (wb.models.JointTrajectory): The planned joint trajectory
            tcp (str): The identifier of the tool center point (TCP)
            actions (list[Action] | Action | None): The actions to be executed. Defaults to None.
            movement_controller (MovementController): The movement controller to be used. Defaults to move_forward
            on_movement (Callable[[MotionState], None]): A callback which is triggered for every movement
        """

    async def execute(
        self,
        joint_trajectory: models.JointTrajectory,
        tcp: str,
        actions: list[Action] | Action | None,
        on_movement: Callable[[MotionState], None] | None,
        movement_controller: MovementController | None,
    ):
        """Execute a planned motion

        Args:
            joint_trajectory (wb.models.JointTrajectory): The planned joint trajectory
            tcp (str): The identifier of the tool center point (TCP)
            actions (list[Action] | Action | None): The actions to be executed. Defaults to None.
            movement_controller (MovementController): The movement controller to be used. Defaults to move_forward
            on_movement (Callable[[MotionState], None]): A callback which is triggered for every movement
        """
        if actions is None:
            actions = []
        elif not isinstance(actions, list):
            actions = [actions]

        self._motion_recording.append([])

        def _on_movement(motion_state_: MotionState):
            self._motion_recording[-1].append(motion_state_)
            if on_movement:
                on_movement(motion_state_)

        await self._execute(
            joint_trajectory,
            tcp,
            actions,
            movement_controller=movement_controller,
            on_movement=_on_movement,
        )

    async def plan_and_execute(
        self, actions: list[Action] | Action, tcp: str, on_movement: Callable[[MotionState], None]
    ):
        joint_trajectory = await self.plan(actions, tcp)
        await self.execute(joint_trajectory, tcp, actions, on_movement, movement_controller=None)

    @abstractmethod
    async def get_state(self, tcp: str | None = None) -> models.MotionGroupStateResponse:
        """Current state (pose, joints) of the robot based on the tcp.

        Args:
            tcp (str): The identifier of the tool center point (TCP) to be used for tcp_pose in response. If not set,
                the flange pose is returned as tcp_pose.

        Returns: the current state of the robot

        """

    @abstractmethod
    async def joints(self) -> tuple:
        """Return the current joint positions of the robot

        Returns: the current joint positions

        """

    @abstractmethod
    async def tcp_pose(self, tcp: str | None = None) -> Pose:
        """Return the current pose of the robot based on the tcp

        Args:
            tcp (str): The identifier of the tool center point (TCP) to be used for tcp_pose in response. If not set,
                the flange pose is returned as tcp_pose.

        Returns: the current pose of the robot

        """

    @abstractmethod
    async def tcps(self) -> list[models.RobotTcp]:
        """Return all TCPs that are configured on the robot with corresponding offset from flange as pose

        Returns: the TCPs of the robot

        """

    @abstractmethod
    async def tcp_names(self) -> list[str]:
        """Return the name of all TCPs that are configured on the robot

        Returns: a list of all TCPs

        """

    @abstractmethod
    async def stop(self):
        """Stop behaviour of the robot"""


class AbstractController(Device):
    @abstractmethod
    def get_robots(self) -> dict[str, AbstractRobot]:
        """Return all robots that are connected to the controller

        Returns: a dict with {robot_name: robot}
        """


class AbstractTimer(Device):
    """A timer"""

    @abstractmethod
    async def __call__(self, duration: float) -> None:
        """Wait for a duration ms"""


class Timer(ConfigurablePeriphery, AbstractTimer):
    """A real timer (blocking the execution)"""

    class Configuration(ConfigurablePeriphery.Configuration):
        type: Literal["timer", "simulated_timer"] = "timer"
        identifier: str = "timer"

    def __init__(self, configuration: Configuration = Configuration()):
        super().__init__(configuration)

    async def __call__(self, duration: float):
        await asyncio.sleep(duration / 1000)


@dataclass
class ExecutionResult:
    motion_group_id: str
    motion_duration: float
    recorded_trajectories: list[list[MotionState]]


class RobotCellKeyError(KeyError):
    pass


class RobotCell:
    """Access a simulated or real robot"""

    _devices: dict

    def __init__(self, timer: AbstractTimer | None = None, **kwargs):
        if timer is None:
            timer = Timer()
        devices = {"timer": timer, **kwargs}
        # TODO: if "timer" has not the same identifier it cannot correctly be serialized/deserialized currently
        for device_name, device in devices.items():
            if device and device_name != device.identifier:
                raise ValueError(
                    f"The device name should match its name in the robotcell but are '{device_name}' and '{device.identifier}'"
                )
        self._devices = devices
        self._device_exit_stack = AsyncExitStack()

    def set_configurations(self, configurations: list[ConfigurablePeriphery.Configuration]):
        """Set the configurations of all devices that are attached to the robot cell

        Args:
            configurations: the configurations of the robot cell

        """
        self._devices.clear()
        self.apply_configurations(configurations)

    def apply_configurations(self, configurations: list[ConfigurablePeriphery.Configuration]):
        """Applies all given device configurations to the robot cell. If the identifier is already in the
        robot cell the device gets overriden.

        Args:
            configurations: the given device configurations

        """
        for configuration in configurations:
            logger.info(f"Setup device with configuration: {configuration}...")
            identifier = configuration.identifier
            result = ConfigurablePeriphery.all_classes[type(configuration)](
                configuration=configuration
            )
            self._devices[identifier] = result

    def to_configurations(self) -> list[ConfigurablePeriphery.Configuration]:
        """Return the configurations of all devices that are attached to the robot cell

        Returns:
            [list[ConfigurablePeriphery.Configuration]]: the configurations of the robot cell

        """
        # TODO: remove 'hasattr(device, "configuration")' See: https://wandelbots.atlassian.net/browse/WP-554
        return [
            device.configuration
            for device in self._devices.values()
            if hasattr(device, "configuration")
        ]

    @classmethod
    def from_configurations(cls, configurations: list[ConfigurablePeriphery.Configuration]):
        """Construct a new robot_cell from the device configurations

        Returns:
            [RobotCell]: the newly created RobotCell

        """
        robot_cell = cls()
        robot_cell.apply_configurations(configurations)
        return robot_cell

    @classmethod
    def from_dict(cls, data):
        AnyConfiguration = Union.__getitem__(tuple(ConfigurablePeriphery.all_classes))

        class RobotCellConfiguration(pydantic.BaseModel):
            devices: list[AnyConfiguration]  # type: ignore

        config = RobotCellConfiguration(devices=data)

        return cls.from_configurations(config.devices)

    def get_controllers(self) -> list[AbstractController]:
        return list(
            controller
            for controller in self._devices.values()
            if isinstance(controller, AbstractController)
        )

    def get_controller(self, name: str) -> AbstractController:
        """Return the controller by its name"""
        controller = self[name]
        if not isinstance(controller, AbstractController):
            raise ValueError(f'Found no controller with name "{name}".')
        return controller

    def get_robots(self) -> dict[str, AbstractRobot]:
        controllers = self.get_controllers()
        return reduce(
            lambda a, b: a.update(b) or a,
            (controller.get_robots() for controller in controllers),
            {},
        )

    def get_robot_ids(self) -> list[str]:
        # TODO this should return a set
        return list(self.get_robots().keys())

    def get_robot(self, robot_id: str) -> AbstractRobot:
        return self.get_robots()[robot_id]

    def get_execution_results(self) -> list[ExecutionResult]:
        return [
            ExecutionResult(
                motion_group_id=robot_id,
                # TODO this is only the duration of the robot movement within a single sync
                # TODO also this raises if there is no robot configured even for robotless skills
                motion_duration=robot.execution_duration(),
                recorded_trajectories=robot.recorded_trajectories(),
            )
            for robot_id, robot in self.get_robots().items()
        ]

    @asyncstdlib.cached_property
    async def tcps(self) -> dict[str, set[str]]:
        """Return a mapping of all TCPs to the robots that have them configured"""
        result = defaultdict(set)
        for robot_name, robot in self.get_robots().items():
            tcp_names = await robot.tcp_names()
            for tcp_name in tcp_names:
                result[tcp_name].add(robot_name)
        return result

    @property
    def timer(self) -> AbstractTimer:
        return self["timer"]

    async def stop(self):
        """Stop the robot cell"""
        stoppable_devices = [
            device for device in self._devices.values() if isinstance(device, StoppableDevice)
        ]
        if not stoppable_devices:
            return

        async with anyio.create_task_group() as task_group:
            for device in stoppable_devices:
                task_group.start_soon(device.stop)

    async def state_stream(self, rate: int):
        """Stream the state of the robot cell"""
        state_streaming_devices = [
            device for device in self._devices.values() if isinstance(device, StateStreamingDevice)
        ]
        if not state_streaming_devices:
            return

        state_streams = [device.state_stream(rate) for device in state_streaming_devices]
        async with aiostream.stream.merge(*state_streams).stream() as devices_state_stream:
            async for state in devices_state_stream:
                yield state

    # async def open(self):
    #    for device in self.values():
    #       await self._exit_stack.enter_async_context(device)
    #    await super().open()
    #    raise NotImplementedError()

    # async def close(self):
    #    raise NotImplementedError()

    async def __aenter__(self):
        for device in self._devices.values():
            await self._device_exit_stack.enter_async_context(device)
        return self

    async def __aexit__(self, exc_type, exc_value, exc_traceback):
        return await self._device_exit_stack.__aexit__(exc_type, exc_value, exc_traceback)

    def __getitem__(self, item):
        try:
            return self._devices.__getitem__(item)
        except KeyError as exc:
            raise RobotCellKeyError(item) from exc

    def __delitem__(self, key):
        try:
            return self._devices.__delitem__(key)
        except KeyError as exc:
            raise RobotCellKeyError(key) from exc
