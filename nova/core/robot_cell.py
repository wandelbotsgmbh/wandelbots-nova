import asyncio
from abc import ABC, abstractmethod
from collections import defaultdict
from contextlib import AsyncExitStack
from functools import reduce
from typing import (
    AsyncIterable,
    Awaitable,
    ClassVar,
    Generic,
    Literal,
    Protocol,
    TypeVar,
    Union,
    final,
    get_origin,
    get_type_hints,
    runtime_checkable,
)

import anyio
import asyncstdlib
import pydantic
from aiostream import pipe, stream

from nova import api
from nova.actions import Action, MovementController
from nova.core import logger
from nova.core.movement_controller import movement_to_motion_state
from nova.types import MotionState, MovementResponse, Pose, RobotState


class RobotCellError(Exception):
    """Base exception for all robot cell specific error"""


class RobotMotionError(RobotCellError):
    """Robot can not move as requested"""


class RobotCellKeyError(KeyError):
    pass


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
            id: A unique id to reference the periphery
        """

        model_config = pydantic.ConfigDict(frozen=True)

        type: str
        id: str

    _configuration: Configuration

    def __init__(self, configuration: Configuration, **kwargs):
        super().__init__(**kwargs)
        self._configuration = configuration

    @property
    def configuration(self):
        return self._configuration

    @property
    def id(self):
        return self.configuration.id


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


T = TypeVar("T")


class AsyncCallableDevice(Generic[T], Device):
    """An awaitable external function or service in the robot cell"""

    async def __call__(self, *args, **kwargs) -> Awaitable[T]:
        if not self._is_active:
            raise ValueError("The device is not activated.")
        return await self._call(*args)

    @abstractmethod
    async def _call(self, key, *args) -> Awaitable[T]:
        """The implementation of the call method. AbstractAwaitable guarantees that the device is activated.

        Args:
            key: A key that represents the id of the external function or service that is called
            *args: Parameters of the external callable

        Returns: the returned values of the external called function or service
        """


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

    def stream_state(self, rate_msecs: int) -> AsyncIterable[AbstractDeviceState]:
        """Read a value given its key
        Args:
            rate: The rate at which the state should be streamed
        """


class AbstractRobot(Device):
    """An interface for real and simulated robots"""

    _id: str

    def __init__(self, id: str, **kwargs):
        super().__init__(**kwargs)
        self._id = id

    @property
    def id(self):
        return self._id

    @abstractmethod
    async def _plan(
        self,
        actions: list[Action],
        tcp: str,
        start_joint_position: tuple[float, ...] | None = None,
        optimizer_setup: api.models.OptimizerSetup | None = None,
    ) -> api.models.JointTrajectory:
        """Plan a trajectory for the given actions

        Args:
            actions (list[Action] | Action): The actions to be planned. Can be a single action or a list of actions.
                Only motion actions are considered for planning.
            tcp (str): The id of the tool center point (TCP)
            start_joint_position (tuple[float, ...] | None): The starting joint position. If None, the current joint

        Returns:
            api.models.JointTrajectory: The planned joint trajectory
        """

    async def plan(
        self,
        actions: list[Action] | Action,
        tcp: str,
        start_joint_position: tuple[float, ...] | None = None,
        optimizer_setup: api.models.OptimizerSetup | None = None,
    ) -> api.models.JointTrajectory:
        """Plan a trajectory for the given actions

        Args:
            actions (list[Action] | Action): The actions to be planned. Can be a single action or a list of actions.
                Only motion actions are considered for planning.
            tcp (str): The id of the tool center point (TCP)
            start_joint_position: the initial position of the robot
            start_joint_position (tuple[float, ...] | None): The starting joint position. If None, the current joint
            optimizer_setup (api.models.OptimizerSetup | None): The optimizer setup to be used for planning

        Returns:
            api.models.JointTrajectory: The planned joint trajectory
        """
        if not isinstance(actions, list):
            actions = [actions]

        if len(actions) == 0:
            raise ValueError("No actions provided")

        return await self._plan(
            actions=actions,
            tcp=tcp,
            start_joint_position=start_joint_position,
            optimizer_setup=optimizer_setup,
        )

    @abstractmethod
    def _execute(
        self,
        joint_trajectory: api.models.JointTrajectory,
        tcp: str,
        actions: list[Action],
        movement_controller: MovementController | None,
    ) -> AsyncIterable[MovementResponse]:
        """Execute a planned motion

        Args:
            joint_trajectory (api.models.JointTrajectory): The planned joint trajectory
            tcp (str): The id of the tool center point (TCP)
            actions (list[Action] | Action | None): The actions to be executed. Defaults to None.
            movement_controller (MovementController): The movement controller to be used. Defaults to move_forward
        """

    async def stream_execute(
        self,
        joint_trajectory: api.models.JointTrajectory,
        tcp: str,
        actions: list[Action] | Action,
        movement_controller: MovementController | None = None,
    ) -> AsyncIterable[MotionState]:
        """Execute a planned motion

        Args:
            joint_trajectory (api.models.JointTrajectory): The planned joint trajectory
            tcp (str): The id of the tool center point (TCP)
            actions (list[Action] | Action | None): The actions to be executed. Defaults to None.
            movement_controller (MovementController): The movement controller to be used. Defaults to move_forward
        """
        if not isinstance(actions, list):
            actions = [actions]

        def is_movement(movement_response: MovementResponse) -> bool:
            return any(
                (
                    isinstance(movement_response, api.models.ExecuteTrajectoryResponse)
                    and isinstance(movement_response.actual_instance, api.models.Movement),
                    isinstance(movement_response, api.models.StreamMoveResponse),
                )
            )

        def movement_response_to_motion_state(
            movement_response: MovementResponse, *_
        ) -> MotionState:
            if isinstance(movement_response, api.models.ExecuteTrajectoryResponse):
                return movement_to_motion_state(movement_response.actual_instance)
            if isinstance(movement_response, api.models.StreamMoveResponse):
                return movement_to_motion_state(movement_response)
            assert False, f"Unexpected movement response: {movement_response}"

        execute_response_stream = self._execute(
            joint_trajectory, tcp, actions, movement_controller=movement_controller
        )
        motion_states = (
            stream.iterate(execute_response_stream)
            | pipe.filter(is_movement)
            | pipe.map(movement_response_to_motion_state)
        )

        async with motion_states.stream() as motion_states_stream:
            async for motion_state in motion_states_stream:
                yield motion_state

    async def execute(
        self,
        joint_trajectory: api.models.JointTrajectory,
        tcp: str,
        actions: list[Action] | Action,
        movement_controller: MovementController | None = None,
    ) -> None:
        """Execute a planned motion

        Args:
            joint_trajectory (api.models.JointTrajectory): The planned joint trajectory
            tcp (str): The id of the tool center point (TCP)
            actions (list[Action] | Action): The actions to be executed.
            movement_controller (MovementController): The movement controller to be used. Defaults to move_forward
        """
        async for _ in self.stream_execute(
            joint_trajectory, tcp, actions, movement_controller=movement_controller
        ):
            pass

    async def stream_plan_and_execute(
        self,
        actions: list[Action] | Action,
        tcp: str,
        start_joint_position: tuple[float, ...] | None = None,
    ) -> AsyncIterable[MotionState]:
        joint_trajectory = await self.plan(actions, tcp, start_joint_position=start_joint_position)
        async for motion_state in self.stream_execute(joint_trajectory, tcp, actions):
            yield motion_state

    async def plan_and_execute(
        self,
        actions: list[Action] | Action,
        tcp: str,
        start_joint_position: tuple[float, ...] | None = None,
    ) -> None:
        joint_trajectory = await self.plan(actions, tcp, start_joint_position=start_joint_position)
        await self.execute(joint_trajectory, tcp, actions, movement_controller=None)

    @abstractmethod
    async def get_state(self, tcp: str | None = None) -> RobotState:
        """Current state (pose, joints) of the robot based on the tcp.

        Args:
            tcp (str): The id of the tool center point (TCP) to be used for tcp_pose in response. If not set,
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
            tcp (str): The id of the tool center point (TCP) to be used for tcp_pose in response. If not set,
                the flange pose is returned as tcp_pose.

        Returns: the current pose of the robot

        """

    @abstractmethod
    async def tcps(self) -> list[api.models.RobotTcp]:
        """Return all TCPs that are configured on the robot with corresponding offset from flange as pose

        Returns: the TCPs of the robot

        """

    @abstractmethod
    async def tcp_names(self) -> list[str]:
        """Return the name of all TCPs that are configured on the robot

        Returns: a list of all TCPs

        """

    @abstractmethod
    async def active_tcp(self) -> api.models.RobotTcp:
        """Return the active TCP of the robot

        Returns: the active TCP

        """

    @abstractmethod
    async def active_tcp_name(self) -> str:
        """Return the name of the active TCP of the robot

        Returns: the name of the active TCP

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
        id: str = "timer"

    def __init__(self, configuration: Configuration = Configuration()):
        super().__init__(configuration)

    async def __call__(self, duration: float):
        await asyncio.sleep(duration / 1000)


class RobotCell:
    """Access a simulated or real robot"""

    _devices: dict

    def __init__(self, timer: AbstractTimer | None = None, **kwargs):
        if timer is None:
            timer = Timer()
        devices = {"timer": timer, **kwargs}
        # TODO: if "timer" has not the same id it cannot correctly be serialized/deserialized currently
        for device_name, device in devices.items():
            if device and device_name != device.id:
                raise ValueError(
                    f"The device name should match its name in the robotcell but are '{device_name}' and '{device.id}'"
                )
        self._devices = devices
        self._device_exit_stack = AsyncExitStack()

    @property
    def devices(self) -> dict:
        return self._devices

    def set_configurations(self, configurations: list[ConfigurablePeriphery.Configuration]):
        """Set the configurations of all devices that are attached to the robot cell

        Args:
            configurations: the configurations of the robot cell

        """
        self._devices.clear()
        self.apply_configurations(configurations)

    def apply_configurations(self, configurations: list[ConfigurablePeriphery.Configuration]):
        """Applies all given device configurations to the robot cell. If the id is already in the
        robot cell the device gets overriden.

        Args:
            configurations: the given device configurations

        """
        for configuration in configurations:
            logger.info(f"Setup device with configuration: {configuration}...")
            device_id = configuration.id
            result = ConfigurablePeriphery.all_classes[type(configuration)](
                configuration=configuration
            )
            self._devices[device_id] = result

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

    async def stream_state(self, rate_msecs: int):
        """Stream the state of the robot cell"""
        state_streaming_devices = [
            device for device in self._devices.values() if isinstance(device, StateStreamingDevice)
        ]
        if not state_streaming_devices:
            return

        state_streams = [device.stream_state(rate_msecs) for device in state_streaming_devices]
        async with stream.merge(*state_streams).stream() as devices_state_stream:
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
