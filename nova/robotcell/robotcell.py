from __future__ import annotations

from abc import abstractmethod
from collections import defaultdict
from collections.abc import Awaitable
from contextlib import AsyncExitStack
from dataclasses import dataclass
from functools import reduce
from typing import Generic, Literal, TypeVar, Union

import aiostream
import anyio
import asyncstdlib
import pydantic
from loguru import logger
from pyjectory import datatypes as dts
from pyjectory.datatypes.collision_scene import Collider, CollisionRobot, CollisionRobotConfiguration, CollisionScene
from pyplan_mc import MotionGroupModel
from pyplan_mc.collision import make_default_robot_model

from pyriphery.robotics.configurable_collision_scene import ConfigurableCollisionScene
from pyriphery.robotics.configurable_periphery import ConfigurablePeriphery
from pyriphery.robotics.controller import AbstractController
from pyriphery.robotics.device import Device, StateStreamingDevice, StoppableDevice
from pyriphery.robotics.robot import AbstractRobot
from pyriphery.robotics.type_conversions.to_pyjectory import collision_object_dict_to_pyjectory


class RobotCellError(Exception):
    """Base exception for all robot cell specific error"""


class RobotCellKeyError(KeyError):
    pass


class RobotMotionError(RobotCellError):
    """Robot can not move as requested"""


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
        await anyio.sleep(duration / 1000)


@dataclass
class ExecutionResult:
    motion_group_id: str
    motion_duration: float
    recorded_trajectories: list[list[dts.MotionState]]


class RobotCell:
    """Aggregates all devices/periphery that are to the robot cell"""

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
        super().__init__(devices)
        self._device_exit_stack = AsyncExitStack()

    def set_configurations(self, configurations: list[ConfigurablePeriphery.Configuration]):
        """Set the configurations of all devices that are attached to the robot cell

        Args:
            configurations: the configurations of the robot cell

        """
        self.clear()
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
            result = ConfigurablePeriphery.all_classes[type(configuration)](configuration=configuration)
            self[identifier] = result

    def to_configurations(self) -> list[ConfigurablePeriphery.Configuration]:
        """Return the configurations of all devices that are attached to the robot cell

        Returns:
            [list[ConfigurablePeriphery.Configuration]]: the configurations of the robot cell

        """
        # TODO: remove 'hasattr(device, "configuration")' See: https://wandelbots.atlassian.net/browse/WP-554
        return [device.configuration for device in self.values() if hasattr(device, "configuration")]

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
        return list(controller for controller in self.values() if isinstance(controller, AbstractController))

    def get_controller(self, name: str) -> AbstractController:
        """Return the controller by its name"""
        controller = self[name]
        if not isinstance(controller, AbstractController):
            raise ValueError(f'Found no controller with name "{name}".')
        return controller

    def get_robots(self) -> dict[str, AbstractRobot]:
        controllers = self.get_controllers()
        return reduce(lambda a, b: a.update(b) or a, (controller.get_robots() for controller in controllers), {})

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

    def _get_configurable_collision_scene(self) -> ConfigurableCollisionScene | None:
        for v in self.values():
            if isinstance(v, ConfigurableCollisionScene):
                return v
        return None

    async def _initialize_collision_robots(
        self, collision_robot_configs: dict[str, CollisionRobotConfiguration]
    ) -> dict[str, CollisionRobot]:
        collision_robots = {}
        for robot_identifier, robot_config in collision_robot_configs.items():
            robot = self.get_robots()[robot_identifier]

            link_shapes = robot_config.link_attachements.copy()
            if robot_config.use_default_link_shapes:
                robot_model = MotionGroupModel(await robot.get_motion_group_model())
                pp_links = make_default_robot_model(robot_model)
                if len(pp_links) == 0:
                    raise ValueError("No default link shapes available for robot")
                default_link_shapes: dict[int, dict[str, Collider]] = collision_object_dict_to_pyjectory(pp_links)
                # combine default_link_shapes with link_shapes
                for link_number, link_dict in default_link_shapes.items():
                    if link_number not in link_shapes:
                        link_shapes[link_number] = {}
                    for link_name, link_shape in link_dict.items():
                        name = link_name if link_name not in link_shapes[link_number] else f"{link_name}_default"
                        link_shapes[link_number][name] = link_shape

            mounting = await robot.get_mounting()
            dh_parameters = await robot.get_dh_parameters()
            collision_robots[robot_identifier] = CollisionRobot(
                mounting=mounting,
                dh_parameters=dh_parameters,
                joint_positions=[],
                links=link_shapes,
                tool=robot_config.tool,
            )
        return collision_robots

    async def _initialize_collision_scene(self) -> None:
        scene = self._get_configurable_collision_scene()
        if scene is not None:
            scene.current_scene = CollisionScene()
            scene.current_scene.robots = await self._initialize_collision_robots(
                scene.configuration.robot_configurations
            )
            scene.current_scene.static_colliders = scene.configuration.static_colliders.copy()

    async def _update_collision_robot_joints(self) -> None:
        scene = self._get_configurable_collision_scene()
        if scene is None:
            return None
        if scene.current_scene is None:
            return None

        for robot_identifier, collision_robot in scene.current_scene.robots.items():
            robot = self.get_robots()[robot_identifier]
            tcp = await robot.get_active_tcp_name()
            robot_state = await robot.get_state(tcp)
            if robot_state.joints is not None:
                collision_robot.joint_positions = robot_state.joints

    async def get_current_collision_scene(self) -> CollisionScene | None:
        scene = self._get_configurable_collision_scene()
        if scene is None:
            return None
        if scene.current_scene is None:
            await self._initialize_collision_scene()
        await self._update_collision_robot_joints()
        return scene.current_scene

    @asyncstdlib.cached_property
    async def tcps(self) -> dict[str, set[str]]:
        """Return a mapping of all TCPs to the robots that have them configured"""
        result = defaultdict(set)
        for robot_name, robot in self.get_robots().items():
            for tcp_name in await robot.get_tcps():
                result[tcp_name].add(robot_name)
        return result

    @property
    def timer(self) -> AbstractTimer:
        return self["timer"]

    async def stop(self):
        """Stop the robot cell"""
        stoppable_devices = [device for device in self.values() if isinstance(device, StoppableDevice)]
        if not stoppable_devices:
            return

        async with anyio.create_task_group() as task_group:
            for device in stoppable_devices:
                task_group.start_soon(device.stop)

    async def state_stream(self, rate: int):
        """Stream the state of the robot cell"""
        state_streaming_devices = [device for device in self.values() if isinstance(device, StateStreamingDevice)]
        if not state_streaming_devices:
            return

        state_streams = [device.state_stream(rate) for device in state_streaming_devices]
        async with aiostream.stream.merge(*state_streams).stream() as devices_state_stream:
            async for state in devices_state_stream:
                yield state

    async def __aenter__(self):
        for device in self.values():
            await self._device_exit_stack.enter_async_context(device)
        return self

    async def __aexit__(self, exc_type, exc_value, exc_traceback):
        return await self._device_exit_stack.__aexit__(exc_type, exc_value, exc_traceback)

    def __getitem__(self, item):
        try:
            return super().__getitem__(item)
        except KeyError as exc:
            raise RobotCellKeyError(item) from exc

    def __delitem__(self, key):
        try:
            return super().__delitem__(key)
        except KeyError as exc:
            raise RobotCellKeyError(key) from exc
