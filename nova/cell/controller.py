from typing import AsyncGenerator, Literal, Sized

from nova import api
from nova.core.gateway import NovaDevice

from .io import IOAccess
from .motion_group import MotionGroup
from .robot_cell import AbstractController, AbstractRobot, IODevice, ValueType


class Controller(Sized, AbstractController, NovaDevice, IODevice):
    """
    Represents a Nova controller, managing motion groups and IO interactions.
    """

    class Configuration(NovaDevice.Configuration):
        type: Literal["controller"] = "controller"
        id: str = "controller"
        cell_id: str
        controller_id: str

    def __init__(self, configuration: Configuration):
        super().__init__(configuration)
        self._motion_group_ids = None
        self._io_access = IOAccess(
            api_client=self._nova_api,
            cell=self.configuration.cell_id,
            controller_id=self.configuration.controller_id,
        )

    @property
    def id(self) -> str:
        """The unique identifier for this controller in the shape "controller_id" e.g. "ur10e".

        Returns:
            str: The unique identifier for this controller.
        """
        return self.configuration.controller_id

    async def open(self):
        self._motion_group_ids = (await self._fetch_description()).connected_motion_groups
        await super().open()
        return self

    async def close(self):
        # RPS-1174: when a motion group is deactivated, RAE closes all open connections
        #           this behaviour is not desired in some cases,
        #           so for now we will not deactivate for the user
        await super().close()

    def __len__(self) -> int:
        # TODO What is this for? Is it still needed when motion group activation is gone?
        return len(self._motion_group_ids) if self._motion_group_ids is not None else 0

    def motion_group(self, motion_group_id: str) -> MotionGroup:
        """Returns motion group with specific id.

        Args:
            motion_group_id (str): The ID of the motion group.

        Returns:
            MotionGroup: A MotionGroup instance corresponding to the given ID.
        """
        return MotionGroup(
            api_client=self._nova_api,
            cell=self.configuration.cell_id,
            controller_id=self.id,
            motion_group_id=motion_group_id,
        )

    def __getitem__(self, motion_group_id: int) -> MotionGroup:
        return self.motion_group(f"{motion_group_id}@{self.id}")

    async def motion_groups(self) -> list[MotionGroup]:
        """Retrieves a list of `MotionGroup` instances for all motion groups attached to this controller.

        Returns:
            list[MotionGroup]: All motion groups as `MotionGroup` objects.
        """
        motion_group_description = await self._nova_api.controller_api.get_controller_description(
            cell=self.configuration.cell_id, controller=self.id
        )
        return [
            self.motion_group(motion_group_id)
            for motion_group_id in motion_group_description.connected_motion_groups
        ]

    def get_motion_groups(self) -> dict[str, AbstractRobot]:
        """Retrieves a dictionary of motion group IDs to their corresponding robots.

        Note:
            This method interprets motion group IDs to create corresponding `MotionGroup`
            objects and returns them as `AbstractRobot` references.

        Returns:
            dict[str, AbstractRobot]: A mapping of motion group ID to `MotionGroup` instance.
        """
        if self._motion_group_ids is None:
            raise ValueError("Controller is not opened")
        return {  # type: ignore[unreachable]
            motion_group_id: self.motion_group(motion_group_id)
            for motion_group_id in self._motion_group_ids
        }

    async def read(self, key: str) -> ValueType:
        """Reads an IO value from the controller.

        Args:
            key (str): The IO key name.

        Returns:
            ValueType: The value associated with the specified IO key.
        """
        return await self._io_access.read(key)

    async def write(self, key: str, value: ValueType) -> None:
        """Writes an IO value to the controller.

        Args:
            key (str): The IO key name.
            value (ValueType): The value to write to the specified IO key.
        """
        return await self._io_access.write(key, value)

    async def stream_state(
        self, rate_msecs
    ) -> AsyncGenerator[api.models.RobotControllerState, None]:
        """
        Stream the robot controller state.
        """
        async for state in self._nova_api.controller_api.stream_robot_controller_state(
            cell=self.configuration.cell_id, controller=self.id, response_rate=rate_msecs
        ):
            yield state

    async def _fetch_description(self):
        return await self._nova_api.controller_api.get_controller_description(
            self.configuration.cell_id, self.id
        )

    async def get_estop(self) -> bool:
        """Get the emergency stop state for the controller. Works on virtual controllers only.

        Returns:
            bool: Whether the emergency stop is active (True) or not (False).

        Raises:
            NotImplementedError: If called on a non-virtual controller.
        """
        flag = await self._nova_api.virtual_controller_api.get_emergency_stop(
            cell=self.configuration.cell_id, controller=self.id
        )
        return flag.active

    async def set_estop(self, active: bool):
        """Set the emergency stop state for the controller. Works on virtual controllers only.

        Args:
            active (bool): Whether to activate (True) or deactivate (False) the emergency stop.

        Raises:
            NotImplementedError: If called on a non-virtual controller.
        """
        await self._nova_api.virtual_controller_api.set_emergency_stop(
            cell=self.configuration.cell_id, controller=self.id, active=active
        )
