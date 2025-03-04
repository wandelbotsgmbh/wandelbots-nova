from typing import Literal, Sized

from nova.core import logger
from nova.core.gateway import NovaDevice
from nova.core.io import IOAccess
from nova.core.motion_group import MotionGroup
from nova.core.robot_cell import AbstractController, AbstractRobot, IODevice, ValueType


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
        self._controller_api = self._nova_api.controller_api
        self._motion_group_api = self._nova_api.motion_group_api
        self._activated_motion_group_ids: list[str] = []
        self._io_access = IOAccess(
            api_gateway=self._nova_api,
            cell=self.configuration.cell_id,
            controller_id=self.configuration.controller_id,
        )

    @property
    def controller_id(self) -> str:
        """Returns the unique controller ID."""
        return self.configuration.controller_id

    async def open(self):
        """Activates all motion groups."""
        motion_group_ids = await self.activated_motion_group_ids()
        self._activated_motion_group_ids = motion_group_ids
        logger.info(f"Found motion group {motion_group_ids}")
        return self

    async def close(self):
        # RPS-1174: when a motion group is deactivated, RAE closes all open connections
        #           this behaviour is not desired in some cases,
        #           so for now we will not deactivate for the user
        pass

    def __len__(self) -> int:
        return len(self._activated_motion_group_ids)

    def motion_group(self, motion_group_id: int = 0) -> MotionGroup:
        """Returns motion group with specific id.

        Args:
            motion_group_id (int): The numerical ID of the motion group.
                Defaults to 0.

        Returns:
            MotionGroup: A MotionGroup instance corresponding to the given ID.
        """
        return MotionGroup(
            api_gateway=self._nova_api,
            cell=self.configuration.cell_id,
            motion_group_id=f"{motion_group_id}@{self.configuration.controller_id}",
        )

    def __getitem__(self, motion_group_id: int) -> MotionGroup:
        return self.motion_group(motion_group_id)

    async def activated_motion_group_ids(self) -> list[str]:
        """Activates and retrieves the list of motion group IDs available on this controller.

        The system automatically activates all motion groups on the associated controller.

        Returns:
            list[str]: A list of activated motion group IDs (e.g., ["0@controller_id"]).
        """
        activate_all_motion_groups_response = (
            await self._motion_group_api.activate_all_motion_groups(
                cell=self.configuration.cell_id, controller=self.configuration.controller_id
            )
        )
        motion_groups = activate_all_motion_groups_response.instances
        return [mg.motion_group for mg in motion_groups]

    async def activated_motion_groups(self) -> list[MotionGroup]:
        """Retrieves a list of `MotionGroup` instances for all activated motion groups.

        Returns:
            list[MotionGroup]: All activated motion groups as `MotionGroup` objects.
        """
        motion_group_ids = await self.activated_motion_group_ids()
        return [self.motion_group(int(mg.split("@")[0])) for mg in motion_group_ids]

    def get_robots(self) -> dict[str, AbstractRobot]:
        """Retrieves a dictionary of motion group IDs to their corresponding robots.

        Note:
            This method interprets motion group IDs to create corresponding `MotionGroup`
            objects and returns them as `AbstractRobot` references.

        Returns:
            dict[str, AbstractRobot]: A mapping of motion group ID to `MotionGroup` instance.
        """
        return {
            motion_group_id: self.motion_group(int(motion_group_id.split("@")[0]))
            for motion_group_id in self._activated_motion_group_ids
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
