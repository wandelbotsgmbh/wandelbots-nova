from typing import AsyncGenerator, Literal, Sized

from nova import api
from nova.cell.robot_cell import AbstractController, AbstractRobot, IODevice, ValueType
from nova.core.gateway import NovaDevice
from nova.core.io import IOAccess
from nova.core.motion_group import MotionGroup


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
            api_gateway=self._nova_api,
            cell=self.configuration.cell_id,
            controller_id=self.configuration.controller_id,
        )

    @property
    def controller_id(self) -> str:
        """Returns the unique controller ID."""
        return self.configuration.controller_id

    async def open(self):
        self._motion_group_ids = (await self._fetch_description()).connected_motion_groups
        return self

    async def close(self):
        # RPS-1174: when a motion group is deactivated, RAE closes all open connections
        #           this behaviour is not desired in some cases,
        #           so for now we will not deactivate for the user
        pass

    def __len__(self) -> int:
        # TODO What is this for? Is it still needed when motion group activation is gone?
        return len(self._motion_group_ids)

    def motion_group(self, motion_group_id: str) -> MotionGroup:
        """Returns motion group with specific id.

        Args:
            motion_group_id (str): The ID of the motion group.

        Returns:
            MotionGroup: A MotionGroup instance corresponding to the given ID.
        """
        return MotionGroup(
            api_gateway=self._nova_api,
            cell=self.configuration.cell_id,
            controller_id=self.configuration.controller_id,
            motion_group_id=motion_group_id,
        )

    def __getitem__(self, motion_group_id: int) -> MotionGroup:
        return self.motion_group(f"{motion_group_id}@{self.configuration.controller_id}")

    async def motion_groups(self) -> list[MotionGroup]:
        """Retrieves a list of `MotionGroup` instances for all motion groups attached to this controller.

        Returns:
            list[MotionGroup]: All motion groups as `MotionGroup` objects.
        """
        raise NotImplementedError(
            "This can be implemented when the new controllers/{controller}/description endpoint is available."
        )
        motion_group_ids = await self._nova_api.controller_description_api
        return [self.motion_group(motion_group_id) for motion_group_id in motion_group_ids]

    def get_motion_groups(self) -> dict[str, AbstractRobot]:
        """Retrieves a dictionary of motion group IDs to their corresponding robots.

        Note:
            This method interprets motion group IDs to create corresponding `MotionGroup`
            objects and returns them as `AbstractRobot` references.

        Returns:
            dict[str, AbstractRobot]: A mapping of motion group ID to `MotionGroup` instance.
        """
        # TODO Why do we have get_robots() and motion_groups()?
        return {
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
        async for state in self._nova_api.stream_robot_controller_state(
            cell=self.configuration.cell_id,
            controller_id=self.configuration.controller_id,
            response_rate=rate_msecs,
        ):
            yield state

    async def _fetch_description(self):
        return await self._nova_api.controller_api.get_controller_description(
            self.configuration.cell_id, self.configuration.controller_id
        )
