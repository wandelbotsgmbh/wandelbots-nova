from typing import AsyncGenerator, Literal, Sized

from wandelbots_api_client.models.robot_system_mode import RobotSystemMode

from nova import api
from nova.cell.robot_cell import AbstractController, AbstractRobot, IODevice, ValueType
from nova.core.gateway import NovaDevice
from nova.core.io import IOAccess
from nova.core.motion_group import MotionGroup
from nova.logging import logger


class Controller(Sized, AbstractController, NovaDevice, IODevice):
    """
    Represents a Nova controller, managing motion groups and IO interactions.
    """

    class Configuration(NovaDevice.Configuration):
        type: Literal["controller"] = "controller"  # type: ignore[assignment]
        id: str = "controller"
        cell_id: str
        controller_id: str

    def __init__(self, configuration: Configuration):
        super().__init__(configuration)
        self.configuration: Controller.Configuration
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

    async def open(self) -> None:
        """Open the controller connection."""
        await super().open()
        self._activated_motion_group_ids = await self._nova_api.activate_all_motion_groups(
            cell=self.configuration.cell_id, controller=self.configuration.controller_id
        )
        logger.info(f"Activated motion groups: {self._activated_motion_group_ids}")

    async def close(self):
        # RPS-1174: when a motion group is deactivated, RAE closes all open connections
        #           this behaviour is not desired in some cases,
        #           so for now we will not deactivate for the user
        pass

    def __len__(self) -> int:
        return len(self._activated_motion_group_ids)

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
            motion_group_id=motion_group_id,
            controller=self,
        )

    def __getitem__(self, motion_group_id: int) -> MotionGroup:
        return self.motion_group(f"{motion_group_id}@{self.configuration.controller_id}")

    async def activated_motion_group_ids(self) -> list[str]:
        """Activates and retrieves the list of motion group IDs available on this controller.

        The system automatically activates all motion groups on the associated controller.

        Returns:
            list[str]: A list of activated motion group IDs (e.g., ["0@controller_id"]).
        """
        return await self._nova_api.activate_all_motion_groups(
            cell=self.configuration.cell_id, controller=self.configuration.controller_id
        )

    async def activated_motion_groups(self) -> list[MotionGroup]:
        """Retrieves a list of `MotionGroup` instances for all activated motion groups.

        Returns:
            list[MotionGroup]: All activated motion groups as `MotionGroup` objects.
        """
        motion_group_ids = await self.activated_motion_group_ids()
        return [self.motion_group(motion_group_id) for motion_group_id in motion_group_ids]

    def get_motion_groups(self) -> dict[str, AbstractRobot]:
        """Retrieves a dictionary of motion group IDs to their corresponding robots.

        Note:
            This method interprets motion group IDs to create corresponding `MotionGroup`
            objects and returns them as `AbstractRobot` references.

        Returns:
            dict[str, AbstractRobot]: A mapping of motion group ID to `MotionGroup` instance.
        """
        return {
            motion_group_id: self.motion_group(motion_group_id)
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

    async def set_default_mode(self, mode: RobotSystemMode) -> None:
        """
        Switch between monitor and control usage as default for this robot controller.

        Monitoring mode is used to read information from the robot controller and control mode
        is used to command the robot system. As long as the robot controller is connected via
        network monitoring mode is always possible.

        To switch to control mode the robot controller must be in automatic or manual operating
        mode and safety state 'normal' or 'reduced'. If the robot controller is in manual
        operating mode, you have manually confirm the control usage activation on the robot
        control panel.

        Important: When switching from CONTROL to MONITOR mode, the robot's brakes are
        automatically activated for safety. When switching from MONITOR to CONTROL mode,
        the brakes are released. Frequent mode switching causes brake activation/deactivation
        cycles which significantly impact cycle time performance. For optimal performance,
        minimize mode switches by staying in CONTROL mode during execution sessions.

        Args:
            mode: The mode to set. Use RobotSystemMode.ROBOT_SYSTEM_MODE_MONITOR or RobotSystemMode.ROBOT_SYSTEM_MODE_CONTROL.

        Raises:
            ValueError: If mode is not a valid controller mode.
        """
        # Validate mode parameter
        if mode not in [
            RobotSystemMode.ROBOT_SYSTEM_MODE_MONITOR,
            RobotSystemMode.ROBOT_SYSTEM_MODE_CONTROL,
        ]:
            raise ValueError(
                f"Invalid mode: {mode}. Must be ROBOT_SYSTEM_MODE_MONITOR or ROBOT_SYSTEM_MODE_CONTROL"
            )

        await self._nova_api.controller_api.set_default_mode(
            cell=self.configuration.cell_id, controller=self.configuration.controller_id, mode=mode
        )

    async def get_current_mode(self) -> RobotSystemMode:
        """
        Get the current default mode of the robot controller.

        Returns:
            RobotSystemMode: The current mode (ROBOT_SYSTEM_MODE_MONITOR or ROBOT_SYSTEM_MODE_CONTROL).
        """
        response = await self._nova_api.controller_api.get_mode(
            cell=self.configuration.cell_id, controller=self.configuration.controller_id
        )
        return response.robot_system_mode

    async def stream_state(
        self, rate_msecs
    ) -> AsyncGenerator[api.models.RobotControllerState, None]:
        async for state in self._nova_api.stream_robot_controller_state(
            cell=self.configuration.cell_id,
            controller_id=self.configuration.controller_id,
            response_rate=rate_msecs,
        ):
            yield state
