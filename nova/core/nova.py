from __future__ import annotations

import asyncio
from math import pi

from decouple import config

from nova import api
from nova.core import logger
from nova.core.controller import Controller
from nova.core.exceptions import ControllerNotFound
from nova.core.robot_cell import RobotCell
from nova.integrations.api.gateway import ApiGateway

LOG_LEVEL = config("LOG_LEVEL", default="INFO")
CELL_NAME = config("CELL_NAME", default="cell")

MANUFACTURER_HOME_POSITIONS = {
    api.models.Manufacturer.ABB: [0.0, 0.0, 0.0, 0.0, pi / 2, 0.0, 0.0],
    api.models.Manufacturer.FANUC: [0.0, 0.0, 0.0, 0.0, -pi / 2, 0.0, 0.0],
    api.models.Manufacturer.YASKAWA: [0.0, 0.0, 0.0, 0.0, -pi / 2, 0.0, 0.0],
    api.models.Manufacturer.KUKA: [0.0, -pi / 2, pi / 2, 0.0, pi / 2, 0.0, 0.0],
    api.models.Manufacturer.UNIVERSALROBOTS: [0.0, -pi / 2, -pi / 2, -pi / 2, pi / 2, -pi / 2, 0.0],
}


# TODO: could also extend NovaDevice
class Nova:
    """A high-level Nova client for interacting with robot cells and controllers."""

    _api_client: ApiGateway

    def __init__(
        self,
        *,
        host: str | None = None,
        access_token: str | None = None,
        username: str | None = None,
        password: str | None = None,
        version: str = "v1",
        verify_ssl: bool = True,
    ):
        """
        Initialize the Nova client.

        Args:
            host (str | None): The Nova API host.
            access_token (str | None): An access token for the Nova API.
            username (str | None): Username to authenticate with the Nova API.
            password (str | None): Password to authenticate with the Nova API.
            version (str): The API version to use (default: "v1").
            verify_ssl (bool): Whether or not to verify SSL certificates (default: True).
        """
        self._api_client = ApiGateway(
            host=host,
            access_token=access_token,
            username=username,
            password=password,
            version=version,
            verify_ssl=verify_ssl,
        )

    def cell(self, cell_id: str = CELL_NAME) -> Cell:
        """
        Returns the cell object with the given ID.
        """
        return Cell(self._api_client, cell_id)

    async def close(self):
        """
        Closes the underlying API client session.
        """
        return await self._api_client.close()

    async def __aenter__(self):
        return self

    async def __aexit__(self, exc_type, exc_val, exc_tb):
        await self.close()


class Cell:
    """A representation of a robot cell, providing high-level operations on controllers."""

    def __init__(self, api_gateway: ApiGateway, cell_id: str):
        """
        Initializes a Cell instance.

        Args:
            api_gateway (ApiGateway): The underlying gateway for making API calls.
            cell_id (str): The unique identifier for the cell.
        """
        self._api_gateway = api_gateway
        self._cell_id = cell_id

    @property
    def cell_id(self) -> str:
        """
        Returns unique identifier for this cell.

        Returns:
            str: The cell ID.
        """
        return self._cell_id

    async def _get_controller_instances(self) -> list[api.models.ControllerInstance]:
        """
        Return all controller instances associated with this cell.
        """
        return await self._api_gateway.list_controllers(cell=self._cell_id)

    async def _get_controller_instance(self, name: str) -> api.models.ControllerInstance | None:
        """
        Get the controller instance by name, or None if not found.
        """
        return await self._api_gateway.get_controller_instance(cell=self._cell_id, name=name)

    def _create_controller(self, controller_id: str) -> Controller:
        return Controller(
            configuration=Controller.Configuration(
                nova_api=self._api_gateway.host,
                nova_access_token=self._api_gateway.access_token,
                nova_username=self._api_gateway.username,
                nova_password=self._api_gateway.password,
                cell_id=self._cell_id,
                controller_id=controller_id,
                id=controller_id,
            )
        )

    async def _wait_for_controller_ready(self, name: str, timeout: int):
        """
        Wait until the given controller has finished initializing or until timeout.
        """
        iteration = 0
        controller = await self._get_controller_instance(name)

        while iteration < timeout:
            if controller is not None:
                # Check whether it's still initializing
                if controller.error_details in [
                    "Controller not initialized or disposed",
                    "Initializing controller connection.",
                ]:
                    await self._api_gateway.get_current_robot_controller_state(
                        cell=self._cell_id, controller_id=controller.host
                    )
                elif controller.has_error:
                    # As long has an error its being initialized
                    logger.error(controller.error_details)
                else:
                    # Controller is good to go
                    return

            logger.info(f"Waiting for {self._cell_id}/{name} controller availability")
            await asyncio.sleep(1)
            controller = await self._get_controller_instance(name)
            iteration += 1

        raise TimeoutError(f"Timeout waiting for {self._cell_id}/{name} controller availability")

    async def add_virtual_robot_controller(
        self,
        name: str,
        controller_type: api.models.VirtualControllerTypes,
        controller_manufacturer: api.models.Manufacturer,
        timeout: int = 25,
        position: str | None = None,
    ) -> Controller:
        """
        Add a virtual robot controller to the cell.
        """

        home_position = (
            position
            if position is not None
            else str(MANUFACTURER_HOME_POSITIONS.get(controller_manufacturer, [0.0] * 7))
        )

        await self._api_gateway.add_robot_controller(
            cell=self._cell_id,
            name=name,
            controller_type=controller_type,
            controller_manufacturer=controller_manufacturer,
            position=home_position,
            completion_timeout=timeout,
        )
        await self._wait_for_controller_ready(name, timeout)

        controller_instance = await self._get_controller_instance(name)
        if controller_instance is None:
            raise ControllerNotFound(controller=name)

        return self._create_controller(controller_instance.controller)

    async def ensure_virtual_robot_controller(
        self,
        name: str,
        controller_type: api.models.VirtualControllerTypes,
        controller_manufacturer: api.models.Manufacturer,
    ) -> Controller:
        """
        Ensure a virtual robot controller with the given name exists.
        If it does not exist, create it.
        """
        controller_instance = await self._get_controller_instance(name)
        if controller_instance:
            return self._create_controller(controller_instance.controller)

        return await self.add_virtual_robot_controller(
            name, controller_type, controller_manufacturer
        )

    async def controllers(self) -> list[Controller]:
        """
        List all controllers associated with this cell.
        """
        instances = await self._get_controller_instances()
        return [self._create_controller(ci.controller) for ci in instances]

    async def controller(self, name: str) -> Controller:
        """
        Retrieve a specific controller by name.

        Raises:
            ControllerNotFound: If no controller with the specified name exists.
        """
        controller_instance = await self._get_controller_instance(name)
        if not controller_instance:
            raise ControllerNotFound(controller=name)

        return self._create_controller(controller_instance.controller)

    async def delete_robot_controller(self, name: str, timeout: int = 25):
        """
        Delete a robot controller from the cell.
        """
        await self._api_gateway.delete_robot_controller(
            cell=self._cell_id, controller=name, completion_timeout=timeout
        )

    async def get_robot_cell(self) -> RobotCell:
        """
        Return a RobotCell object containing all known controllers.
        """
        controllers = await self.controllers()
        return RobotCell(timer=None, **{controller.id: controller for controller in controllers})
