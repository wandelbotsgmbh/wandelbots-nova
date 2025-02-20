from __future__ import annotations

import asyncio

from decouple import config
from loguru import logger

from nova import api
from nova.core.controller import Controller
from nova.core.exceptions import ControllerNotFound
from nova.core.gateway import ApiGateway
from nova.core.logging_setup import configure_logging
from nova.core.robot_cell import RobotCell

LOG_LEVEL = config("LOG_LEVEL", default="INFO")
CELL_NAME = config("CELL_NAME", default="cell")


# TODO: could also extend NovaDevice
class Nova:
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
        log_level: str = LOG_LEVEL,
    ):
        configure_logging(log_level)
        self._api_client = ApiGateway(
            host=host,
            access_token=access_token,
            username=username,
            password=password,
            version=version,
            verify_ssl=verify_ssl,
        )

    def cell(self, cell_id: str = CELL_NAME) -> Cell:
        return Cell(self._api_client, cell_id)

    async def close(self):
        return await self._api_client.close()

    async def __aenter__(self):
        return self

    async def __aexit__(self, exc_type, exc_val, exc_tb):
        await self.close()


class Cell:
    def __init__(self, api_gateway: ApiGateway, cell_id: str):
        self._api_gateway = api_gateway
        self._cell_id = cell_id

    @property
    def cell_id(self) -> str:
        return self._cell_id

    async def _get_controller_instances(self) -> list[api.models.ControllerInstance]:
        response = await self._api_gateway.controller_api.list_controllers(cell=self._cell_id)
        return response.instances

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

    async def _get_controller_instance(self, name: str) -> api.models.ControllerInstance | None:
        """Get the controller instance

        Args:
            name (str): The name of the controller

        Return: models.ControllerInstance: The controller instance
        """
        controllers = await self._get_controller_instances()
        controller = next((c for c in controllers if c.controller == name), None)
        return controller

    async def _wait_for_controller_ready(self, name: str, timeout: int):
        """Waits for the controller to be available

        Args:
            name (str): The name of the controller
            timeout (int): The time to wait for the controller to be available in seconds
        """
        iteration = 0
        controller = await self._get_controller_instance(name)

        while iteration < timeout:
            if controller is not None:
                if (
                    controller.error_details == "Controller not initialized or disposed"
                    or controller.error_details == "Initializing controller connection."
                ):
                    await self._api_gateway.controller_api.get_current_robot_controller_state(
                        cell=self._cell_id, controller=controller.host
                    )
                elif controller.has_error:
                    # As long has an error its being initialized
                    logger.error(controller.error_details)
                else:
                    # Controller is ready
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
        # TODO: This is not optimal yet and will be automatically resolved in the new v2 API. The default configuration
        #   is only valid for UR.
        #   See: https://code.wabo.run/robotics/wbr/-/blob/develop/wbr/src/service/internal/config/HomeJoints.h
        position: str = "[1.170,-1.6585,1.4051,-1.5707,-1.5709,1.170,0]",
    ) -> Controller:
        """Add a virtual robot controller to the cell

        Args:
            name (str): The name of the controller
            controller_type (models.VirtualControllerTypes): The type of the controller
            controller_manufacturer (models.Manufacturer): The manufacturer of the controller
            timeout (int): The time to wait for the controller to be available in seconds
            position (str): The initial position of the robot

        """
        await self._api_gateway.controller_api.add_robot_controller(
            cell=self._cell_id,
            robot_controller=api.models.RobotController(
                name=name,
                configuration=api.models.RobotControllerConfiguration(
                    api.models.VirtualController(
                        type=controller_type,
                        manufacturer=controller_manufacturer,
                        position=position,
                    )
                ),
            ),
            completion_timeout=timeout,
        )
        # Technically not needed because of the completion_timeout but it handles edge cases right now
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
    ) -> "Controller":
        controller_instance = await self._get_controller_instance(name)
        if controller_instance:
            return self._create_controller(controller_instance.controller)
        return await self.add_virtual_robot_controller(
            name, controller_type, controller_manufacturer
        )

    async def controllers(self) -> list["Controller"]:
        controller_instances = await self._get_controller_instances()
        return [
            self._create_controller(controller_instance.controller)
            for controller_instance in controller_instances
        ]

    async def controller(self, name: str) -> "Controller":
        controller_instance = await self._get_controller_instance(name)

        if controller_instance is None:
            raise ControllerNotFound(controller=name)

        return self._create_controller(controller_instance.controller)

    async def delete_robot_controller(self, name: str, timeout: int = 25):
        await self._api_gateway.controller_api.delete_robot_controller(
            cell=self._cell_id, controller=name, completion_timeout=timeout
        )

    async def get_robot_cell(self) -> RobotCell:
        """Return the configured robot cell"""
        controllers = await self.controllers()
        return RobotCell(timer=None, **{controller.id: controller for controller in controllers})
