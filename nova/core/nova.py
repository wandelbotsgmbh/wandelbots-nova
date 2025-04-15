from __future__ import annotations

from math import pi

from decouple import config
from wandelbots_api_client.models.abb_controller import AbbController
from wandelbots_api_client.models.fanuc_controller import FanucController
from wandelbots_api_client.models.kuka_controller import KukaController
from wandelbots_api_client.models.universalrobots_controller import UniversalrobotsController
from wandelbots_api_client.models.yaskawa_controller import YaskawaController

from nova import api
from nova.core.controller import Controller
from nova.core.exceptions import ControllerNotFound
from nova.core.gateway import ApiGateway
from nova.core.robot_cell import RobotCell

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
        """Returns the cell object with the given ID."""
        return Cell(self._api_client, cell_id)

    async def close(self):
        """Closes the underlying API client session."""
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

    async def add_virtual_robot_controller(
        self,
        name: str,
        controller_type: api.models.VirtualControllerTypes,
        controller_manufacturer: api.models.Manufacturer,
        timeout: int = 25,
        position: str | None = None,
    ) -> Controller:
        await self.add_controller(
            robot_controller=virtual_controller(
                name=name,
                type=controller_type,
                manufacturer=controller_manufacturer,
                position=position,
            ),
            add_timeout=timeout,
        )

    async def ensure_virtual_robot_controller(
        self,
        name: str,
        controller_type: api.models.VirtualControllerTypes,
        controller_manufacturer: api.models.Manufacturer,
        timeout: int = 25,
    ) -> Controller:
        return await self.ensure_controller(
            robot_controller=virtual_controller(
                name=name, type=controller_type, manufacturer=controller_manufacturer
            ),
            timeout=timeout,
        )

    async def add_controller(
        self,
        robot_controller: api.models.RobotController,
        add_timeout: int = 25,
        wait_for_ready_timeout: int = 25,
    ) -> Controller:
        """
        Add a robot controller to the cell and wait for it to get ready.
        Args:
            robot_controller (api.models.RobotController): The robot controller to add. You can use helper functions from nova to create these configs easily,
                see :func:`nova.abb_controller`, :func:`nova.fanuc_controller`, :func:`nova.kuka_controller`,
                :func:`nova.universal_robots_controller`, :func:`nova.virtual_controller`, :func:`nova.yaskawa_controller`.
            add_timeout (int): The time to wait for the controller to be added (default: 25).
            wait_for_ready_timeout (int): The time to wait for the controller to be ready (default: 25).

        Returns:
            Controller: The added Controller object.
        """
        await self._api_gateway.add_robot_controller(
            cell=self._cell_id, robot_controller=robot_controller, timeout=add_timeout
        )

        await self._api_gateway.wait_for_controller_ready(
            cell=self._cell_id, name=robot_controller.name, timeout=wait_for_ready_timeout
        )

        return self._create_controller(robot_controller.name)

    async def ensure_controller(
        self,
        robot_controller: api.models.RobotController,
        add_timeout: int = 25,
        waitfor_ready_timeout: int = 25,
    ) -> Controller:
        """
        Ensure that a robot controller is added to the cell. If it already exists, it will be returned.
        If it doesn't exist, it will be added and waited for to be ready.
        Args:
            robot_controller (api.models.RobotController): The robot controller to add. You can use helper functions from nova to create these configs easily,
                see :func:`nova.abb_controller`, :func:`nova.fanuc_controller`, :func:`nova.kuka_controller`,
                :func:`nova.universal_robots_controller`, :func:`nova.virtual_controller`, :func:`nova.yaskawa_controller`.
            add_timeout (int): The time to wait for the controller to be added (default: 25).
            wait_for_ready_timeout (int): The time to wait for the controller to be ready (default: 25).

        Returns:
            Controller: The added Controller object.
        """
        controller = await self._api_gateway.get_controller_instance(
            cell=self.cell_id, name=robot_controller.name
        )

        if controller:
            return self._create_controller(controller.controller)
        return await self.add_controller(
            robot_controller, add_timeout=add_timeout, wait_for_ready_timeout=waitfor_ready_timeout
        )

    async def controllers(self) -> list[Controller]:
        """
        List all controllers for this cell.
        Returns:
            list[Controller]: A list of Controller objects associated with this cell.
        """
        instances = await self._api_gateway.list_controllers(cell=self._cell_id)
        return [self._create_controller(ci.controller) for ci in instances]

    async def controller(self, name: str) -> Controller:
        """
        Retrieve a specific controller by name.
        Args:
            name (str): The name of the controller.
        Returns:
            Controller: The Controller object.
        Raises:
            ControllerNotFound: If no controller with the specified name exists.
        """
        controller_instance = await self._api_gateway.get_controller_instance(
            cell=self._cell_id, name=name
        )
        if not controller_instance:
            raise ControllerNotFound(controller=name)
        return self._create_controller(controller_instance.controller)

    async def delete_robot_controller(self, name: str, timeout: int = 25):
        """
        Delete a robot controller from the cell.
        Args:
            name (str): The name of the controller to delete.
            timeout (int): The time to wait for the controller deletion to complete (default: 25).
        """
        await self._api_gateway.delete_robot_controller(
            cell=self._cell_id, controller=name, completion_timeout=timeout
        )

    async def get_robot_cell(self) -> RobotCell:
        """
        Return a RobotCell object containing all known controllers.
        Returns:
            RobotCell: A RobotCell initialized with the available controllers.
        """
        controllers = await self.controllers()
        return RobotCell(timer=None, **{controller.id: controller for controller in controllers})


def abb_controller(
    name: str, controller_ip: str, egm_server_ip: str, egm_server_port: str
) -> api.models.RobotController:
    """
    Create an ABB controller configuration for a pysical robot.
    Args:
        controller_ip (str): The IP address of the ABB robot.
        egm_server_ip (str): The IP address of the EGM server.
        egm_server_port (str): The port of the EGM server.
    """
    return api.models.RobotController(
        name=name,
        configuration=api.models.RobotControllerConfiguration(
            AbbController(
                controller_ip=controller_ip,
                egm_server=api.models.AbbControllerEgmServer(
                    ip=egm_server_ip, port=egm_server_port
                ),
            )
        ),
    )


def universal_robots_controller(name: str, controller_ip: str) -> api.models.RobotController:
    """
    Create a Universal Robots controller configuration.
    Args:
        controller_ip (str): The IP address of the Universal Robots robot.
    """
    return api.models.RobotController(
        name=name,
        configuration=api.models.RobotControllerConfiguration(
            UniversalrobotsController(controller_ip=controller_ip)
        ),
    )


def kuka_controller(
    name: str, controller_ip: str, controller_port: str, rsi_server_ip: str, rsi_server_port: str
) -> api.models.RobotController:
    """
    Create a KUKA controller configuration for a physical robot.
    Args:
        controller_ip (str): The IP address of the KUKA robot.
        controller_port (str): The port of the KUKA robot.
        rsi_server_ip (str): The IP address of the RSI server.
        rsi_server_port (str): The port of the RSI server.
    """

    return api.models.RobotController(
        name=name,
        configuration=api.models.RobotControllerConfiguration(
            KukaController(
                controller_ip=controller_ip,
                controller_port=controller_port,
                rsi_server=api.models.KukaControllerRsiServer(
                    ip=rsi_server_ip, port=rsi_server_port
                ),
            )
        ),
    )


def fanuc_controller(name: str, controller_ip: str) -> api.models.RobotController:
    """
    Create a FANUC controller configuration.
    Args:
        controller_ip (str): The IP address of the FANUC robot.
    """
    return api.models.RobotController(
        name=name,
        configuration=api.models.RobotControllerConfiguration(
            FanucController(controller_ip=controller_ip)
        ),
    )


def yaskawa_controller(name: str, controller_ip: str) -> api.models.RobotController:
    """
    Create a Yaskawa controller configuration.
    Args:
        controller_ip (str): The IP address of the Yaskawa robot.
    """
    return api.models.RobotController(
        name=name,
        configuration=api.models.RobotControllerConfiguration(
            YaskawaController(controller_ip=controller_ip)
        ),
    )


def virtual_controller(
    name: str,
    manufacturer: api.models.Manufacturer,
    type: api.models.VirtualControllerTypes | None = None,
    json: str | None = None,
    position: list[float] | None = None,
) -> api.models.RobotController:
    """
    Create a virtual controller configuration.
    Args:
        name (str): The name of the controller.
        manufacturer (api.models.Manufacturer): The manufacturer of the robot.
        type (api.models.VirtualControllerTypes | None): One of the available virtual controller types for this manufacturer.
        position: (str | None): Initial joint position of the first motion group from the virtual robot controller.
        json (str | None): Additional data to save on controller.
    """
    if position is None:
        position = MANUFACTURER_HOME_POSITIONS.get(manufacturer, [0.0] * 7)

    return api.models.RobotController(
        name=name,
        configuration=api.models.RobotControllerConfiguration(
            api.models.VirtualController(
                manufacturer=manufacturer, type=type, json=json, position=str(position)
            )
        ),
    )
