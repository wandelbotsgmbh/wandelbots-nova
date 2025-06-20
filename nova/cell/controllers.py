from math import pi

from wandelbots_api_client.models.abb_controller import AbbController
from wandelbots_api_client.models.fanuc_controller import FanucController
from wandelbots_api_client.models.kuka_controller import KukaController
from wandelbots_api_client.models.universalrobots_controller import UniversalrobotsController
from wandelbots_api_client.models.virtual_controller import VirtualController
from wandelbots_api_client.models.yaskawa_controller import YaskawaController

from nova import api

MANUFACTURER_HOME_POSITIONS = {
    api.models.Manufacturer.ABB: [0.0, 0.0, 0.0, 0.0, pi / 2, 0.0, 0.0],
    api.models.Manufacturer.FANUC: [0.0, 0.0, 0.0, 0.0, -pi / 2, 0.0, 0.0],
    api.models.Manufacturer.YASKAWA: [0.0, 0.0, 0.0, 0.0, -pi / 2, 0.0, 0.0],
    api.models.Manufacturer.KUKA: [0.0, -pi / 2, pi / 2, 0.0, pi / 2, 0.0, 0.0],
    api.models.Manufacturer.UNIVERSALROBOTS: [0.0, -pi / 2, -pi / 2, -pi / 2, pi / 2, -pi / 2, 0.0],
}


def _build_controller(
    name: str,
    controller: AbbController
    | FanucController
    | KukaController
    | UniversalrobotsController
    | VirtualController
    | YaskawaController,
) -> api.models.RobotController:
    """Helper function to wrap a controller configuration in a RobotController object."""
    return api.models.RobotController(
        name=name, configuration=api.models.RobotControllerConfiguration(controller)
    )


def abb_controller(
    name: str,
    controller_ip: str,
    egm_server_ip: str,
    egm_server_port: int,
    controller_port: int = 80,
) -> api.models.RobotController:
    """
    Create an ABB controller configuration for a physical robot.
    Args:
        name (str): The name of the controller.
        controller_ip (str): The IP address of the ABB robot.
        egm_server_ip (str): The IP address of the EGM server.
        egm_server_port (str): The port of the EGM server.
        controller_port (int): The port of the ABB controller
    """
    abb_config = AbbController(
        controllerIp=controller_ip,
        egmServer=api.models.AbbControllerEgmServer(ip=egm_server_ip, port=egm_server_port),
        controllerPort=controller_port,
    )
    return _build_controller(name=name, controller=abb_config)


def universal_robots_controller(name: str, controller_ip: str) -> api.models.RobotController:
    """
    Create a Universal Robots controller configuration for a physical robot.
    Args:
        name (str): The name of the controller.
        controller_ip (str): The IP address of the Universal Robots robot.
    """
    universal_config = UniversalrobotsController(controllerIp=controller_ip)
    return _build_controller(name=name, controller=universal_config)


def kuka_controller(
    name: str, controller_ip: str, controller_port: int, rsi_server_ip: str, rsi_server_port: int
) -> api.models.RobotController:
    """
    Create a KUKA controller configuration for a physical robot.
    Args:
        name (str): The name of the controller.
        controller_ip (str): The IP address of the KUKA robot.
        controller_port (str): The port of the KUKA robot.
        rsi_server_ip (str): The IP address of the RSI server.
        rsi_server_port (str): The port of the RSI server.
    """
    kuka_config = KukaController(
        controllerIp=controller_ip,
        controllerPort=controller_port,
        rsiServer=api.models.KukaControllerRsiServer(ip=rsi_server_ip, port=rsi_server_port),
    )
    return _build_controller(name=name, controller=kuka_config)


def fanuc_controller(name: str, controller_ip: str) -> api.models.RobotController:
    """
    Create a FANUC controller configuration for a physical robot.
    Args:
        name (str): The name of the controller.
        controller_ip (str): The IP address of the FANUC robot.
    """
    fanuc_config = FanucController(controllerIp=controller_ip)
    return _build_controller(name=name, controller=fanuc_config)


def yaskawa_controller(name: str, controller_ip: str) -> api.models.RobotController:
    """
    Create a Yaskawa controller configuration for a physical robot.
    Args:
        name (str): The name of the controller.
        controller_ip (str): The IP address of the Yaskawa robot.
    """
    yaskawa_config = YaskawaController(controllerIp=controller_ip)
    return _build_controller(name=name, controller=yaskawa_config)


def virtual_controller(
    name: str,
    manufacturer: api.models.Manufacturer,
    type: api.models.VirtualControllerTypes | None = None,
    json: str | None = None,
    position: list[float] | str | None = None,
) -> api.models.RobotController:
    """
    Create a virtual controller configuration.
    Args:
        name (str): The name of the controller.
        manufacturer (api.models.Manufacturer): The manufacturer of the robot.
        type (api.models.VirtualControllerTypes | None): One of the available virtual controller types for this manufacturer.
        position: (list[float] | None): Initial joint position of the first motion group from the virtual robot controller.
        json (str | None): Additional data to save on controller.
    """
    if position is None:
        position = str(MANUFACTURER_HOME_POSITIONS.get(manufacturer, [0.0] * 7))

    if isinstance(position, list):
        position = str(position)

    virtual_config = api.models.VirtualController(
        manufacturer=manufacturer, type=type, json=json, position=position
    )
    return _build_controller(name=name, controller=virtual_config)
