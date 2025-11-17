import json
from math import pi

from nova import api

MANUFACTURER_HOME_POSITIONS = {
    api.models.Manufacturer.ABB: [0.0, 0.0, 0.0, 0.0, pi / 2, pi, 0.0],
    api.models.Manufacturer.FANUC: [0.0, 0.0, 0.0, 0.0, -pi / 2, 0.0, 0.0],
    api.models.Manufacturer.YASKAWA: [0.0, 0.0, 0.0, 0.0, -pi / 2, 0.0, 0.0],
    api.models.Manufacturer.KUKA: [0.0, -pi / 2, pi / 2, 0.0, pi / 2, pi, 0.0],
    api.models.Manufacturer.UNIVERSALROBOTS: [0.0, -pi / 2, -pi / 2, -pi / 2, pi / 2, -pi / 2, 0.0],
}


def _build_controller(
    name: str,
    configuration: api.models.AbbController
    | api.models.FanucController
    | api.models.KukaController
    | api.models.UniversalrobotsController
    | api.models.VirtualController
    | api.models.YaskawaController,
) -> api.models.RobotController:
    """Helper function to wrap a controller configuration in a RobotController object."""
    return api.models.RobotController(name=name, configuration=configuration)


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
    abb_config = api.models.AbbController(
        controller_ip=controller_ip,
        egm_server=api.models.EgmServer(ip=egm_server_ip, port=egm_server_port),
        controller_port=controller_port,
    )
    return _build_controller(name=name, configuration=abb_config)


def universal_robots_controller(name: str, controller_ip: str) -> api.models.RobotController:
    """
    Create a Universal Robots controller configuration for a physical robot.
    Args:
        name (str): The name of the controller.
        controller_ip (str): The IP address of the Universal Robots robot.
    """
    universal_config = api.models.UniversalrobotsController(controller_ip=controller_ip)
    return _build_controller(name=name, configuration=universal_config)


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
    kuka_config = api.models.KukaController(
        controller_ip=controller_ip,
        controller_port=controller_port,
        rsi_server=api.models.RsiServer(ip=rsi_server_ip, port=rsi_server_port),
    )
    return _build_controller(name=name, configuration=kuka_config)


def fanuc_controller(name: str, controller_ip: str) -> api.models.RobotController:
    """
    Create a FANUC controller configuration for a physical robot.
    Args:
        name (str): The name of the controller.
        controller_ip (str): The IP address of the FANUC robot.
    """
    fanuc_config = api.models.FanucController(controller_ip=controller_ip)
    return _build_controller(name=name, configuration=fanuc_config)


def yaskawa_controller(name: str, controller_ip: str) -> api.models.RobotController:
    """
    Create a Yaskawa controller configuration for a physical robot.
    Args:
        name (str): The name of the controller.
        controller_ip (str): The IP address of the Yaskawa robot.
    """
    yaskawa_config = api.models.YaskawaController(controller_ip=controller_ip)
    return _build_controller(name=name, configuration=yaskawa_config)


def virtual_controller(
    name: str,
    manufacturer: api.models.Manufacturer,
    type: api.models.VirtualControllerTypes | None = None,
    controller_config_json: str | None = None,
    position: list[float] | str | None = None,
) -> api.models.RobotController:
    """
    Create a virtual controller configuration.
    Args:
        name (str): The name of the controller.
        manufacturer (api.models.Manufacturer): The manufacturer of the robot.
        type (api.models.VirtualControllerTypes | None): One of the available virtual controller types for this manufacturer.
        position: (list[float] | None): Initial joint position of the first motion group from the virtual robot controller.
        controller_config_json (str | None): Complete JSON configuration of the virtual robot controller.
    """
    # TODO remove if the underlying API has a decent error message
    if isinstance(position, list) and len(position) != 7:
        raise ValueError("Position list must contain exactly 7 elements.")

    if position is None:
        position = MANUFACTURER_HOME_POSITIONS.get(manufacturer, [0.0] * 7)

    virtual_config = api.models.VirtualController(
        manufacturer=manufacturer,
        type=type,
        json_=controller_config_json,
        initial_joint_position=json.dumps(position),
    )
    return _build_controller(name=name, configuration=virtual_config)
