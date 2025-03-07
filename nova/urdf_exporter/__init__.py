"""
URDF Exporter Package - Simple Implementation

This module provides tools to export robot models to URDF format
using direct DH parameter conversion without external libraries.
"""

from typing import Union

from wandelbots_api_client.models.optimizer_setup import OptimizerSetup

from nova.urdf_exporter.urdf_exporter import (
    URDFExporter,
    create_urdf_exporter_from_optimizer_setup,
    export_urdf_from_optimizer_setup,
)
from nova_rerun_bridge.dh_robot import DHRobot


def export_urdf_from_dh_robot(
    dh_robot: DHRobot, model_name: str = "robot", export_path: str = None
) -> str:
    """
    Export a URDF file from a DHRobot object.

    Args:
        dh_robot: DHRobot object with DH parameters
        model_name: Name for the robot model
        export_path: Directory to export files to

    Returns:
        Path to the exported URDF file
    """
    exporter = URDFExporter(robot=dh_robot, model_name=model_name, export_path=export_path)
    urdf_path = exporter.export_urdf()
    return urdf_path


__all__ = [
    "URDFExporter",
    "export_urdf_from_optimizer_setup",
    "export_urdf_from_dh_robot",
    "create_urdf_exporter_from_optimizer_setup",
]
