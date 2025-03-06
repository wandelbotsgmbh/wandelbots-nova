"""
URDF Exporter Package

This module provides tools to export robot models to URDF (Unified Robot Description Format)
and SRDF (Semantic Robot Description Format) for use in simulation, visualization, and motion planning.

Main functions:
- export_urdf_from_optimizer_setup: Export a URDF from an OptimizerSetup object
- export_urdf_from_dh_robot: Export a URDF from a DHRobot object

Example usage:
    from nova.urdf_exporter import export_urdf_from_optimizer_setup

    urdf_path, srdf_path = export_urdf_from_optimizer_setup(
        optimizer_setup=optimizer_setup,
        model_name="ur10e",
        export_path="/path/to/output"
    )
"""

import json
import os
from typing import Dict, List, Optional, Tuple, Union

from wandelbots_api_client.models.optimizer_setup import OptimizerSetup

from nova.urdf_exporter.urdf_exporter import (
    URDFExporter,
    create_urdf_exporter_from_optimizer_setup,
    export_urdf_from_optimizer_setup,
)
from nova_rerun_bridge.dh_robot import DHRobot


def export_urdf_from_dh_robot(
    dh_robot: DHRobot,
    model_name: str = "robot",
    export_path: str = None,
    mesh_model: str = None,
    create_ros_package: bool = False,  # Keeping parameter but default to False
    export_srdf: bool = True,
    robot_geometries: list = None,
    tcp_geometries: list = None,
) -> Union[str, tuple[str, str]]:
    """
    Export a URDF file from a DHRobot object.

    Args:
        dh_robot: DHRobot object with DH parameters
        model_name: Name for the robot model
        export_path: Directory to export files to
        mesh_model: Path to a mesh model to include
        create_ros_package: Whether to create a ROS package structure (defaults to False)
        export_srdf: Whether to also export an SRDF file
        robot_geometries: Optional list of geometries for robot links
        tcp_geometries: Optional list of geometries for TCP

    Returns:
        Path to the exported URDF file and SRDF file (if requested)
    """
    exporter = URDFExporter(
        robot=dh_robot,
        model_name=model_name,
        export_path=export_path,
        mesh_model=mesh_model,
        robot_model_geometries=robot_geometries,
        tcp_geometries=tcp_geometries,
    )

    urdf_path = exporter.export_urdf(create_package=create_ros_package)

    if export_srdf:
        srdf_path = exporter.export_srdf()
        return urdf_path, srdf_path

    return urdf_path


def export_urdf_from_json(
    json_config: Union[str, dict],
    model_name: str = None,
    export_path: str = None,
    mesh_model: str = None,
    create_ros_package: bool = False,  # Keeping parameter but default to False
    export_srdf: bool = True,
) -> Union[str, tuple[str, str]]:
    """
    Export a URDF file from a JSON configuration (file path or dict).

    Args:
        json_config: Path to JSON file or dictionary containing optimizer config
        model_name: Name for the robot model
        export_path: Directory to export files to
        mesh_model: Path to a mesh model to include
        create_ros_package: Whether to create a ROS package structure (defaults to False)
        export_srdf: Whether to also export an SRDF file

    Returns:
        Path to the exported URDF file and SRDF file (if requested)
    """
    # Load optimizer config from JSON file or dictionary
    if isinstance(json_config, str):
        with open(json_config, "r") as f:
            config_dict = json.load(f)
    else:
        config_dict = json_config

    # Convert to OptimizerSetup
    optimizer_setup = OptimizerSetup.from_dict(config_dict)

    # Export using the optimizer setup
    return export_urdf_from_optimizer_setup(
        optimizer_setup=optimizer_setup,
        model_name=model_name,
        export_path=export_path,
        mesh_model=mesh_model,
        create_ros_package=create_ros_package,
        export_srdf=export_srdf,
    )


__all__ = [
    "URDFExporter",
    "export_urdf_from_optimizer_setup",
    "export_urdf_from_dh_robot",
    "export_urdf_from_json",
    "create_urdf_exporter_from_optimizer_setup",
]
