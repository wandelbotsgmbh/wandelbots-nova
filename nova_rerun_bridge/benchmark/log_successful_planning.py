from dataclasses import dataclass, field
from pathlib import Path
from typing import Any

import numpy as np
import rerun as rr
from scipy.spatial.transform import Rotation

from nova.core.motion_group import MotionGroup
from nova_rerun_bridge import NovaRerunBridge
from nova_rerun_bridge.benchmark.robometrics_helper import convert_position


def _default_colors() -> dict[str, tuple[float, float, float, float]]:
    """Default color configuration for visualization."""
    return {
        "x_axis": (1.0, 0.125, 0.376, 1.0),  # #ff2060 - Red/Pink
        "y_axis": (0.125, 0.875, 0.502, 1.0),  # #20df80 - Green
        "z_axis": (0.125, 0.502, 1.0, 1.0),  # #2080ff - Blue
        "target": (0, 255, 0, 255),  # Green
    }


@dataclass
class VisualizationConfig:
    """Configuration for trajectory visualization."""

    arrow_length: float = 50.0  # mm
    point_radius: float = 5.0
    arrow_radius: float = 2.5
    colors: dict[str, tuple[float, float, float, float]] = field(default_factory=_default_colors)


async def log_successful_planning(
    bridge: NovaRerunBridge,
    strategy: str,
    collision_scene_id: str,
    problem: dict[str, Any],
    key: str,
    iteration: int,
    trajectory: Any,
    tcp: str,
    motion_group: MotionGroup,
    vis_config: VisualizationConfig = VisualizationConfig(),
) -> str:
    """Log successful planning attempt with visualizations.

    Args:
        bridge: NovaRerunBridge instance
        collision_scene_id: ID of the collision scene
        problem: Benchmark problem definition
        key: Scene identifier
        iteration: Current iteration number
        trajectory: Planned trajectory
        tcp: Tool Center Point identifier
        motion_group: Motion group instance
        vis_config: Visualization configuration
    """
    result_dir = Path("benchmark_results") / strategy / key
    result_dir.mkdir(parents=True, exist_ok=True)

    # Initialize rerun visualization
    recording_id = f"{strategy}_{key}_{iteration}"

    rr.init(application_id="nova", recording_id=recording_id, spawn=False)
    rr.save(str(result_dir / f"{recording_id}.rrd"))

    await bridge.setup_blueprint()
    await bridge.log_collision_scene(collision_scene_id)

    # Process goal pose
    goal_position = convert_position(problem["goal_pose"]["position_xyz"])
    goal_quat = problem["goal_pose"]["quaternion_wxyz"]

    # Log target position
    _log_target_position(goal_position, vis_config)

    # Log orientation arrows
    _log_orientation_arrows(goal_position, goal_quat, vis_config)

    # Log trajectory
    await bridge.log_trajectory(trajectory, tcp, motion_group)

    return f"{result_dir / f'{recording_id}.rrd'}"


def _log_target_position(position: list[float], config: VisualizationConfig) -> None:
    """Log target position as a 3D point."""
    rr.log(
        "motion/target_orientation",
        rr.Points3D(
            positions=[position], radii=[config.point_radius], colors=[config.colors["target"]]
        ),
    )


def _log_orientation_arrows(
    position: list[float], quaternion: list[float], config: VisualizationConfig
) -> None:
    """Log orientation as colored coordinate arrows."""
    w, x, y, z = quaternion
    rot = Rotation.from_quat([x, y, z, w])

    # Create arrow vectors for each axis
    arrow_vectors = [
        rot.apply([config.arrow_length, 0, 0]),  # X axis
        rot.apply([0, config.arrow_length, 0]),  # Y axis
        rot.apply([0, 0, config.arrow_length]),  # Z axis
    ]

    # Convert colors to numpy array
    coordinate_colors = np.array(
        [config.colors["x_axis"], config.colors["y_axis"], config.colors["z_axis"]]
    )

    rr.log(
        "motion/target_orientation",
        rr.Arrows3D(
            origins=[position] * 3,
            vectors=arrow_vectors,
            colors=coordinate_colors,
            radii=[config.arrow_radius] * 3,
        ),
        static=True,
    )
