"""Per-step observation logging: robot mesh, TCP trail, joint scalars."""

from __future__ import annotations

import time
from typing import TYPE_CHECKING, Any

from policy.rerun.constants import _MIN_LINE_STEPS, _TCP_TRAIL_COLOR, _TRAIL_WIDTH_UI
import rerun as rr

if TYPE_CHECKING:
    from nova.types import RobotState


def log_observation(
    states: dict[str, RobotState],
    step: int,
    *,
    start_time: float,
    dh_robots: dict[str, Any],
    visualizers: dict[str, Any],
    tcp_trail: dict[str, list[list[float]]],
    max_trail_points: int,
) -> None:
    """Log robot state: update 3D mesh positions, joint scalars, TCP trail."""
    elapsed = time.monotonic() - start_time
    rr.set_time("policy_time", duration=elapsed)
    rr.set_time("policy_step", sequence=step)

    for mg_id, state in states.items():
        if not hasattr(state, "joints"):
            continue
        joints = list(state.joints)

        # Joint timeseries
        for i, j in enumerate(joints):
            rr.log(f"policy/{mg_id}/joints/j{i}", rr.Scalars(j))

        # Update 3D robot mesh
        visualizer = visualizers.get(mg_id)
        if visualizer is not None:
            visualizer.log_robot_geometry(joint_position=joints)

        # TCP trail (actual path in green, screen-space width)
        dh_robot = dh_robots.get(mg_id)
        if dh_robot is not None:
            positions = dh_robot.calculate_joint_positions(joints)
            tcp_pos = positions[-1]
            trail = tcp_trail[mg_id]
            trail.append(tcp_pos)
            if len(trail) > max_trail_points:
                trail.pop(0)
            if len(trail) >= _MIN_LINE_STEPS:
                rr.log(
                    f"policy/{mg_id}/tcp_trail",
                    rr.LineStrips3D(
                        [trail],
                        colors=[_TCP_TRAIL_COLOR],
                        radii=rr.components.Radius.ui_points(_TRAIL_WIDTH_UI),
                    ),
                )
            rr.log(
                f"policy/{mg_id}/tcp",
                rr.Points3D(
                    [tcp_pos],
                    colors=[_TCP_TRAIL_COLOR],
                    radii=rr.components.Radius.ui_points(4.0),
                ),
            )
