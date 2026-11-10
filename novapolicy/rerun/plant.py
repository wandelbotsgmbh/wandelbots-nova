"""Rerun entities for a :class:`novapolicy.plant.VirtualPlant` in the loop.

Three things move when a plant is in the loop, and each gets one line:

* the **robot** (green): its live state, streamed by the state streamer into
  ``joints/``, the mesh and ``tcp_trail``;
* the **plant** (blue): what the policy observed and what the robot is asked to
  follow, so it is the ``joint_target`` of the tracking plots and gets a
  ``plant_tcp`` point and trail here;
* the policy's **setpoint** (amber): the joint values it returned, as
  ``setpoint/j<i>`` scalars and a ``setpoint_tcp`` point and trail. It is what
  the servo is pulled toward, not where the arm goes.
"""

from __future__ import annotations

from typing import TYPE_CHECKING, Any

from novapolicy.rerun.constants import (
    MIN_LINE_STEPS,
    TCP_TARGET_TRAIL_COLOR,
    TRAIL_WIDTH_UI,
)
from novapolicy.rerun.kinematics import joint_tcp_position
from novapolicy.rerun.logtime import elapsed_since
import rerun as rr

if TYPE_CHECKING:
    from nova_rerun_bridge.dh_robot import DHRobot
    import numpy as np

    from nova.types import RobotState
    from rerun import RecordingStream

PLANT_TRAIL_COLOR = (120, 160, 255)  # blue — the plant the policy observes


def _set_time(step: int, start_time: float, recording: RecordingStream | None) -> None:
    # The pinned produce instant when written through the logger's sink, so a
    # backlog does not shift the setpoint against the tracking plots.
    rr.set_time("policy_time", duration=elapsed_since(start_time), recording=recording)
    rr.set_time("policy_step", sequence=step, recording=recording)


def _log_trail(
    entity: str,
    position: list[float],
    trail: list[list[float]],
    *,
    color: tuple[int, int, int],
    max_trail_points: int,
    recording: RecordingStream | None,
) -> None:
    trail.append(position)
    if len(trail) > max_trail_points:
        trail.pop(0)
    if len(trail) >= MIN_LINE_STEPS:
        rr.log(
            f"{entity}_trail",
            rr.LineStrips3D(
                [trail],
                colors=[color],
                radii=rr.components.Radius.ui_points(TRAIL_WIDTH_UI),
            ),
            recording=recording,
        )
    rr.log(
        entity,
        rr.Points3D([position], colors=[color], radii=rr.components.Radius.ui_points(4.0)),
        recording=recording,
    )


def log_plant_observation(
    states: dict[str, RobotState],
    step: int,
    *,
    start_time: float,
    dh_robots: dict[str, Any],
    tcp_offsets: dict[str, Any],
    tcp_trail: dict[str, list[list[float]]],
    max_trail_points: int,
    recording: RecordingStream | None,
) -> None:
    """Draw where the plant's TCP is. Its joints appear as the tracking target."""
    _set_time(step, start_time, recording)
    for mg_id, state in states.items():
        dh_robot = dh_robots.get(mg_id)
        if dh_robot is None:
            continue
        _log_trail(
            f"policy/{mg_id}/plant_tcp",
            joint_tcp_position(dh_robot, list(state.joints), tcp_offsets.get(mg_id)),
            tcp_trail.setdefault(mg_id, []),
            color=PLANT_TRAIL_COLOR,
            max_trail_points=max_trail_points,
            recording=recording,
        )


def log_plant_setpoint(
    mg_id: str,
    setpoint: list[float],
    step: int,
    *,
    start_time: float,
    dh_robot: DHRobot | None,
    tcp_offset: np.ndarray | None,
    trail: list[list[float]],
    max_trail_points: int,
    recording: RecordingStream | None,
) -> None:
    """Log the joint values the policy returned and their TCP."""
    _set_time(step, start_time, recording)
    for i, value in enumerate(setpoint):
        rr.log(f"policy/{mg_id}/setpoint/j{i}", rr.Scalars(value), recording=recording)
    if dh_robot is None:
        return
    _log_trail(
        f"policy/{mg_id}/setpoint_tcp",
        joint_tcp_position(dh_robot, setpoint, tcp_offset),
        trail,
        color=TCP_TARGET_TRAIL_COLOR,
        max_trail_points=max_trail_points,
        recording=recording,
    )
