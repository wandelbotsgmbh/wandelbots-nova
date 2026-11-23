"""What every policy Rerun entry is logged against, and the drawing helpers they share."""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import TYPE_CHECKING, Any

from novapolicy.rerun.constants import MIN_LINE_STEPS, TRAIL_WIDTH_UI
from novapolicy.rerun.kinematics import joint_tcp_position, to_robot_frame
from novapolicy.rerun.logtime import elapsed_since
import rerun as rr

if TYPE_CHECKING:
    from nova_rerun_bridge.dh_robot import DHRobot
    import numpy as np

    from nova.types import RobotState
    from rerun import RecordingStream

_POINT_RADIUS_UI = 4.0


@dataclass(frozen=True)
class RerunContext:
    """The recording and robot models of one policy run.

    The dicts are shared with the logger that built them, which keeps
    ``pose_frames`` up to date as the chains move.
    """

    start_time: float
    recording: RecordingStream | None
    dh_robots: dict[str, DHRobot] = field(default_factory=dict)
    tcp_offsets: dict[str, np.ndarray] = field(default_factory=dict)
    """Motion group id -> 4x4 flange->TCP transform, where it is known."""
    pose_frames: dict[str, np.ndarray] = field(default_factory=dict)
    """Motion group id -> 4x4 taking a reported pose into the robot's frame."""
    visualizers: dict[str, Any] = field(default_factory=dict)
    max_trail_points: int = 500

    def set_time(self, step: int | None = None, *, elapsed: float | None = None) -> None:
        """Stamp the entries that follow at ``elapsed`` (default: now) and at ``step``."""
        if elapsed is None:
            elapsed = elapsed_since(self.start_time)
        rr.set_time("policy_time", duration=elapsed, recording=self.recording)
        if step is not None:
            rr.set_time("policy_step", sequence=step, recording=self.recording)

    def joint_tcp(self, mg_id: str, joints: list[float]) -> list[float] | None:
        """TCP position for a joint target, or None without a kinematic model."""
        dh_robot = self.dh_robots.get(mg_id)
        if dh_robot is None:
            return None
        return joint_tcp_position(dh_robot, joints, self.tcp_offsets.get(mg_id))

    def measured_tcp(self, mg_id: str, state: RobotState) -> list[float] | None:
        """TCP position of a state in the robot's frame.

        Prefers the pose the robot reports, which honours the active TCP offset,
        and falls back to the flange from DH forward kinematics.
        """
        pose = getattr(state, "pose", None)
        if pose is not None:
            return to_robot_frame(list(pose.position), self.pose_frames.get(mg_id))
        dh_robot = self.dh_robots.get(mg_id)
        if dh_robot is None:
            return None
        return dh_robot.calculate_joint_positions(list(state.joints))[-1]


def log_trail(
    ctx: RerunContext,
    path: str,
    trail: list[list[float]],
    position: list[float],
    color: tuple[int, int, int],
) -> bool:
    """Add ``position`` to ``trail``, keep its newest points, and draw it at ``path``.

    Returns whether the trail had enough points to be drawn as a line.
    """
    trail.append(position)
    del trail[: -ctx.max_trail_points]
    if len(trail) < MIN_LINE_STEPS:
        return False
    rr.log(
        path,
        rr.LineStrips3D(
            [trail], colors=[color], radii=rr.components.Radius.ui_points(TRAIL_WIDTH_UI)
        ),
        recording=ctx.recording,
    )
    return True


def log_points(
    ctx: RerunContext,
    path: str,
    positions: list[list[float]],
    color: tuple[int, int, int],
) -> None:
    """Draw ``positions`` as markers of one color."""
    rr.log(
        path,
        rr.Points3D(
            positions, colors=[color], radii=rr.components.Radius.ui_points(_POINT_RADIUS_UI)
        ),
        recording=ctx.recording,
    )
