"""Fit a GR00T policy's action convention from one training episode.

The episode's recorded joints, run through the robot's forward kinematics (DH parameters
and TCP from NOVA), give the TCP path the training robot actually moved. Comparing that
path with the recorded actions tells what the actions mean: which frame, how much of a
commanded step was realized (gain), and how the gripper is driven. Used by
``novapolicy.gr00t.inspect_policy --episode``.
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import TYPE_CHECKING, cast

import numpy as np
from scipy.spatial.transform import Rotation

if TYPE_CHECKING:
    from collections.abc import Iterable, Sequence

    from nova import api

R2_GOOD = 0.6
"""A frame or representation fitting at least this well counts as found."""

_POSE_DIM = 6  # [x, y, z, rx, ry, rz]
_MIN_STEPS = 3  # fewer steps cannot be fitted
_BASE_ROTATIONS_DEG = (0, 90, 180, 270)
_GRIPPER_SETTLE_STEPS = 15


def pose_matrix(position: Iterable[float], rotvec: Iterable[float]) -> np.ndarray:
    """4x4 transform from a position (mm) and a rotation vector (rad)."""
    t = np.eye(4)
    t[:3, :3] = Rotation.from_rotvec(np.asarray(list(rotvec), dtype=float)).as_matrix()
    t[:3, 3] = np.asarray(list(position), dtype=float)
    return t


def _pose_or_identity(
    position: Iterable[float] | None, rotvec: Iterable[float] | None
) -> np.ndarray:
    """``pose_matrix`` with missing parts of an API pose taken as zero."""
    return pose_matrix(position or (0.0, 0.0, 0.0), rotvec or (0.0, 0.0, 0.0))


@dataclass
class Kinematics:
    """Forward kinematics of a NOVA motion group with one of its TCPs (mm, rad)."""

    dh: list[tuple[float, float, float, float, bool]]  # a, alpha, d, theta offset, reversed
    tcp: np.ndarray  # flange -> TCP
    mounting: np.ndarray  # world -> robot base

    @classmethod
    def from_description(
        cls, description: api.models.MotionGroupDescription, tcp_name: str
    ) -> Kinematics:
        dh = [
            (
                p.a or 0.0,
                p.alpha or 0.0,
                p.d or 0.0,
                p.theta or 0.0,
                bool(p.reverse_rotation_direction),
            )
            for p in description.dh_parameters or []
        ]
        tcps = description.tcps or {}
        tcp_pose = tcps[tcp_name].pose
        mount = description.mounting
        mounting = _pose_or_identity(mount.position, mount.orientation) if mount else np.eye(4)
        return cls(dh, _pose_or_identity(tcp_pose.position, tcp_pose.orientation), mounting)

    def fk(self, joints: Sequence[float]) -> np.ndarray:
        """TCP pose (4x4, mm) for the given joint positions (rad)."""
        t = self.mounting.copy()
        for (a, alpha, d, theta, reverse), q in zip(self.dh, joints, strict=True):
            th = theta + (-q if reverse else q)
            ca, sa, ct, st = np.cos(alpha), np.sin(alpha), np.cos(th), np.sin(th)
            t @= np.array([
                [ct, -st * ca, st * sa, a * ct],
                [st, ct * ca, -ct * sa, a * st],
                [0, sa, ca, d],
                [0, 0, 0, 1],
            ])
        return t @ self.tcp


@dataclass
class FrameFit:
    """How well ``realized step ~ gain * frame(action)`` fits one candidate frame."""

    frame: str  # "base", "base rotated 90 deg about z", ..., "tool"
    rot_z_deg: int | None  # base-frame rotation; None for the tool frame
    lag: int  # 0: action k moves k -> k+1; 1: it shows up one step later
    pos_gain: float
    pos_r2: float
    rot_gain: float
    rot_r2: float


def _gain_r2(command: np.ndarray, realized: np.ndarray) -> tuple[float, float]:
    """Least-squares gain of ``realized ~ gain * command`` and its R^2."""
    denom = float((command * command).sum())
    total = float(((realized - realized.mean(axis=0)) ** 2).sum())
    if denom <= 0.0 or total <= 0.0:
        return 0.0, 0.0
    gain = float((command * realized).sum()) / denom
    residual = realized - gain * command
    return gain, 1.0 - float((residual**2).sum()) / total


def _fit_lag(poses: list[np.ndarray], actions: np.ndarray, lag: int) -> list[FrameFit]:
    """All candidate frames for one lag between a command and its realized step."""
    n = len(poses) - 1 - lag
    positions = np.array([p[:3, 3] for p in poses]) / 1000.0
    rotations = Rotation.from_matrix(np.array([p[:3, :3] for p in poses]))
    cmd_p, cmd_r = actions[:n, :3], actions[:n, 3:_POSE_DIM]
    d_pos = positions[1 + lag : 1 + lag + n] - positions[lag : lag + n]
    r0, r1 = rotations[lag : lag + n], rotations[1 + lag : 1 + lag + n]
    fits = []
    for deg in _BASE_ROTATIONS_DEG:
        rz = Rotation.from_euler("z", deg, degrees=True)
        realized_rot = cast("Rotation", r1 * r0.inv()).as_rotvec()
        name = "base" if deg == 0 else f"base rotated {deg} deg about z"
        fits.append(
            FrameFit(
                name, deg, lag, *_gain_r2(rz.apply(cmd_p), d_pos),
                *_gain_r2(rz.apply(cmd_r), realized_rot),
            )
        )  # fmt: skip
    tool_cmd = np.array([r.apply(c) for r, c in zip(r0, cmd_p, strict=True)])
    tool_rot = cast("Rotation", r0.inv() * r1).as_rotvec()
    fits.append(FrameFit("tool", None, lag, *_gain_r2(tool_cmd, d_pos), *_gain_r2(cmd_r, tool_rot)))
    return fits


def fit_delta_frames(poses: list[np.ndarray], actions: np.ndarray) -> list[FrameFit]:
    """Fit ``actions[:, :6]`` (xyz in m + rotation vector) as per-step deltas.

    Candidates: the robot base frame rotated by 0/90/180/270 degrees about z, and the tool
    frame of the pose each step starts from, each with a lag of 0 or 1 steps. Best first.
    """
    fits = [
        fit
        for lag in (0, 1)
        if len(poses) - 1 - lag >= _MIN_STEPS
        for fit in _fit_lag(poses, actions, lag)
    ]
    return sorted(fits, key=lambda f: f.pos_r2, reverse=True)


def absolute_r2(poses: list[np.ndarray], actions: np.ndarray) -> float:
    """R^2 of ``actions[:, :3]`` as an absolute target position (any scale and offset)."""
    target = np.array([p[:3, 3] for p in poses]).ravel()
    design = np.column_stack([actions[:, :3].ravel(), np.ones(target.size)])
    coef, *_ = np.linalg.lstsq(design, target, rcond=None)
    total = float(((target - target.mean()) ** 2).sum())
    if total <= 0.0:
        return 0.0
    return 1.0 - float(((target - design @ coef) ** 2).sum()) / total


@dataclass
class GripperFit:
    close_when_negative: bool  # True: an action < 0 closes
    open_state: float  # state at the episode start, before the first command
    holding_state: float  # settled state after the close command
    close_frame: int
    open_frame: int | None


def fit_gripper(state: np.ndarray, action: np.ndarray) -> GripperFit | None:
    """Gripper polarity and values from the first switch of a binary gripper action.

    Episodes start with the jaws open; the first switch closes them, so the jaws then
    move away from their starting value.
    """
    sign = np.sign(action)
    switches = [i for i in range(1, len(sign)) if sign[i] not in {0, sign[i - 1]}]
    if not switches:
        return None
    first = switches[0]
    after = state[first + 1 : first + 1 + _GRIPPER_SETTLE_STEPS]
    if not len(after):
        return None
    open_state = float(state[0])
    moved_away = abs(after[-1] - open_state) > abs(state[first] - open_state)
    end = switches[1] if len(switches) > 1 else len(state)
    holding = float(np.median(state[min(first + _GRIPPER_SETTLE_STEPS, end - 1) : end]))
    return GripperFit(
        close_when_negative=(sign[first] < 0) == moved_away,
        open_state=open_state,
        holding_state=holding,
        close_frame=first,
        open_frame=switches[1] if len(switches) > 1 else None,
    )


@dataclass
class PathSummary:
    """Envelope of one or more episodes' TCP paths."""

    start_joints: list[float]  # of the first episode
    lo: np.ndarray  # xyz min (mm)
    hi: np.ndarray  # xyz max (mm)
    speed_median: float  # mm/s
    speed_max: float
    episodes: int = 1


def path_summary(poses: list[np.ndarray], joints: np.ndarray, fps: float) -> PathSummary:
    """Start pose, TCP bounds and speeds of an episode."""
    positions = np.array([p[:3, 3] for p in poses])
    speeds = np.linalg.norm(np.diff(positions, axis=0), axis=1) * fps
    return PathSummary(
        start_joints=[float(q) for q in joints[0]],
        lo=positions.min(axis=0),
        hi=positions.max(axis=0),
        speed_median=float(np.median(speeds)),
        speed_max=float(speeds.max()),
    )


def merge_summaries(summaries: list[PathSummary]) -> PathSummary:
    """One envelope over several episodes."""
    first = summaries[0]
    return PathSummary(
        start_joints=first.start_joints,
        lo=np.min([s.lo for s in summaries], axis=0),
        hi=np.max([s.hi for s in summaries], axis=0),
        speed_median=float(np.median([s.speed_median for s in summaries])),
        speed_max=max(s.speed_max for s in summaries),
        episodes=sum(s.episodes for s in summaries),
    )
