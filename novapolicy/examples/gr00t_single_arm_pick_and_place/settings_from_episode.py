"""Read the schema settings a GR00T policy needs from one of its training episodes.

The episode's recorded joints, run through the robot's forward kinematics (DH parameters
and TCP from NOVA), give the TCP path the training robot actually moved. Comparing that
path with the recorded actions shows which frame the deltas are in, how much of each
commanded step the training controller realized (the gains, by least squares), whether
the actions are deltas or absolute targets, and how the gripper is driven. The output maps
onto the ``Action.tcp(mode="relative", ...)`` options, the ``Scale`` operators, the gripper
values and the ``TcpLimits`` in ``gr00t_ur10e.py``. Optional: when the training config
states these, read them there.

    NOVA_API=http://<nova> uv run python settings_from_episode.py --episode episode_000075.parquet \
        [--controller ur10e-calibration --tcp umi_corrected]

Columns default to this policy's dataset: ``observation.state`` = [6 joints (rad), gripper],
``action`` = [6 eef deltas (m, rotation vector), gripper].
"""

from __future__ import annotations

import argparse
import asyncio
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


def _load(path: str) -> tuple[np.ndarray, np.ndarray, float]:
    import pyarrow.parquet as pq

    table = pq.read_table(path)
    state = np.array(table.column("observation.state").to_pylist(), dtype=float)
    action = np.array(table.column("action").to_pylist(), dtype=float)
    stamps = np.array(table.column("timestamp").to_pylist(), dtype=float)
    fps = 1.0 / float(np.median(np.diff(stamps))) if len(stamps) > 1 else 0.0
    return state, action, fps


async def _main(args: argparse.Namespace) -> None:
    import nova

    state, action, fps = _load(args.episode)
    async with nova.Nova() as n:
        mg = (await n.cell().controller(args.controller))[0]
        kinematics = Kinematics.from_description(await mg.get_description(), args.tcp)
    joints = state[:, :6]
    poses = [kinematics.fk(q) for q in joints]
    tcp = action[:, :6]
    print(f"{args.episode}: {len(state)} frames at {fps:.1f} Hz")
    print("Delta frames (best first): frame, lag, position gain / R2, rotation gain / R2")
    for f in fit_delta_frames(poses, tcp)[:4]:
        print(
            f"  {f.frame:32s} lag {f.lag}  pos {f.pos_gain:+.3f} / {f.pos_r2:.2f}"
            f"  rot {f.rot_gain:+.3f} / {f.rot_r2:.2f}"
        )
    print(f"As absolute targets instead: R2 {absolute_r2(poses, tcp):.2f} (delta if well below)")
    grip = fit_gripper(state[:, 6], action[:, 6])
    if grip is not None:
        sign = "negative" if grip.close_when_negative else "positive"
        print(
            f"Gripper: a {sign} command closes; state open {grip.open_state:.4g}, "
            f"holding {grip.holding_state:.4g} (close at frame {grip.close_frame})"
        )
    env = path_summary(poses, joints, fps)
    print(
        f"Envelope: x {env.lo[0]:.0f}..{env.hi[0]:.0f}, y {env.lo[1]:.0f}..{env.hi[1]:.0f}, "
        f"z {env.lo[2]:.0f}..{env.hi[2]:.0f} mm; speed median {env.speed_median:.0f}, "
        f"max {env.speed_max:.0f} mm/s"
    )
    print(f"Start joints (deg): {np.round(np.degrees(env.start_joints), 1).tolist()}")


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    parser.add_argument("--episode", required=True)
    parser.add_argument("--controller", default="ur10e-calibration")
    parser.add_argument("--tcp", default="umi_corrected")
    asyncio.run(_main(parser.parse_args()))
