"""Write the settings a policy needs, measured from one of its training episodes.

Works for any number of robots. The dataset's ``meta/modality.json`` says which columns of
``observation.state`` and ``action`` belong to which key; ``--robot`` says which keys belong
to which motion group. For every robot, the episode's recorded joints, run through the
robot's forward kinematics (DH parameters and TCP from NOVA), give the path the training
robot actually moved. Comparing that path with the recorded actions tells what each action
means: absolute joint targets, joint deltas, absolute TCP targets or TCP deltas (and for
deltas, which coordinate system), the scale between action units and realized motion (for
a differential-IK training controller, the share of each command it realized), and how a
binary gripper is driven. The episode also gives the rate and each robot's start joints.

``--write`` stores the result as JSON (:class:`EpisodeSettings`); a run script loads it and
builds each robot's action with :meth:`RobotSettings.action`. Nothing is guessed: a fit
below ``R2_GOOD``, an ambiguous answer or a convention ``Action`` cannot express stops the
script instead of writing settings.

    NOVA_API=http://<nova> uv run python settings_from_episode.py \
        --episode <dataset>/data/chunk-000/episode_000075.parquet \
        --modality <dataset>/meta/modality.json \
        --robot arm mg=0@ur10e-calibration tcp=umi_corrected joints=arm_joints \
                action=eef_delta gripper=gripper gripper_closed=-0.023 \
        --write episode_settings.json

Per robot: ``tcp`` is the point on the tool the training poses refer to (it cannot be
measured); ``gripper_closed`` is the fully closed gripper state (the training config's close
command; an episode never shows it because the fingers stop on the object). Use
``gripper_action=`` when the gripper's action key differs from its state key. A dual-arm
policy passes ``--robot`` twice.
"""

from __future__ import annotations

import argparse
import asyncio
from dataclasses import asdict, dataclass
import json
import math
from pathlib import Path
import sys
from typing import TYPE_CHECKING, Any, cast

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


_JOINT_KINDS = ("joint_absolute", "joint_delta")
_TCP_KINDS = ("tcp_absolute", "tcp_delta")
_MAX_ABSOLUTE_OFFSET_MM = 5.0  # absolute TCP targets further off are in another frame


def _load(path: str) -> tuple[np.ndarray, np.ndarray, float]:
    import pyarrow.parquet as pq

    table = pq.read_table(path)
    state = np.array(table.column("observation.state").to_pylist(), dtype=float)
    action = np.array(table.column("action").to_pylist(), dtype=float)
    stamps = np.array(table.column("timestamp").to_pylist(), dtype=float)
    fps = 1.0 / float(np.median(np.diff(stamps))) if len(stamps) > 1 else 0.0
    return state, action, fps


def load_layout(path: str | Path) -> dict[str, dict[str, slice]]:
    """``{"state": {key: columns}, "action": {key: columns}}`` from a dataset's modality.json."""
    raw = json.loads(Path(path).read_text(encoding="utf-8"))
    return {
        part: {key: slice(int(v["start"]), int(v["end"])) for key, v in raw[part].items()}
        for part in ("state", "action")
    }


@dataclass(frozen=True)
class RobotSpec:
    """What ``--robot`` declares: the names a measurement cannot recover."""

    name: str
    motion_group: str  # e.g. "0@ur10e-calibration"
    tcp: str
    joints: str  # state key of the joint positions
    action: str  # action key of the arm
    gripper: str | None = None  # state key of the gripper
    gripper_action: str | None = None  # action key, if it differs from the state key
    gripper_closed: float | None = None

    @classmethod
    def parse(cls, words: list[str]) -> RobotSpec:
        name, *pairs = words
        fields: dict[str, Any] = dict(p.split("=", 1) for p in pairs)
        if "mg" in fields:
            fields["motion_group"] = fields.pop("mg")
        if "gripper_closed" in fields:
            fields["gripper_closed"] = float(fields["gripper_closed"])
        return cls(name=name, **fields)


@dataclass(frozen=True)
class RobotSettings:
    """What one robot's columns of the episode mean."""

    motion_group: str
    tcp: str
    joints_key: str
    action_key: str
    action_kind: str  # joint_absolute | joint_delta | tcp_absolute | tcp_delta
    fit_r2: float  # how well the chosen reading explains the motion (position for TCP)
    # Realized motion per action unit: rad (joints) or mm (TCP position), rad (TCP rotation).
    scale: float
    rotation_scale: float | None  # TCP kinds only
    use_tool_coordinate_system: bool  # tcp_delta only; False: world
    home_deg: list[float]  # the episode's start joints
    gripper_state_key: str | None
    gripper_action_key: str | None
    gripper_close_when_negative: bool | None
    gripper_open: float | None
    gripper_closed: float | None
    gripper_holding: float | None

    def action(self, motion_group: Any) -> Any:
        """The ``Action`` entry for this robot's arm, with units and scale as measured."""
        from novapolicy import Action, Scale

        to_unit = [Scale(1.0 / self.scale)]  # NOVA unit -> policy unit
        if self.action_kind in _JOINT_KINDS:
            mode = "relative" if self.action_kind == "joint_delta" else "absolute"
            return Action.joint_positions(
                self.action_key, target=motion_group, mode=mode, ops=to_unit
            )
        assert self.rotation_scale is not None
        common: dict[str, Any] = {
            "position_ops": [Scale(0.001), Scale(1000.0 / self.scale)],  # mm -> m -> unit
            "orientation_ops": [Scale(1.0 / self.rotation_scale)],
        }
        if self.action_kind == "tcp_absolute":
            return Action.tcp(self.action_key, target=motion_group, **common)
        return Action.tcp(
            self.action_key,
            target=motion_group,
            mode="relative",
            accumulate=True,
            use_tool_coordinate_system=self.use_tool_coordinate_system,
            **common,
        )


@dataclass(frozen=True)
class EpisodeSettings:
    episode: str
    rate_hz: float
    robots: dict[str, RobotSettings]

    @classmethod
    def load(cls, path: str | Path) -> EpisodeSettings:
        raw = json.loads(Path(path).read_text(encoding="utf-8"))
        robots = {name: RobotSettings(**r) for name, r in raw["robots"].items()}
        return cls(episode=raw["episode"], rate_hz=raw["rate_hz"], robots=robots)

    def write(self, path: str | Path) -> None:
        Path(path).write_text(json.dumps(asdict(self), indent=2) + "\n", encoding="utf-8")


def _joint_fits(joints: np.ndarray, action: np.ndarray) -> dict[str, tuple[float, float]]:
    """``{kind: (scale, r2)}`` for joint readings of ``action`` (same width as ``joints``)."""
    nxt = joints[1:]
    return {
        "joint_absolute": _gain_r2(action[:-1], nxt),  # target for the next step
        "joint_delta": _gain_r2(action[:-1], nxt - joints[:-1]),
    }


def _tcp_absolute(poses: list[np.ndarray], action: np.ndarray) -> tuple[float, float, float, float]:
    """(position scale mm/unit, R^2, offset mm, rotation scale) for absolute TCP targets."""
    target = np.array([p[:3, 3] for p in poses])[1:].ravel()
    design = np.column_stack([action[:-1, :3].ravel(), np.ones(target.size)])
    (scale, offset), *_ = np.linalg.lstsq(design, target, rcond=None)
    total = float(((target - target.mean()) ** 2).sum())
    r2 = 1.0 - float(((target - design @ [scale, offset]) ** 2).sum()) / total if total else 0.0
    rotvecs = Rotation.from_matrix(np.array([p[:3, :3] for p in poses[1:]])).as_rotvec()
    rot_scale, _ = _gain_r2(action[:-1, 3:6], rotvecs)
    return float(scale), r2, float(offset), rot_scale


def measure_robot(
    spec: RobotSpec,
    state: np.ndarray,
    action: np.ndarray,
    layout: dict[str, dict[str, slice]],
    kinematics: Kinematics,
) -> RobotSettings:
    """One robot's settings; raises ``ValueError`` where the episode cannot tell."""
    for part, key in (("state", spec.joints), ("action", spec.action)):
        if key not in layout[part]:
            msg = f"{spec.name}: {part} key {key!r} is not in modality.json {sorted(layout[part])}"
            raise ValueError(msg)
    joints = state[:, layout["state"][spec.joints]]
    arm = action[:, layout["action"][spec.action]]
    poses = [kinematics.fk(q) for q in joints]
    readings: dict[str, tuple[float, float]] = {}  # kind -> (scale, r2)
    tool = False
    rotation_scale: float | None = None
    if arm.shape[1] == joints.shape[1]:
        readings |= _joint_fits(joints, arm)
    if arm.shape[1] == _POSE_DIM:
        best = fit_delta_frames(poses, arm)[0]
        if best.lag == 0 and best.rot_z_deg in {None, 0}:
            readings["tcp_delta"] = (best.pos_gain * 1000.0, best.pos_r2)
            tool, delta_rot = best.rot_z_deg is None, best.rot_gain
        scale, r2, offset, abs_rot = _tcp_absolute(poses, arm)
        if abs(offset) <= _MAX_ABSOLUTE_OFFSET_MM:
            readings["tcp_absolute"] = (scale, r2)
    if not readings:
        msg = f"{spec.name}: a {arm.shape[1]}-wide action fits no reading Action can express"
        raise ValueError(msg)
    ranked = sorted(readings.items(), key=lambda kv: kv[1][1], reverse=True)
    kind, (scale, r2) = ranked[0]
    if r2 < R2_GOOD:
        msg = f"{spec.name}: no reading fits (best {kind}: R2 {r2:.2f}); joints in radians?"
        raise ValueError(msg)
    if len(ranked) > 1 and ranked[1][1][1] > r2 - 0.1:
        msg = (
            f"{spec.name}: ambiguous, {kind} R2 {r2:.2f} vs {ranked[1][0]} R2 {ranked[1][1][1]:.2f}"
        )
        raise ValueError(msg)
    if kind == "tcp_delta":
        rotation_scale = delta_rot
    elif kind == "tcp_absolute":
        rotation_scale = abs_rot
    grip = _measure_gripper(spec, state, action, layout)
    return RobotSettings(
        motion_group=spec.motion_group,
        tcp=spec.tcp,
        joints_key=spec.joints,
        action_key=spec.action,
        action_kind=kind,
        fit_r2=round(float(r2), 3),
        scale=round(float(scale), 6),
        rotation_scale=None if rotation_scale is None else round(float(rotation_scale), 6),
        use_tool_coordinate_system=kind == "tcp_delta" and tool,
        home_deg=[round(math.degrees(float(q)), 3) for q in joints[0]],
        **grip,
    )


def _measure_gripper(
    spec: RobotSpec, state: np.ndarray, action: np.ndarray, layout: dict[str, dict[str, slice]]
) -> dict[str, Any]:
    keys = {"gripper_state_key": spec.gripper, "gripper_action_key": None}
    empty = dict.fromkeys((
        "gripper_close_when_negative",
        "gripper_open",
        "gripper_closed",
        "gripper_holding",
    ))
    if spec.gripper is None:
        return keys | empty
    action_key = spec.gripper_action or spec.gripper
    for part, key in (("state", spec.gripper), ("action", action_key)):
        if key not in layout[part]:
            msg = f"{spec.name}: gripper {part} key {key!r} is not in modality.json"
            raise ValueError(msg)
    grip = fit_gripper(
        state[:, layout["state"][spec.gripper]][:, 0], action[:, layout["action"][action_key]][:, 0]
    )
    if grip is None:
        msg = f"{spec.name}: the gripper never switches in this episode"
        raise ValueError(msg)
    if spec.gripper_closed is None:
        msg = f"{spec.name}: pass gripper_closed= (the fully closed state from the training config)"
        raise ValueError(msg)
    return {
        "gripper_state_key": spec.gripper,
        "gripper_action_key": action_key,
        "gripper_close_when_negative": bool(grip.close_when_negative),
        "gripper_open": round(float(grip.open_state), 6),
        "gripper_closed": spec.gripper_closed,
        "gripper_holding": round(float(grip.holding_state), 6),
    }


async def _main(args: argparse.Namespace) -> None:
    import nova

    state, action, fps = _load(args.episode)
    layout = load_layout(args.modality)
    specs = [RobotSpec.parse(words) for words in args.robot]
    robots: dict[str, RobotSettings] = {}
    async with nova.Nova() as n:
        for spec in specs:
            controller = spec.motion_group.split("@", 1)[1]
            mg = (await n.cell().controller(controller)).motion_group(spec.motion_group)
            kinematics = Kinematics.from_description(await mg.get_description(), spec.tcp)
            try:
                robots[spec.name] = measure_robot(spec, state, action, layout, kinematics)
            except ValueError as e:
                sys.exit(f"Not writing settings: {e}")
            joints = state[:, layout["state"][spec.joints]]
            env = path_summary([kinematics.fk(q) for q in joints], joints, fps)
            print(  # for choosing TcpLimits wider than the motion the policy learned
                f"{spec.name} training envelope: x {env.lo[0]:.0f}..{env.hi[0]:.0f}, "
                f"y {env.lo[1]:.0f}..{env.hi[1]:.0f}, z {env.lo[2]:.0f}..{env.hi[2]:.0f} mm; "
                f"speed max {env.speed_max:.0f} mm/s"
            )
    settings = EpisodeSettings(
        episode=Path(args.episode).name, rate_hz=round(fps, 3), robots=robots
    )
    print(json.dumps(asdict(settings), indent=2))
    if args.write:
        settings.write(args.write)
        print(f"Wrote {args.write}")


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    parser.add_argument("--episode", required=True)
    parser.add_argument("--modality", required=True, help="the dataset's meta/modality.json")
    parser.add_argument(
        "--robot",
        nargs="+",
        action="append",
        required=True,
        metavar="NAME KEY=VALUE",
        help="NAME mg=<motion group> tcp=<tcp> joints=<state key> action=<action key> "
        "[gripper=<state key> gripper_action=<action key> gripper_closed=<value>]",
    )
    parser.add_argument("--write", help="write the settings to this JSON file")
    asyncio.run(_main(parser.parse_args()))
