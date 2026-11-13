"""Inspect a deployed GR00T policy and check its declared config (``policy.toml``).

Nothing moves. Run (see README.md next to this file)::

    # check a declared config against the server, the checkpoint, the training data, the robot
    uv run python -m novapolicy.gr00t.inspect_policy --config policy.toml --data <folder>

    # start a new config: facts filled in, everything that must be declared left as TODO
    uv run python -m novapolicy.gr00t.inspect_policy --groot-host <host> --groot-port <port> \\
        --controller <controller> --tcp <tcp> --data <folder> --write-template policy.toml

Evidence, each optional except the server:
- the GR00T server: keys, action horizon, language key, how actions are returned
- the checkpoint the server loaded (``statistics.json``, ``processor_config.json``,
  ``experiment_cfg/conf.yaml``): found under ``--data``, given as ``--checkpoint``, or read
  from the server's pod with ``--kubeconfig``
- the training dataset's ``meta/`` (``info.json``, ``tasks.jsonl``, ``modality.json``) and
  episodes (``episode_*.parquet``): found under ``--data``, or ``--meta`` / ``--episode``
- the robot through ``NOVA_API``: its joints, kinematics and TCP

With ``--config`` every declared value is checked against the evidence: PASS, WARN, FAIL,
or SKIP when no evidence covers it. Any FAIL exits with code 1. Fits against training
episodes only ever check a declared value; they never supply one.
"""

from __future__ import annotations

import argparse
import asyncio
from dataclasses import dataclass, field
import importlib
import json
import math
from pathlib import Path
import re
import subprocess  # ruff: ignore[suspicious-subprocess-import] - kubectl only
import sys
from typing import TYPE_CHECKING, Any, cast

import numpy as np

from nova import Nova
from novapolicy.gr00t import policy_config, policy_fit
from novapolicy.gr00t.client import Gr00tPolicyClient

if TYPE_CHECKING:
    from collections.abc import Iterable

CHECKPOINT_FILES = ("statistics.json", "processor_config.json", "experiment_cfg/conf.yaml")
META_FILES = ("meta/info.json", "meta/tasks.jsonl", "meta/modality.json")
MAX_EPISODES = 200  # envelope over at most this many; the action and gripper fit use the first
_POSE_DIM = 6
_HW = 2  # an image shape starts with (height, width)
_BINARY_MIN_STD = 0.8  # a -1/+1 action spends most steps at the extremes
# How far a declared value may be from what the evidence shows.
_GAIN_TOLERANCE = 0.15  # relative, position gain
_ROT_GAIN_TOLERANCE = 0.30  # relative, rotation gain (fits less tightly)
_FRAME_R2_MARGIN = 0.05  # the declared frame must fit within this of the best candidate
_RATE_TOLERANCE = 0.01  # relative
_GRIPPER_TOLERANCE = 0.1  # of the declared open-closed span
_HOME_TOLERANCE_DEG = 2.0


# --- Evidence ---------------------------------------------------------------------------------


@dataclass
class EpisodeEvidence:
    name: str
    fps: float
    joints_key: str
    tcp_key: str
    fits: list[policy_fit.FrameFit]
    absolute_r2: float
    gripper: policy_fit.GripperFit | None
    envelope: policy_fit.PathSummary


@dataclass
class RobotEvidence:
    joints: list[float]
    kinematics: policy_fit.Kinematics | None
    fk_error_mm: float | None
    tcps: list[str]
    error: str = ""


@dataclass
class Evidence:
    host: str
    port: int
    keys: dict[str, list[str]]
    horizon: int
    action_configs: dict[str, dict[str, Any]]
    stats: dict[str, Any] = field(default_factory=dict)  # this embodiment's statistics
    datasets: list[str] = field(default_factory=list)
    processor: dict[str, Any] = field(default_factory=dict)
    fps: float | None = None  # meta/info.json
    images: dict[str, tuple[int, int]] = field(default_factory=dict)  # feature -> (h, w)
    prompts: list[str] = field(default_factory=list)
    layout: dict[str, Any] = field(default_factory=dict)  # meta/modality.json
    episode: EpisodeEvidence | None = None
    robot: RobotEvidence | None = None
    notes: list[str] = field(default_factory=list)


def _checkpoint_number(path: Path) -> int:
    match = re.search(r"checkpoint-(\d+)", str(path))
    return int(match.group(1)) if match else -1


def discover(args: argparse.Namespace) -> None:
    """Fill ``--checkpoint``, ``--meta`` and ``--episode`` from a ``--data`` folder.

    The checkpoint is the folder holding ``statistics.json`` (the newest ``checkpoint-N`` if
    there are several), the meta the folder of ``meta/info.json``, the episodes every
    ``episode_*.parquet``. Arguments given explicitly win.
    """
    if not args.data:
        return
    root = Path(args.data).expanduser()
    if not root.is_dir():
        msg = f"--data {root} is not a directory"
        raise SystemExit(msg)
    if not (args.checkpoint or args.kubeconfig):
        stats = sorted(root.rglob("statistics.json"), key=lambda p: _checkpoint_number(p.parent))
        if stats:
            args.checkpoint = str(stats[-1].parent)
    if not args.meta:
        infos = sorted(root.rglob("meta/info.json"))
        if infos:
            args.meta = str(infos[0].parent)
    if not args.episode:
        args.episode = [str(p) for p in sorted(root.rglob("episode_*.parquet"))[:MAX_EPISODES]]
    print(
        f"(--data {root}: checkpoint {args.checkpoint}, meta {args.meta}, "
        f"{len(args.episode or [])} episodes)"
    )


def read_files(args: argparse.Namespace, names: tuple[str, ...]) -> dict[str, str]:
    """Raw text of the files that exist at the checkpoint source; empty without one."""
    texts: dict[str, str] = {}
    for name in names:
        if args.checkpoint:
            path = Path(args.checkpoint) / name
            if path.exists():
                texts[name] = path.read_text()
        elif args.kubeconfig:
            namespace, target = args.kube_target.split("/", 1)
            cmd = ["kubectl", "--kubeconfig", str(Path(args.kubeconfig).expanduser())]
            cmd += ["exec", "-n", namespace, target, "-c", args.kube_container]
            cmd += ["--", "cat", f"{args.kube_path}/{name}"]
            # The command is built from this script's own CLI arguments.
            result = subprocess.run(  # ruff: ignore[subprocess-without-shell-equals-true]
                cmd, capture_output=True, text=True, check=False
            )
            if result.returncode == 0:
                texts[name] = result.stdout
    return texts


def read_meta(args: argparse.Namespace) -> dict[str, str]:
    """Dataset meta files: from ``--meta``, else from the checkpoint source."""
    if args.meta:
        return {
            f"meta/{p.name}": p.read_text()
            for p in (Path(args.meta) / n.split("/", 1)[1] for n in META_FILES)
            if p.exists()
        }
    return read_files(args, META_FILES)


def _modality(config: dict[str, Any], name: str) -> dict[str, Any]:
    entry = config.get(name) or {}
    return entry.get("as_json", entry) if isinstance(entry, dict) else {}


async def read_server(host: str, port: int) -> Evidence:
    client = Gr00tPolicyClient(host=host, port=port)
    await client.connect([])
    try:
        config = await client.get_modality_config()
        info = await client.get_server_info()
    finally:
        await client.close()
    names = ("state", "action", "video", "language")
    keys = {n: list(_modality(config, n).get("modality_keys") or []) for n in names}
    configs = _modality(config, "action").get("action_configs") or []
    return Evidence(
        host=host,
        port=port,
        keys=keys,
        horizon=int(cast("int", info["action_horizon"])),
        action_configs=dict(zip(keys["action"], configs, strict=False)),
    )


def add_checkpoint(ev: Evidence, texts: dict[str, str]) -> None:
    if "statistics.json" not in texts:
        ev.notes.append("checkpoint not read (pass --data, --checkpoint or --kubeconfig)")
        return
    for entry in json.loads(texts["statistics.json"]).values():
        if isinstance(entry, dict) and set(ev.keys["state"]) <= set(entry.get("state", {})):
            ev.stats = entry
            break
    ev.datasets = re.findall(
        r"dataset_paths:\s*\n\s*-\s*(\S+)", texts.get("experiment_cfg/conf.yaml", "")
    )
    if "processor_config.json" in texts:
        ev.processor = json.loads(texts["processor_config.json"]).get("processor_kwargs", {})


def add_meta(ev: Evidence, meta: dict[str, str]) -> None:
    if "meta/info.json" in meta:
        info = json.loads(meta["meta/info.json"])
        ev.fps = float(info["fps"]) if "fps" in info else None
        for name, feature in info.get("features", {}).items():
            shape = feature.get("shape") or []
            if feature.get("dtype") in {"video", "image"} and len(shape) >= _HW:
                ev.images[name] = (int(shape[0]), int(shape[1]))
    if "meta/tasks.jsonl" in meta:
        lines = meta["meta/tasks.jsonl"].splitlines()
        ev.prompts = [json.loads(line).get("task") for line in lines if line.strip()]
    if "meta/modality.json" in meta:
        ev.layout = json.loads(meta["meta/modality.json"])


async def read_robot(controller: str, tcp: str) -> RobotEvidence:
    try:  # ruff: ignore[too-many-statements-in-try-clause]
        async with Nova() as nova:
            mg = (await nova.cell().controller(controller))[0]
            description = await mg.get_description()
            tcps = sorted((description.tcps or {}).keys())
            joints = list((await mg.get_state()).joints)
            if tcp not in tcps:
                return RobotEvidence(joints, None, None, tcps, f"TCP {tcp!r} not on {controller}")
            state = await mg.get_state(tcp)
            kinematics = policy_fit.Kinematics.from_description(description, tcp)
            measured = policy_fit.pose_matrix(state.pose.position, state.pose.orientation)
            error = float(np.linalg.norm(kinematics.fk(state.joints)[:3, 3] - measured[:3, 3]))
            return RobotEvidence(joints, kinematics, error, tcps)
    except Exception as e:  # ruff: ignore[blind-except] - reported as evidence
        return RobotEvidence([], None, None, [], f"{type(e).__name__}: {e}")


def _columns(ev: Evidence, group: str, width: int) -> dict[str, slice]:
    """Where each key sits in the episode's ``width``-wide state/action vector."""
    keys = ev.keys[group]
    layout = ev.layout.get(group, {})
    if all(k in layout for k in keys):
        return {k: slice(layout[k]["start"], layout[k]["end"]) for k in keys}
    sizes = [len(ev.stats.get(group, {}).get(k, {}).get("min", [])) or None for k in keys]
    if None in sizes or sum(s or 0 for s in sizes) != width:
        return {}
    out, start = {}, 0
    for key, size in zip(keys, sizes, strict=True):
        out[key] = slice(start, start + (size or 0))
        start += size or 0
    return out


def _load_episode(path: Path) -> tuple[np.ndarray, np.ndarray, float]:
    pq = importlib.import_module("pyarrow.parquet")  # optional: only for episodes
    table = pq.read_table(path)
    state = np.asarray(table.column("observation.state").to_pylist(), dtype=float)
    action = np.asarray(table.column("action").to_pylist(), dtype=float)
    stamps = np.asarray(table.column("timestamp").to_pylist(), dtype=float)
    return state, action, 1.0 / float(np.median(np.diff(stamps)))


def _summary(
    path: Path, kinematics: policy_fit.Kinematics, joint_columns: slice
) -> policy_fit.PathSummary:
    state, _action, fps = _load_episode(path)
    joints = state[:, joint_columns]
    return policy_fit.path_summary([kinematics.fk(q) for q in joints], joints, fps)


def _gripper_fit(
    state: np.ndarray, action: np.ndarray, s_cols: dict[str, slice], a_cols: dict[str, slice]
) -> policy_fit.GripperFit | None:
    scalar_state = next((c for c in s_cols.values() if c.stop - c.start == 1), None)
    scalar_action = next((c for c in a_cols.values() if c.stop - c.start == 1), None)
    if scalar_state is None or scalar_action is None:
        return None
    return policy_fit.fit_gripper(state[:, scalar_state][:, 0], action[:, scalar_action][:, 0])


def add_episodes(ev: Evidence, paths: list[Path], kinematics: policy_fit.Kinematics) -> None:
    state, action, fps = _load_episode(paths[0])
    s_cols = _columns(ev, "state", state.shape[1])
    a_cols = _columns(ev, "action", action.shape[1])
    joints_key = next((k for k, c in s_cols.items() if c.stop - c.start == _POSE_DIM), None)
    tcp_key = next((k for k, c in a_cols.items() if c.stop - c.start == _POSE_DIM), None)
    if joints_key is None or tcp_key is None:
        ev.notes.append(
            "episodes: column layout unknown (needs meta/modality.json or a checkpoint)"
        )
        return
    joints = state[:, s_cols[joints_key]]
    poses = [kinematics.fk(q) for q in joints]
    tcp_actions = action[:, a_cols[tcp_key]]
    summaries = [policy_fit.path_summary(poses, joints, fps)]
    summaries += [_summary(p, kinematics, s_cols[joints_key]) for p in paths[1:]]
    ev.episode = EpisodeEvidence(
        name=paths[0].name,
        fps=fps,
        joints_key=joints_key,
        tcp_key=tcp_key,
        fits=policy_fit.fit_delta_frames(poses, tcp_actions),
        absolute_r2=policy_fit.absolute_r2(poses, tcp_actions),
        gripper=_gripper_fit(state, action, s_cols, a_cols),
        envelope=policy_fit.merge_summaries(summaries),
    )


# --- Report -----------------------------------------------------------------------------------


def _fmt(values: Iterable[float]) -> str:
    return "[" + ", ".join(f"{v:.4g}" for v in values) + "]"


def _is_binary(s: dict[str, list[float]]) -> bool:
    lo, hi, std = s["min"], s["max"], s["std"]
    return (
        len(lo) == 1
        and math.isclose(lo[0], -1, abs_tol=1e-3)
        and math.isclose(hi[0], 1, abs_tol=1e-3)
        and std[0] > _BINARY_MIN_STD
    )


def print_report(ev: Evidence) -> None:
    print(f"== GR00T server {ev.host}:{ev.port}")
    for name, value in ev.keys.items():
        print(f"  {name:8s}: {value}")
    print(f"  horizon : {ev.horizon} steps")
    for key, cfg in ev.action_configs.items():
        print(f"  action.{key}: {cfg.get('rep')}/{cfg.get('type')}/{cfg.get('format')}")
    if ev.stats:
        print("== checkpoint")
        if ev.datasets:
            print(f"  trained on: {', '.join(ev.datasets)}")
        stats = [(g, k, s) for g in ("state", "action") for k, s in ev.stats.get(g, {}).items()]
        for group, key, s in stats:
            print(f"  {group}.{key}: min {_fmt(s['min'])}, max {_fmt(s['max'])}")
    if ev.fps or ev.images or ev.prompts:
        print("== dataset meta")
        print(f"  rate {ev.fps} Hz; images {ev.images}; prompts {ev.prompts}")
    if ev.episode:
        _print_episode(ev.episode)
    if ev.robot:
        r = ev.robot
        status = r.error or f"forward kinematics within {r.fk_error_mm:.2f} mm of NOVA's TCP pose"
        print(f"== robot: {status}")
    for note in ev.notes:
        print(f"  ({note})")


def _print_episode(e: EpisodeEvidence) -> None:
    print(f"== episodes ({e.envelope.episodes}; fits on {e.name}, {e.fps:.4g} Hz)")
    best = {f.frame: f for f in reversed(e.fits)}
    for f in sorted(best.values(), key=lambda f: f.pos_r2, reverse=True):
        print(
            f"  {e.tcp_key} as {f.frame} delta, lag {f.lag}: gain {f.pos_gain:.3f} "
            f"(R^2 {f.pos_r2:.2f}), rotation gain {f.rot_gain:.3f} (R^2 {f.rot_r2:.2f})"
        )
    print(f"  {e.tcp_key} as absolute target positions: R^2 {e.absolute_r2:.2f}")
    if e.gripper:
        g = e.gripper
        sign = "negative" if g.close_when_negative else "positive"
        print(
            f"  gripper: closes on {sign}; state open {g.open_state:.4g}, holding {g.holding_state:.4g}"
        )
    v = e.envelope
    print(
        f"  envelope: x {v.lo[0]:.0f}..{v.hi[0]:.0f}, y {v.lo[1]:.0f}..{v.hi[1]:.0f}, z >= "
        f"{v.lo[2]:.0f} mm; speed max {v.speed_max:.0f} mm/s; tilt <= {v.tilt_max:.1f} deg; "
        f"yaw {v.yaw_lo:+.1f}..{v.yaw_hi:+.1f} deg"
    )


# --- Validation -------------------------------------------------------------------------------


@dataclass
class Check:
    status: str  # PASS, WARN, FAIL, SKIP
    item: str
    detail: str


def _same(item: str, declared: object, actual: object) -> Check:
    if declared == actual:
        return Check("PASS", item, f"{declared}")
    return Check("FAIL", item, f"declared {declared}, evidence {actual}")


def _check_server(cfg: policy_config.PolicyConfig, ev: Evidence) -> list[Check]:
    p = cfg.policy
    checks = [
        _same("server state keys", sorted(p.state_keys), sorted(ev.keys["state"])),
        _same("server action keys", sorted(p.action_keys), sorted(ev.keys["action"])),
        _same("server video keys", sorted(p.video_keys), sorted(ev.keys["video"])),
        _same("server language key", [p.language_key], ev.keys["language"]),
        _same("server action horizon", p.action_horizon, ev.horizon),
    ]
    rep = ev.action_configs.get(p.tcp_action.key, {}).get("rep")
    if rep == "RELATIVE":
        checks.append(
            Check("FAIL", "tcp action raw", "the server converts it relative->absolute itself")
        )
    elif rep is not None:
        checks.append(Check("PASS", "tcp action raw", f"server returns it as is ({rep})"))
    return checks


def _check_checkpoint(cfg: policy_config.PolicyConfig, ev: Evidence) -> list[Check]:
    p = cfg.policy
    if not ev.stats:
        return [Check("SKIP", "checkpoint", "not read")]
    checks: list[Check] = []
    joints = ev.stats.get("state", {}).get(p.joints.key)
    if joints and ev.robot and len(ev.robot.joints) == len(joints["min"]):
        live = (
            ev.robot.joints
            if p.joints.unit == "rad"
            else [math.degrees(q) for q in ev.robot.joints]
        )
        inside = all(
            a <= q <= b for q, a, b in zip(live, joints["min"], joints["max"], strict=True)
        )
        checks.append(
            Check(
                "PASS" if inside else "FAIL",
                "joint unit",
                f"robot joints in {p.joints.unit} {'inside' if inside else 'outside'} the trained range",
            )
        )
    else:
        checks.append(Check("SKIP", "joint unit", "needs the checkpoint and the robot"))
    gripper_state = ev.stats.get("state", {}).get(p.gripper.state_key)
    if gripper_state:
        lo, hi = gripper_state["min"][0], gripper_state["max"][0]
        span = abs(p.gripper.state_open - p.gripper.state_closed) * _GRIPPER_TOLERANCE
        for name, value in (("open", p.gripper.state_open), ("holding", p.gripper.state_holding)):
            ok = lo - span <= value <= hi + span
            checks.append(
                Check(
                    "PASS" if ok else "FAIL",
                    f"gripper state {name}",
                    f"{value} {'within' if ok else 'outside'} trained {lo:.4g}..{hi:.4g}",
                )
            )
    gripper_action = ev.stats.get("action", {}).get(p.gripper.action_key)
    if gripper_action:
        binary = _is_binary(gripper_action)
        checks.append(
            Check(
                "PASS" if binary == (p.gripper.action == "binary") else "FAIL",
                "gripper action binary",
                f"trained values {'are' if binary else 'are not'} -1/+1",
            )
        )
    return checks


def _check_meta(cfg: policy_config.PolicyConfig, ev: Evidence) -> list[Check]:
    p = cfg.policy
    checks: list[Check] = []
    if ev.fps is None:
        checks.append(Check("SKIP", "rate (meta)", "no meta/info.json"))
    else:
        ok = math.isclose(ev.fps, p.rate_hz, rel_tol=_RATE_TOLERANCE)
        checks.append(
            Check("PASS" if ok else "FAIL", "rate (meta)", f"declared {p.rate_hz}, fps {ev.fps}")
        )
    if ev.images:
        sizes = set(ev.images.values())
        ok = sizes == {(p.image_height, p.image_width)}
        checks.append(
            Check(
                "PASS" if ok else "FAIL",
                "image size (meta)",
                f"declared {p.image_width}x{p.image_height}, dataset (h, w) {sorted(sizes)}",
            )
        )
    if ev.prompts:
        ok = p.prompt in ev.prompts
        checks.append(
            Check("PASS" if ok else "FAIL", "prompt (meta)", f"dataset prompts {ev.prompts}")
        )
    return checks


def _check_episode_action(cfg: policy_config.PolicyConfig, e: EpisodeEvidence) -> list[Check]:
    a = cfg.policy.tcp_action
    rot = None if a.frame == "tool" else a.base_rotation_z_deg
    declared = [
        f
        for f in e.fits
        if (f.rot_z_deg is None) == (rot is None) and (rot is None or f.rot_z_deg == rot)
    ]
    if not declared:
        return [
            Check(
                "SKIP",
                "tcp action frame",
                f"the fit only tests base rotations 0/90/180/270, not {rot}",
            )
        ]
    fit, best = declared[0], e.fits[0]
    checks = [
        Check(
            "PASS" if fit.pos_r2 >= policy_fit.R2_GOOD else "FAIL",
            "tcp action as delta",
            f"declared frame fits R^2 {fit.pos_r2:.2f} (absolute targets: {e.absolute_r2:.2f})",
        ),
        Check(
            "PASS" if fit.pos_r2 >= best.pos_r2 - _FRAME_R2_MARGIN else "FAIL",
            "tcp action frame",
            f"declared fits R^2 {fit.pos_r2:.2f}; best is {best.frame} at {best.pos_r2:.2f}",
        ),
    ]
    if a.representation == "delta" and e.absolute_r2 > fit.pos_r2:
        checks.append(Check("FAIL", "tcp action representation", "fits better as absolute targets"))
    scale = 1000.0 / a.position_scale_mm  # the fit works in metres
    for name, declared_gain, fitted, tolerance in (
        ("tcp gain position", a.gain_position, fit.pos_gain * scale, _GAIN_TOLERANCE),
        ("tcp gain rotation", a.gain_rotation, fit.rot_gain, _ROT_GAIN_TOLERANCE),
    ):
        ok = math.isclose(declared_gain, fitted, rel_tol=tolerance)
        checks.append(
            Check("PASS" if ok else "FAIL", name, f"declared {declared_gain}, episode {fitted:.3f}")
        )
    return checks


def _check_episode(cfg: policy_config.PolicyConfig, ev: Evidence) -> list[Check]:
    e = ev.episode
    if e is None:
        return [
            Check("SKIP", "training episodes", "none read (needs --data/--episode and the robot)")
        ]
    p, c = cfg.policy, cfg.cell
    ok = math.isclose(e.fps, p.rate_hz, rel_tol=_RATE_TOLERANCE)
    checks = [
        Check(
            "PASS" if ok else "FAIL",
            "rate (episode)",
            f"declared {p.rate_hz}, episode {e.fps:.4g} Hz",
        )
    ]
    checks += _check_episode_action(cfg, e)
    if e.gripper:
        g, d = e.gripper, p.gripper
        checks.append(
            _same(
                "gripper closes when",
                d.close_when,
                "negative" if g.close_when_negative else "positive",
            )
        )
        span = abs(d.state_open - d.state_closed) * _GRIPPER_TOLERANCE
        ok = abs(g.open_state - d.state_open) <= span
        checks.append(
            Check(
                "PASS" if ok else "FAIL",
                "gripper state open (episode)",
                f"declared {d.state_open}, episode {g.open_state:.4g}",
            )
        )
        ok = abs(g.holding_state - d.state_holding) <= span
        checks.append(
            Check(
                "PASS" if ok else "WARN",
                "gripper state holding (episode)",
                f"declared {d.state_holding}, episode {g.holding_state:.4g}",
            )
        )
    start = [math.degrees(q) for q in e.envelope.start_joints]
    off = max(abs(a - b) for a, b in zip(start, c.home_deg, strict=True))
    checks.append(
        Check(
            "PASS" if off <= _HOME_TOLERANCE_DEG else "WARN",
            "home vs. episode start",
            f"max {off:.1f} deg apart",
        )
    )
    return checks


def _check_guards(cfg: policy_config.PolicyConfig, ev: Evidence) -> list[Check]:
    """Guards are the deployer's choice; warn where they would stop trained motion."""
    if ev.episode is None:
        return [Check("SKIP", "guards", "no training episodes to compare with")]
    g, v = cfg.guards, ev.episode.envelope
    tighter = [
        (
            g.max_tcp_speed_mm_s < v.speed_max,
            f"max speed {g.max_tcp_speed_mm_s} < trained {v.speed_max:.0f} mm/s",
        ),
        (g.min_tcp_z_mm > v.lo[2], f"min z {g.min_tcp_z_mm} > trained {v.lo[2]:.0f} mm"),
        (g.max_tilt_deg < v.tilt_max, f"max tilt {g.max_tilt_deg} < trained {v.tilt_max:.1f} deg"),
        (
            g.max_yaw_deg < max(abs(v.yaw_lo), abs(v.yaw_hi)),
            f"max yaw {g.max_yaw_deg} < trained {max(abs(v.yaw_lo), abs(v.yaw_hi)):.1f} deg",
        ),
        (
            not g.workspace_x_mm[0] <= v.lo[0] <= v.hi[0] <= g.workspace_x_mm[1],
            f"workspace x {g.workspace_x_mm} vs. trained {v.lo[0]:.0f}..{v.hi[0]:.0f}",
        ),
        (
            not g.workspace_y_mm[0] <= v.lo[1] <= v.hi[1] <= g.workspace_y_mm[1],
            f"workspace y {g.workspace_y_mm} vs. trained {v.lo[1]:.0f}..{v.hi[1]:.0f}",
        ),
    ]
    found = [text for broken, text in tighter if broken]
    if found:
        return [Check("WARN", "guards vs. training envelope", "; ".join(found))]
    return [Check("PASS", "guards vs. training envelope", f"cover {v.episodes} episode(s)")]


def _check_robot(cfg: policy_config.PolicyConfig, ev: Evidence) -> list[Check]:
    if ev.robot is None:
        return [Check("SKIP", "robot", "not read")]
    if ev.robot.error:
        return [Check("FAIL", "robot / TCP", ev.robot.error)]
    return [
        Check(
            "PASS",
            "robot / TCP",
            f"{cfg.cell.tcp} on {cfg.cell.controller}; FK within {ev.robot.fk_error_mm:.2f} mm",
        )
    ]


def validate(cfg: policy_config.PolicyConfig, ev: Evidence) -> list[Check]:
    return [
        *_check_server(cfg, ev),
        *_check_checkpoint(cfg, ev),
        *_check_meta(cfg, ev),
        *_check_robot(cfg, ev),
        *_check_episode(cfg, ev),
        *_check_guards(cfg, ev),
    ]


# --- Template ---------------------------------------------------------------------------------


def _toml(value: object) -> str:
    return json.dumps(value)  # strings, numbers and flat lists read the same in TOML


def template(ev: Evidence, controller: str, tcp: str) -> str:
    """A ``policy.toml`` with the facts filled in; everything to declare is ``"TODO"``."""
    todo = _toml(policy_config.TODO)
    scalar = [
        k for k in ev.keys["state"] if len(ev.stats.get("state", {}).get(k, {}).get("min", [])) == 1
    ]
    six = [
        k
        for k in ev.keys["state"]
        if len(ev.stats.get("state", {}).get(k, {}).get("min", [])) == _POSE_DIM
    ]
    tcp_keys = [
        k
        for k in ev.keys["action"]
        if len(ev.stats.get("action", {}).get(k, {}).get("min", [])) == _POSE_DIM
    ]
    g_action = [
        k
        for k in ev.keys["action"]
        if len(ev.stats.get("action", {}).get(k, {}).get("min", [])) == 1
    ]
    hints: dict[str, str] = {}
    if ev.episode:
        e = ev.episode
        best = e.fits[0]
        hints["frame"] = f"episode fits best: {best.frame} (R^2 {best.pos_r2:.2f})"
        hints["gain"] = f"episode: position {best.pos_gain:.3f}, rotation {best.rot_gain:.3f}"
        if e.gripper:
            g = e.gripper
            hints["gripper"] = (
                f"episode: closes on {'negative' if g.close_when_negative else 'positive'}, "
                f"open {g.open_state:.4g}, holding {g.holding_state:.4g}"
            )
        v = e.envelope
        hints["guards"] = (
            f"training envelope: speed <= {v.speed_max:.0f} mm/s, x {v.lo[0]:.0f}..{v.hi[0]:.0f}, "
            f"y {v.lo[1]:.0f}..{v.hi[1]:.0f}, z >= {v.lo[2]:.0f}, tilt <= {v.tilt_max:.1f}, "
            f"yaw {v.yaw_lo:+.1f}..{v.yaw_hi:+.1f}"
        )
    height, width = next(iter(ev.images.values()), (None, None))
    lines = [
        "# Generated by novapolicy.gr00t.inspect_policy: facts filled in, every TODO must be",
        "# declared from the training setup and the cell. Comments show evidence, not values.",
        "",
        "[server]",
        f"host = {_toml(ev.host)}",
        f"port = {ev.port}",
        "",
        "[policy]",
        f"prompt = {_toml(ev.prompts[0]) if ev.prompts else todo}",
        f"language_key = {_toml(ev.keys['language'][0]) if ev.keys['language'] else todo}",
        f"rate_hz = {ev.fps if ev.fps else todo}",
        f"action_horizon = {ev.horizon}",
        f"state_keys = {_toml(ev.keys['state'])}",
        f"action_keys = {_toml(ev.keys['action'])}",
        f"video_keys = {_toml(ev.keys['video'])}",
        f"image_width = {width or todo}",
        f"image_height = {height or todo}",
        "",
        "[policy.joints]",
        f"key = {_toml(six[0]) if len(six) == 1 else todo}",
        f"unit = {todo}  # rad or deg",
        "",
        "[policy.tcp_action]",
        f"key = {_toml(tcp_keys[0]) if len(tcp_keys) == 1 else todo}",
        f"representation = {todo}  # delta",
        f"frame = {todo}  # base or tool; {hints.get('frame', 'no episode')}",
        "base_rotation_z_deg = 0.0",
        f"position_unit = {todo}  # m or mm",
        'rotation_format = "rotvec"',
        f"gain_position = {todo}  # {hints.get('gain', 'no episode')}",
        f"gain_rotation = {todo}",
        "",
        "[policy.gripper]",
        f"state_key = {_toml(scalar[0]) if len(scalar) == 1 else todo}",
        f"action_key = {_toml(g_action[0]) if len(g_action) == 1 else todo}",
        f"action = {todo}  # binary",
        f"close_when = {todo}  # negative or positive; {hints.get('gripper', 'no episode')}",
        f"state_open = {todo}",
        f"state_closed = {todo}",
        f"state_holding = {todo}",
        "",
        "[cell]",
        f"controller = {_toml(controller)}",
        f"tcp = {_toml(tcp)}",
        f"home_deg = {todo}",
        f"camera_server = {todo}",
        f"gripper_host = {todo}",
        "",
        "[cell.cameras]  # video key -> camera serial",
        *[f"{key} = {todo}" for key in ev.keys["video"]],
        "",
        f"[guards]  # {hints.get('guards', 'no episode')}",
        *[
            f"{name} = {todo}"
            for name in (
                "max_tcp_speed_mm_s",
                "max_chunk_travel_mm",
                "max_rotation_step_deg",
                "max_tilt_deg",
                "max_yaw_deg",
                "workspace_x_mm",
                "workspace_y_mm",
                "min_tcp_z_mm",
            )
        ],
        "",
    ]
    return "\n".join(lines)


# --- Main -------------------------------------------------------------------------------------


async def gather(args: argparse.Namespace, cfg: policy_config.PolicyConfig | None) -> Evidence:
    host = args.groot_host or (cfg.server.host if cfg else None)
    port = args.groot_port or (cfg.server.port if cfg else 5555)
    controller = args.controller or (cfg.cell.controller if cfg else None)
    tcp = args.tcp or (cfg.cell.tcp if cfg else None)
    if host is None:
        msg = "--groot-host or --config is required"
        raise SystemExit(msg)
    ev = await read_server(host, port)
    discover(args)
    add_checkpoint(ev, read_files(args, CHECKPOINT_FILES))
    add_meta(ev, read_meta(args))
    if not args.no_robot and controller and tcp:
        ev.robot = await read_robot(controller, tcp)
    if args.episode:
        if ev.robot and ev.robot.kinematics:
            add_episodes(ev, [Path(e) for e in args.episode], ev.robot.kinematics)
        else:
            ev.notes.append("episodes not fitted: needs the robot's kinematics and TCP")
    return ev


async def main(args: argparse.Namespace) -> int:
    cfg = policy_config.load(args.config) if args.config else None
    ev = await gather(args, cfg)
    print_report(ev)
    if args.write_template:
        controller = args.controller or (cfg.cell.controller if cfg else policy_config.TODO)
        tcp = args.tcp or (cfg.cell.tcp if cfg else policy_config.TODO)
        text = template(ev, controller, tcp)
        await asyncio.to_thread(Path(args.write_template).write_text, text, encoding="utf-8")
        print(f"\nwrote {args.write_template}: declare every TODO, then run with --config")
    if cfg is None:
        return 0
    checks = validate(cfg, ev)
    print(f"\n== {args.config}")
    for check in checks:
        print(f"  [{check.status}] {check.item}: {check.detail}")
    failed = sum(c.status == "FAIL" for c in checks)
    print(f"\n{failed} failed, {sum(c.status == 'WARN' for c in checks)} warnings")
    return 1 if failed else 0


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--config", help="policy.toml to check")
    parser.add_argument("--write-template", help="write a policy.toml template to this path")
    parser.add_argument("--groot-host", help="GR00T server host (default: from --config)")
    parser.add_argument("--groot-port", type=int, help="GR00T server port (default: from --config)")
    parser.add_argument("--data", help="folder to search for checkpoint, meta/ and episodes")
    source = parser.add_mutually_exclusive_group()
    source.add_argument("--checkpoint", help="local checkpoint directory")
    source.add_argument("--kubeconfig", help="kubeconfig file: read the checkpoint from the pod")
    parser.add_argument("--kube-target", default="gr00t/deploy/gr00t", help="NAMESPACE/TARGET")
    parser.add_argument("--kube-container", default="gr00t")
    parser.add_argument("--kube-path", default="/models-ro", help="checkpoint path in the pod")
    parser.add_argument("--meta", help="the LeRobot dataset's meta/ folder")
    parser.add_argument("--episode", nargs="+", help="training episode parquet(s)")
    parser.add_argument("--controller", help="NOVA controller (default: from --config)")
    parser.add_argument("--tcp", help="TCP of the policy (default: from --config)")
    parser.add_argument("--no-robot", action="store_true", help="skip reading the robot")
    try:
        sys.exit(asyncio.run(main(parser.parse_args())))
    except policy_config.PolicyConfigError as e:
        sys.exit(f"invalid config: {e}")
