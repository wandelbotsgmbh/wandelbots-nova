"""
Replay a recorded dataset episode's joint path on one or more robots.

Reads a LeRobot-format episode parquet (the format GR00T and LeRobot train on) and replays
its joint positions through waypoint jogging, time-faithfully at ``--speed`` times the
recorded rate. If the episode has a binary gripper column, the replay is split at the
gripper switches; between segments the arm stands still and ``on_gripper`` runs.

Without ``--controller`` it replays the bundled sample (``action`` + ``timestamp``
columns, two arms) on two virtual UR5e, as a quick look. With ``--controller`` it drives
real robots: a dry run by default, ``--execute`` asks before anything moves.

Grippers (``--gripper``): ``io:<name>`` writes a controller IO (True = closed);
``zimmer:<host>`` drives a Zimmer gripper through an IO-Link master with the optional
``zimmer-gripper-controller`` package (homed and opened before the replay, grips with
force). ``--stop-at-grip`` stops before the first grip with the jaws open, to place the
object between them.

Prerequisites:
    NOVA_API=http://<instance-ip>
    uv sync --extra novapolicy   (plus pyarrow)

Usage:
    # Bundled sample on two virtual UR5e
    uv run python novapolicy/examples/replay/replay_episode.py

    # A single real arm whose joints are recorded in observation.state, gripper in action[6]
    uv run python novapolicy/examples/replay/replay_episode.py --episode <episode.parquet> \\
        --controller <controller> --joints-from state --gripper-column 6 \\
        --gripper zimmer:<host> --tcp <tcp> --lift-to-mm 200 --speed 0.4 \\
        [--stop-at-grip] [--execute]
"""

from __future__ import annotations

import argparse
import asyncio
from dataclasses import dataclass
import importlib
from itertools import pairwise
from pathlib import Path
import time
from typing import TYPE_CHECKING, Any

import numpy as np
import pyarrow.parquet as pq

import nova
from nova.actions import joint_ptp, linear
from nova.cell import virtual_controller
from nova.program import ProgramPreconditions
from nova.types import MotionSettings, Pose
from novapolicy import (
    ActionChunk,
    CallbackPolicyClient,
    ContinuousExecution,
    Observation,
    PolicyExecutor,
    PolicySchema,
    WaypointConfig,
)

if TYPE_CHECKING:
    from collections.abc import Awaitable, Callable

    from nova.cell.controller import Controller
    from nova.cell.motion_group import MotionGroup

SAMPLE = Path(__file__).parent / "episode_000000.parquet"
JOINTS_PER_ARM = 6
CHUNK_SIZE = 8
START_MOVE = MotionSettings(tcp_velocity_limit=100.0)
ZIMMER_FORCE_PERCENT = 50  # in Zimmer force mode the force also sets the closing speed
ZIMMER_VELOCITY_PERCENT = 100


@dataclass
class Episode:
    joints: np.ndarray  # (N, JOINTS_PER_ARM * arms), radians
    gripper: np.ndarray | None  # (N,) binary gripper command, < 0 closes
    fps: float

    @property
    def duration_s(self) -> float:
        return (len(self.joints) - 1) / self.fps


def load_episode(
    path: Path, *, joints_from: str = "action", arms: int = 1, gripper_column: int | None = None
) -> Episode:
    """Joint path, gripper command and frame rate of one episode parquet.

    ``joints_from``: ``"action"`` when the actions are joint targets, ``"state"`` to replay
    the measured joints (``observation.state``), e.g. when the actions are end-effector
    deltas. The first ``JOINTS_PER_ARM * arms`` columns are the joints.
    """
    column = "observation.state" if joints_from == "state" else "action"
    names = pq.read_schema(path).names
    wanted = dict.fromkeys((column, "action", "timestamp"))  # ordered, no duplicates
    table = pq.read_table(path, columns=[c for c in wanted if c in names])
    values = np.asarray(table.column(column).to_pylist(), dtype=float)
    stamps = np.asarray(table.column("timestamp").to_pylist(), dtype=float)
    gripper = None
    if gripper_column is not None:
        gripper = np.asarray(table.column("action").to_pylist(), dtype=float)[:, gripper_column]
    return Episode(
        joints=values[:, : JOINTS_PER_ARM * arms],
        gripper=gripper,
        fps=1.0 / float(np.median(np.diff(stamps))),
    )


def segments(episode: Episode) -> list[tuple[int, int, bool | None]]:
    """Frame ranges between gripper switches: ``(start, end, close_after)``.

    ``close_after`` is True for a grip, False for a release, None at the end.
    """
    last = len(episode.joints) - 1
    if episode.gripper is None:
        return [(0, last, None)]
    closed = episode.gripper < 0
    switches = [i for i in range(1, len(closed)) if closed[i] != closed[i - 1]]
    bounds = [0, *switches, last]
    return [
        (start, end, bool(closed[end]) if end in switches else None)
        for start, end in pairwise(bounds)
    ]


async def move_to_start(motion_groups: list[MotionGroup], episode: Episode) -> None:
    """Joint move of every arm to the episode's first frame."""

    async def move(mg: MotionGroup, joints: tuple[float, ...]) -> None:
        tcp = await mg.active_tcp_name() or (await mg.tcp_names())[0]
        actions = [joint_ptp(joints, settings=START_MOVE)]
        await mg.execute(await mg.plan(actions, tcp), tcp, actions=actions)

    await asyncio.gather(
        *(move(mg, tuple(arm_joints(episode, 0, i))) for i, mg in enumerate(motion_groups))
    )


def arm_joints(episode: Episode, frame: int, arm: int) -> list[float]:
    return list(episode.joints[frame, arm * JOINTS_PER_ARM : (arm + 1) * JOINTS_PER_ARM])


async def replay_segment(
    motion_groups: list[MotionGroup], episode: Episode, start: int, end: int, speed: float
) -> None:
    """Replay frames ``start..end`` time-faithfully at ``speed`` x the recorded rate."""
    schema = PolicySchema(
        observations=[
            Observation.joint_positions(f"arm_{i}", source=mg) for i, mg in enumerate(motion_groups)
        ]
    )
    dt_ms = 1000.0 / (episode.fps * speed)
    t0: float | None = None

    async def frames(_obs: dict[str, Any]) -> ActionChunk:
        nonlocal t0
        t0 = time.monotonic() if t0 is None else t0
        frame = min(start + int((time.monotonic() - t0) * 1000.0 / dt_ms), end)
        stop = min(frame + CHUNK_SIZE, end + 1)
        return ActionChunk(
            joints={
                mg.id: [arm_joints(episode, f, i) for f in range(frame, stop)]
                for i, mg in enumerate(motion_groups)
            },
            dt_ms=dt_ms,
        )

    executor = PolicyExecutor(
        schema,
        CallbackPolicyClient(frames),
        timeout_s=(end - start) * dt_ms / 1000.0 + 1.0,
        execution=ContinuousExecution(rate_hz=20),
        motion=WaypointConfig(state_rate_ms=10),
    )
    await executor.run()


async def replay(
    motion_groups: list[MotionGroup],
    episode: Episode,
    *,
    speed: float = 1.0,
    on_gripper: Callable[[bool], Awaitable[None]] | None = None,
    stop_at_grip: bool = False,
) -> None:
    """Move to the start, then replay segment by segment, with gripper actions in between.

    ``stop_at_grip`` ends the replay at the first grip, before the gripper closes (to place
    the object between open jaws).
    """
    await move_to_start(motion_groups, episode)
    for start, end, close in segments(episode):
        print(f"Replaying frames {start}-{end} at {speed:g}x...")
        await replay_segment(motion_groups, episode, start, end, speed)
        if close is True and stop_at_grip:
            print("Stopped at the recorded grasp, jaws open.")
            return
        if close is not None:
            print(f"Gripper {'close' if close else 'open'} (frame {end})")
            if on_gripper is not None:
                await on_gripper(close)
    print("Replay done.")


def describe(episode: Episode, speed: float) -> None:
    print(
        f"Episode: {len(episode.joints)} frames at {episode.fps:.4g} Hz, "
        f"{episode.duration_s:.1f} s recorded, {episode.duration_s / speed:.1f} s at {speed:g}x"
    )
    for start, end, close in segments(episode):
        after = {True: "grip", False: "release", None: "end"}[close]
        print(f"  frames {start:3d}-{end:3d}, then {after}")


class IoGripper:
    """A gripper on a controller digital output: True closes."""

    def __init__(self, controller: Controller, io: str) -> None:
        self._controller, self._io = controller, io

    async def start(self) -> None:
        await self._controller.write(self._io, False)

    async def __call__(self, close: bool) -> None:
        await self._controller.write(self._io, close)

    def close(self) -> None:
        pass


class ZimmerGripper:
    """A Zimmer GEH/GED gripper on a Turck IO-Link master (``zimmer-gripper-controller``)."""

    def __init__(self, host: str) -> None:
        self._host = host
        self._session: Any = None

    async def start(self) -> None:
        """Connect, home and open (moves the jaws)."""
        zimmer = importlib.import_module("zimmer_gripper_controller")  # optional package
        config = zimmer.SessionConfig(modbus=zimmer.ModbusConfig(host=self._host))
        self._session = zimmer.GripperSession(config)
        self._session.connect()
        await asyncio.to_thread(self._session.wait_until_ready)
        self._session.set_force_percent(ZIMMER_FORCE_PERCENT)
        self._session.set_velocity_percent(ZIMMER_VELOCITY_PERCENT)

    async def __call__(self, close: bool) -> None:
        if close:
            result = await asyncio.to_thread(self._session.grip)
            print(f"Grip: holding={result.holding} gap={result.jaw_gap_m * 1000:.1f} mm")
        else:
            await asyncio.to_thread(self._session.open_and_wait)

    def close(self) -> None:
        if self._session is not None:
            self._session.close()


def make_gripper(spec: str, controller: Controller) -> IoGripper | ZimmerGripper:
    """``io:<name>`` or ``zimmer:<host>``."""
    kind, _, target = spec.partition(":")
    if kind == "io" and target:
        return IoGripper(controller, target)
    if kind == "zimmer" and target:
        return ZimmerGripper(target)
    msg = f"--gripper must be io:<name> or zimmer:<host>, got {spec!r}"
    raise ValueError(msg)


async def lift_if_low(mg: MotionGroup, tcp: str, z_mm: float) -> None:
    """Below ``z_mm``, move straight up to it, so the joint move to the start cannot sweep
    the tool over the table or a workpiece."""
    pose = (await mg.get_state(tcp)).pose
    if pose.position.z >= z_mm:
        return
    print(f"Lifting straight up from z {pose.position.z:.0f} to {z_mm:.0f} mm...")
    up = Pose((pose.position.x, pose.position.y, z_mm, *pose.orientation))
    actions = [linear(up, settings=START_MOVE)]
    await mg.execute(await mg.plan(actions, tcp), tcp, actions=actions)


@nova.program(
    id="replay_episode",
    name="Replay Dataset Episode",
    description="Replays the bundled two-arm sample on two virtual UR5e via waypoint jogging.",
    viewer=nova.viewers.Rerun(),
    preconditions=ProgramPreconditions(
        controllers=[
            virtual_controller(
                name=name,
                manufacturer=nova.api.models.Manufacturer.UNIVERSALROBOTS,
                type="universalrobots-ur5e",
            )
            for name in ("ur5e-left", "ur5e-right")
        ],
        cleanup_controllers=False,
    ),
)
async def replay_sample(ctx: nova.ProgramContext) -> None:
    episode = load_episode(SAMPLE, joints_from="action", arms=2)
    describe(episode, 1.0)
    cell = ctx.nova.cell()
    groups = [(await cell.controller(name))[0] for name in ("ur5e-left", "ur5e-right")]
    await replay(groups, episode)


async def start_and_replay(
    groups: list[MotionGroup],
    episode: Episode,
    gripper: IoGripper | ZimmerGripper | None,
    args: argparse.Namespace,
) -> None:
    """Start the gripper (homes and opens it), lift if low, replay."""
    if gripper is not None:
        await gripper.start()
    if args.lift_to_mm is not None:
        for mg in groups:
            await lift_if_low(mg, args.tcp, args.lift_to_mm)
    await replay(
        groups, episode, speed=args.speed, on_gripper=gripper, stop_at_grip=args.stop_at_grip
    )


async def replay_real(args: argparse.Namespace) -> None:
    episode = load_episode(
        args.episode,
        joints_from=args.joints_from,
        arms=len(args.controller),
        gripper_column=args.gripper_column,
    )
    describe(episode, args.speed)
    async with nova.Nova() as client:
        controllers = [await client.cell().controller(name) for name in args.controller]
        groups = [c[0] for c in controllers]
        gripper = make_gripper(args.gripper, controllers[0]) if args.gripper else None
        if not args.execute:
            print("Dry run: nothing moved. Pass --execute to replay.")
            return
        what = "up to the first grip" if args.stop_at_grip else "the episode"
        answer = await asyncio.to_thread(
            input,
            f"{'Start the gripper, then r' if gripper else 'R'}eplay {what} on "
            f"{', '.join(args.controller)} at {args.speed:g}x? Type 'yes': ",
        )
        if answer.strip().lower() != "yes":
            print("Aborted, nothing moved.")
            return
        try:
            await start_and_replay(groups, episode, gripper, args)
        finally:
            if gripper is not None:
                gripper.close()


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--episode", type=Path, help="episode parquet (default: bundled sample)")
    parser.add_argument("--controller", action="append", help="real controller, one per arm")
    parser.add_argument("--joints-from", choices=["action", "state"], default="action")
    parser.add_argument("--gripper-column", type=int, help="action column of a binary gripper")
    parser.add_argument("--gripper", help="io:<name> or zimmer:<host>: the gripper to drive")
    parser.add_argument("--tcp", help="TCP for --lift-to-mm")
    parser.add_argument("--lift-to-mm", type=float, help="lift straight up to this z first")
    parser.add_argument("--speed", type=float, default=1.0, help="x the recorded rate")
    parser.add_argument("--stop-at-grip", action="store_true", help="stop before the first grip")
    parser.add_argument("--execute", action="store_true", help="real robots: move (asks first)")
    parsed = parser.parse_args()
    if parsed.controller:
        if parsed.episode is None:
            parser.error("--episode is required with --controller")
        if parsed.lift_to_mm is not None and not parsed.tcp:
            parser.error("--lift-to-mm needs --tcp")
        if parsed.gripper and parsed.gripper_column is None:
            parser.error("--gripper needs --gripper-column")
        asyncio.run(replay_real(parsed))
    else:
        nova.run_program(replay_sample)
