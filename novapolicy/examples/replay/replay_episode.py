"""
Replay a recorded dataset episode's joint path on one or more robots.

Reads a LeRobot-format episode parquet (the format GR00T and LeRobot train on) and replays
its joint positions through waypoint jogging, time-faithfully at ``--speed`` times the
recorded rate. If the episode has a binary gripper column, the replay is split at the
gripper switches; between segments the arm stands still and ``on_gripper`` runs.

Without ``--controller`` it replays the bundled sample (``action`` + ``timestamp``
columns, two arms) on two virtual UR5e, as a quick look. With ``--controller`` it drives
real robots: a dry run by default, ``--execute`` asks before anything moves.

Prerequisites:
    NOVA_API=http://<instance-ip>
    uv sync --extra novapolicy   (plus pyarrow)

Usage:
    # Bundled sample on two virtual UR5e
    uv run python novapolicy/examples/replay/replay_episode.py

    # A single real arm whose joints are recorded in observation.state, gripper in action[6]
    uv run python novapolicy/examples/replay/replay_episode.py --episode <episode.parquet> \\
        --controller <controller> --joints-from state --gripper-column 6 --speed 0.5 [--execute]
"""

from __future__ import annotations

import argparse
import asyncio
from dataclasses import dataclass
from itertools import pairwise
from pathlib import Path
import time
from typing import TYPE_CHECKING, Any

import numpy as np
import pyarrow.parquet as pq

import nova
from nova.actions import joint_ptp
from nova.cell import virtual_controller
from nova.program import ProgramPreconditions
from nova.types import MotionSettings
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

    from nova.cell.motion_group import MotionGroup

SAMPLE = Path(__file__).parent / "episode_000000.parquet"
JOINTS_PER_ARM = 6
CHUNK_SIZE = 8
START_MOVE = MotionSettings(tcp_velocity_limit=100.0)


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


async def replay_real(args: argparse.Namespace) -> None:
    episode = load_episode(
        args.episode,
        joints_from=args.joints_from,
        arms=len(args.controller),
        gripper_column=args.gripper_column,
    )
    describe(episode, args.speed)
    async with nova.Nova() as client:
        groups = [(await client.cell().controller(name))[0] for name in args.controller]
        if not args.execute:
            print("Dry run: nothing moved. Pass --execute to replay.")
            return
        answer = await asyncio.to_thread(
            input, f"Replay on {', '.join(args.controller)} at {args.speed:g}x? Type 'yes': "
        )
        if answer.strip().lower() != "yes":
            print("Aborted, nothing moved.")
            return
        await replay(groups, episode, speed=args.speed, stop_at_grip=args.stop_at_grip)


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--episode", type=Path, help="episode parquet (default: bundled sample)")
    parser.add_argument("--controller", action="append", help="real controller, one per arm")
    parser.add_argument("--joints-from", choices=["action", "state"], default="action")
    parser.add_argument("--gripper-column", type=int, help="action column of a binary gripper")
    parser.add_argument("--speed", type=float, default=1.0, help="x the recorded rate")
    parser.add_argument("--stop-at-grip", action="store_true", help="stop before the first grip")
    parser.add_argument("--execute", action="store_true", help="real robots: move (asks first)")
    parsed = parser.parse_args()
    if parsed.controller:
        if parsed.episode is None:
            parser.error("--episode is required with --controller")
        asyncio.run(replay_real(parsed))
    else:
        nova.run_program(replay_sample)
