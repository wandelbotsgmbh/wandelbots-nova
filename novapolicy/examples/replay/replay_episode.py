"""
Replay a recorded dataset episode on two robots.

Reads a LeRobot-format parquet file and replays each action as joint targets
through waypoint jogging. The simplest possible replay — no error tracking, no
multi-episode loop, just load → home → play.

A trimmed sample episode (``action`` + ``timestamp`` columns only) sits next to
this script; override with the ``DATASET`` env var to point at a full LeRobot
dataset directory.

Prerequisites:
    NOVA_API=http://<instance-ip>
    pip install pyarrow

Usage:
    NOVA_API=http://172.31.12.76 PYTHONPATH=. python novapolicy/examples/replay/replay_episode.py
"""

from __future__ import annotations

import asyncio
import os
from pathlib import Path
import time
from typing import Any

import pyarrow.parquet as pq

import nova
from nova.actions import joint_ptp
from nova.cell import virtual_controller
from nova.program import ProgramPreconditions
from nova.types import MotionSettings
from novapolicy import ActionChunk, Observation, PolicyExecutor, PolicySchema, WaypointConfig

# Recording rate of the bundled sample (LeRobot info.json "fps").
FPS = 15.0
EPISODE = int(os.environ.get("EPISODE", "0"))
CHUNK_SIZE = 8


@nova.program(
    id="replay_episode",
    name="Replay Dataset Episode",
    description="Replays a recorded parquet episode on two UR5e robots via waypoint jogging.",
    viewer=nova.viewers.Rerun(),
    preconditions=ProgramPreconditions(
        controllers=[
            virtual_controller(
                name="ur5e-left",
                manufacturer=nova.api.models.Manufacturer.UNIVERSALROBOTS,
                type="universalrobots-ur5e",
            ),
            virtual_controller(
                name="ur5e-right",
                manufacturer=nova.api.models.Manufacturer.UNIVERSALROBOTS,
                type="universalrobots-ur5e",
            ),
        ],
        cleanup_controllers=False,
    ),
)
async def replay_episode(ctx: nova.ProgramContext):
    # Load episode actions. The bundled sample is trimmed to action+timestamp;
    # a single episode parquet sits next to this script (no chunk dirs).
    dataset = os.environ.get("DATASET")
    if dataset:
        path = Path(dataset) / "data" / "chunk-000" / f"episode_{EPISODE:06d}.parquet"
    else:
        path = Path(__file__).parent / f"episode_{EPISODE:06d}.parquet"
    dt_ms = 1000.0 / FPS

    table = pq.read_table(path, columns=["action", "timestamp"])
    actions = table.column("action").to_pylist()
    timestamps_s = table.column("timestamp").to_pylist()
    duration_s = timestamps_s[-1] - timestamps_s[0]
    print(f"Loaded episode {EPISODE}: {len(actions)} steps, {duration_s:.1f}s duration")

    # Connect to controllers
    cell = ctx.nova.cell()
    mg_left = (await cell.controller("ur5e-left"))[0]
    mg_right = (await cell.controller("ur5e-right"))[0]

    # Move to start position
    start_left, start_right = tuple(actions[0][:6]), tuple(actions[0][6:])
    fast = MotionSettings(tcp_velocity_limit=500.0)
    tcp_l = (await mg_left.tcp_names())[0]
    tcp_r = (await mg_right.tcp_names())[0]
    t1 = await mg_left.plan([joint_ptp(start_left, settings=fast)], tcp_l)
    t2 = await mg_right.plan([joint_ptp(start_right, settings=fast)], tcp_r)
    await asyncio.gather(
        mg_left.execute(t1, tcp_l, actions=[joint_ptp(start_left, settings=fast)]),
        mg_right.execute(t2, tcp_r, actions=[joint_ptp(start_right, settings=fast)]),
    )
    print("At start position")

    # Define schema
    schema = PolicySchema(
        observations=[
            Observation.joint_positions("left_joints", source=mg_left),
            Observation.joint_positions("right_joints", source=mg_right),
        ]
    )

    # Replay policy: index into actions based on elapsed time
    replay_start: float | None = None

    async def replay(obs: dict[str, Any]) -> ActionChunk:
        nonlocal replay_start
        if replay_start is None:
            replay_start = time.monotonic()
        elapsed_s = time.monotonic() - replay_start

        # Find which step corresponds to current elapsed time
        step = 0
        for i, ts in enumerate(timestamps_s):
            if ts - timestamps_s[0] <= elapsed_s:
                step = i
            else:
                break
        step = min(step, len(actions) - 1)

        chunk_end = min(step + CHUNK_SIZE, len(actions))
        chunk = actions[step:chunk_end] if step < len(actions) else [actions[-1]]

        # Use actual recording timestamps for dt between steps
        first_timestamp_ms = int((timestamps_s[step] - timestamps_s[0]) * 1000)

        return ActionChunk(
            joints={
                mg_left.id: [a[:6] for a in chunk],
                mg_right.id: [a[6:] for a in chunk],
            },
            dt_ms=dt_ms,
            first_timestamp_ms=first_timestamp_ms,
        )

    # Run
    executor = PolicyExecutor(
        schema,
        replay,
        timeout_s=duration_s + 5.0,
        policy_rate_hz=20,
        motion=WaypointConfig(state_rate_ms=10),
    )
    print(f"Replaying {len(actions)} steps ({duration_s:.1f}s)...")
    result = await executor.run()
    print(f"Done: {result.steps} steps in {result.duration_s:.1f}s ({result.reason})")


if __name__ == "__main__":
    nova.run_program(replay_episode)
