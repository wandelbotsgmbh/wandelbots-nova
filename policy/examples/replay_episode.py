"""
Replay a recorded dataset episode on two robots.

Reads a LeRobot-format parquet file and replays each action as joint targets
through PID jogging. The simplest possible replay — no error tracking, no
multi-episode loop, just load → home → play.

Prerequisites:
    NOVA_API=http://<instance-ip>
    pip install pyarrow

Usage:
    NOVA_API=http://172.31.12.76 PYTHONPATH=. python policy/examples/replay_episode.py
"""

from __future__ import annotations

import json
import os
from pathlib import Path
from typing import Any

import pyarrow.parquet as pq
from policy import ActionChunk, Observation, PolicyExecutor, PolicySchema

import nova
from nova import api, run_program
from nova.actions import joint_ptp
from nova.cell import virtual_controller
from nova.program import ProgramPreconditions
from nova.types import MotionSettings

DATASET = Path(os.environ.get(
    "DATASET", "/Users/stefanwagner/Downloads/Data_Recordings_old",
))
EPISODE = int(os.environ.get("EPISODE", "0"))
CHUNK_SIZE = 8


@nova.program(
    id="replay_episode",
    name="Replay Dataset Episode",
    description="Replays a recorded parquet episode on two UR5e robots via PID jogging.",
    preconditions=ProgramPreconditions(
        controllers=[
            virtual_controller(
                name="ur5e-left",
                manufacturer=api.models.Manufacturer.UNIVERSALROBOTS,
                type="universalrobots-ur5e",
            ),
            virtual_controller(
                name="ur5e-right",
                manufacturer=api.models.Manufacturer.UNIVERSALROBOTS,
                type="universalrobots-ur5e",
            ),
        ],
        cleanup_controllers=False,
    ),
)
async def replay_episode(ctx: nova.ProgramContext):
    # Load episode actions and metadata
    with (DATASET / "meta" / "info.json").open() as f:
        meta = json.load(f)
    fps = meta["fps"]
    dt_ms = 1000.0 / fps

    path = DATASET / "data" / "chunk-000" / f"episode_{EPISODE:06d}.parquet"
    actions = pq.read_table(path, columns=["action"]).column("action").to_pylist()
    print(f"Loaded episode {EPISODE}: {len(actions)} steps at {fps} fps ({dt_ms:.1f}ms/step)")

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
    import asyncio
    await asyncio.gather(
        mg_left.execute(t1, tcp_l, actions=[joint_ptp(start_left, settings=fast)]),
        mg_right.execute(t2, tcp_r, actions=[joint_ptp(start_right, settings=fast)]),
    )
    print("At start position")

    # Define schema
    schema = PolicySchema(observations=[
        Observation.joint_positions("left_joints", source=mg_left),
        Observation.joint_positions("right_joints", source=mg_right),
    ])

    # Replay policy: each call returns the next chunk of actions
    step = 0

    async def replay(obs: dict[str, Any]) -> ActionChunk:
        nonlocal step
        chunk_end = min(step + CHUNK_SIZE, len(actions))
        chunk = actions[step:chunk_end] if step < len(actions) else [actions[-1]]
        step += 1
        return ActionChunk(
            joints={
                mg_left.id: [a[:6] for a in chunk],
                mg_right.id: [a[6:] for a in chunk],
            },
            dt_ms=dt_ms,
        )

    # Run
    executor = PolicyExecutor(
        schema,
        replay,
        timeout_s=len(actions) / fps + 5.0,
        inference_hz=fps,
    )
    print(f"Replaying {len(actions)} steps...")
    result = await executor.run()
    print(f"Done: {result.steps} steps in {result.duration_s:.1f}s ({result.reason})")


if __name__ == "__main__":
    run_program(replay_episode)
