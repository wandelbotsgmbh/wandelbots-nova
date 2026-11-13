"""Replay a GR00T training episode (IsaacLab cube place) on the real ``ur10e-calibration``.

The training scene's robot base frame is NOVA's base frame (forward kinematics of the
recorded home joints gives NOVA's live ``umi_corrected`` pose), so the recorded joint
angles replay the simulated path in the real cell. The replay is path-faithful, not
time-faithful: each segment between gripper switches is planned as one blended joint
path at ``TCP_VELOCITY_MM_S``, and the Zimmer gripper grips/opens with the arm at rest.

Modes:
    (default)      dry run: load, segment and plan every segment; nothing moves
    --show-grasp   move to the recorded grasp pose with the jaws open and stop there,
                   so the cube can be placed exactly between them (asks first)
    --execute      replay the episode (asks first)

See README.md next to this file for how to start it.
"""

from __future__ import annotations

import argparse
import asyncio
from itertools import pairwise
from pathlib import Path
from typing import Any

import numpy as np
import pyarrow.parquet as pq

from nova import Nova, api
from nova.actions import joint_ptp, linear
from nova.cell.motion_group import MotionGroup
from nova.types import MotionSettings, Pose

CONTROLLER = "ur10e-calibration"
TCP = "umi_corrected"
GRIPPER_HOST = "172.31.13.49"
GRIPPER_FORCE_PERCENT = 50  # also the closing speed in Zimmer force mode
GRIPPER_VELOCITY_PERCENT = 100

TCP_VELOCITY_MM_S = 100.0  # the recording moved at ~240 mm/s (median) in sim time
SUBSAMPLE = 5  # every 5th recorded frame (6 Hz) as a joint waypoint, blended
SAFE_Z_MM = 200.0  # below this, lift straight up before any joint move
MOTION = MotionSettings(
    tcp_velocity_limit=TCP_VELOCITY_MM_S,
    blending=api.models.BlendingAuto(min_velocity_in_percent=50),
)
LIFT = MotionSettings(tcp_velocity_limit=TCP_VELOCITY_MM_S)


def load_episode(path: Path) -> tuple[np.ndarray, np.ndarray]:
    """Recorded arm joints (N, 6) and gripper commands (N,) of one episode."""
    table = pq.read_table(path, columns=["observation.state", "action"])
    state = np.asarray(table.column("observation.state").to_pylist(), dtype=float)
    action = np.asarray(table.column("action").to_pylist(), dtype=float)
    return state[:, :6], action[:, 6]


def segments(gripper: np.ndarray) -> list[tuple[int, int, bool | None]]:
    """Frame ranges between gripper switches: (start, end, close_after) per segment.

    ``close_after`` is True for a grip, False for a release, None after the last.
    """
    closed = gripper < 0  # BinaryJointPositionAction: < 0 closes
    switches = [i for i in range(1, len(closed)) if closed[i] != closed[i - 1]]
    bounds = [0, *switches, len(closed) - 1]
    return [
        (start, end, bool(closed[end]) if end in switches else None)
        for start, end in pairwise(bounds)
    ]


def waypoints(joints: np.ndarray, start: int, end: int) -> list[tuple[float, ...]]:
    frames = [*range(start, end, SUBSAMPLE), end]
    return [tuple(float(q) for q in joints[i]) for i in dict.fromkeys(frames)]


async def lift_if_low(mg: MotionGroup) -> None:
    pose = (await mg.get_state(TCP)).pose
    if pose.position.z < SAFE_Z_MM:
        print(f"Lifting straight up from z {pose.position.z:.0f} to {SAFE_Z_MM:.0f} mm...")
        up = Pose((pose.position.x, pose.position.y, SAFE_Z_MM, *pose.orientation))
        actions = [linear(up, settings=LIFT)]
        await mg.execute(await mg.plan(actions, TCP), TCP, actions=actions)


async def run_path(mg: MotionGroup, path: list[tuple[float, ...]], *, execute: bool) -> None:
    actions = [joint_ptp(q, settings=MOTION) for q in path]
    trajectory = await mg.plan(actions, TCP)
    if execute:
        await mg.execute(trajectory, TCP, actions=actions)


async def confirm(question: str) -> bool:
    answer = await asyncio.to_thread(input, f"{question} Type 'yes': ")
    return answer.strip().lower() == "yes"


async def plan_only(mg: MotionGroup, joints: np.ndarray, segs: list) -> None:
    for start, end, close in segs:
        path = waypoints(joints, start, end)
        await run_path(mg, [tuple(joints[start]), *path], execute=False)
        after = {True: "grip", False: "open", None: "end"}[close]
        print(f"  frames {start:3d}-{end:3d}: {len(path)} waypoints planned, then {after}")
    print("Dry run: every segment plans; nothing moved.")


async def show_grasp(mg: MotionGroup, joints: np.ndarray, grasp: int | None) -> None:
    if grasp is None:
        print("The episode never grips; nothing to show.")
        return
    await run_path(mg, waypoints(joints, 0, grasp), execute=True)
    tcp = (await mg.get_state(TCP)).pose.position
    print(
        f"At the recorded grasp pose, TCP ({tcp.x:.1f}, {tcp.y:.1f}, {tcp.z:.1f}) mm. "
        "Place the cube centred between the open jaws."
    )


async def replay(mg: MotionGroup, joints: np.ndarray, segs: list, gripper: Any) -> None:
    for start, end, close in segs:
        print(f"Replaying frames {start}-{end}...")
        await run_path(mg, waypoints(joints, start, end), execute=True)
        if close is True:
            result = await asyncio.to_thread(gripper.grip)
            print(f"Grip: holding={result.holding} gap={result.jaw_gap_m * 1000:.1f} mm")
        elif close is False:
            await asyncio.to_thread(gripper.open_and_wait)
            print("Released")
    print("Replay done.")


async def main(args: argparse.Namespace) -> None:
    joints, gripper_cmd = load_episode(args.episode)
    segs = segments(gripper_cmd)
    grasp = next((end for _start, end, close in segs if close), None)
    print(f"Episode: {len(joints)} frames, segments {segs}, grasp at frame {grasp}")

    nova = Nova()  # NOVA_API from the environment
    gripper: Any = None
    try:
        mg = (await nova.cell().controller(CONTROLLER))[0]
        async with mg:
            if not (args.show_grasp or args.execute):
                await plan_only(mg, joints, segs)
                return
            what = "move to the recorded grasp pose" if args.show_grasp else "replay the episode"
            if not await confirm(
                f"This homes and opens the gripper, then makes {CONTROLLER} {what} "
                f"at {TCP_VELOCITY_MM_S:.0f} mm/s."
            ):
                print("Aborted, nothing moved.")
                return

            from zimmer_gripper_controller import (  # ty: ignore[unresolved-import]
                GripperSession,
                ModbusConfig,
                SessionConfig,
            )

            gripper = GripperSession(SessionConfig(modbus=ModbusConfig(host=GRIPPER_HOST)))
            gripper.connect()
            await asyncio.to_thread(gripper.wait_until_ready)  # homes, then opens
            gripper.set_force_percent(GRIPPER_FORCE_PERCENT)
            gripper.set_velocity_percent(GRIPPER_VELOCITY_PERCENT)

            await lift_if_low(mg)
            print("Moving to the episode's start joints...")
            await run_path(mg, [tuple(joints[0])], execute=True)
            if args.show_grasp:
                await show_grasp(mg, joints, grasp)
            else:
                await replay(mg, joints, segs, gripper)
    finally:
        if gripper is not None:
            gripper.close()
        await nova.close()


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--episode", type=Path, required=True, help="LeRobot episode parquet")
    mode = parser.add_mutually_exclusive_group()
    mode.add_argument("--show-grasp", action="store_true", help="go to the grasp pose and stop")
    mode.add_argument("--execute", action="store_true", help="replay the episode on the robot")
    asyncio.run(main(parser.parse_args()))
