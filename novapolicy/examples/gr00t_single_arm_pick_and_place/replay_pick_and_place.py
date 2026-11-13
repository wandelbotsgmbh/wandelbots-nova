"""Replay a training episode of the GR00T cube place on ``ur10e-calibration``, with the
Zimmer gripper gripping and releasing at the recorded frames.

A thin wrapper around the generic ``examples/replay/replay_episode.py``: the episode's
joints come from ``observation.state`` (its actions are end-effector deltas), the gripper
command from ``action[6]``. The training scene's base frame is NOVA's, so the recorded
joints replay the simulated path in the real cell; a replay that picks and places
confirms the cell matches the training scene.

Modes:
    (default)      dry run: load and segment the episode; nothing moves
    --show-grasp   replay up to the recorded grasp and stop with the jaws open, so the
                   cube can be placed exactly between them (asks first)
    --execute      replay the whole episode (asks first)

See README.md next to this file for how to start it.
"""

from __future__ import annotations

import argparse
import asyncio
import importlib.util
from pathlib import Path
import sys
from typing import Any

from gr00t_ur10e import CONTROLLER, lift_if_low, open_gripper_session

from nova import Nova


def _load_generic_replay() -> Any:
    """The generic replay next door, ``examples/replay/replay_episode.py``."""
    path = Path(__file__).resolve().parents[1] / "replay" / "replay_episode.py"
    spec = importlib.util.spec_from_file_location("generic_replay_episode", path)
    if spec is None or spec.loader is None:
        msg = f"cannot load {path}"
        raise ImportError(msg)
    module = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = module  # dataclasses look their module up while it loads
    spec.loader.exec_module(module)
    return module


generic = _load_generic_replay()

GRIPPER_COLUMN = 6  # action = [eef_delta (6), gripper (1)]
# The recording moved at a median 229 mm/s (sim time); 0.4x is ~90 mm/s on the real arm.
SPEED = 0.4


async def main(args: argparse.Namespace) -> None:
    episode = generic.load_episode(args.episode, joints_from="state", gripper_column=GRIPPER_COLUMN)
    generic.describe(episode, args.speed)
    if not (args.show_grasp or args.execute):
        print("Dry run: nothing moved.")
        return
    what = "up to the recorded grasp" if args.show_grasp else "the whole episode"
    answer = await asyncio.to_thread(
        input,
        f"This homes and opens the gripper, then replays {what} on {CONTROLLER} at "
        f"{args.speed:g}x. Type 'yes': ",
    )
    if answer.strip().lower() != "yes":
        print("Aborted, nothing moved.")
        return

    gripper: Any = await asyncio.to_thread(open_gripper_session)  # homes, then opens
    try:

        async def on_gripper(close: bool) -> None:
            if close:
                result = await asyncio.to_thread(gripper.grip)
                print(f"Grip: holding={result.holding} gap={result.jaw_gap_m * 1000:.1f} mm")
            else:
                await asyncio.to_thread(gripper.open_and_wait)

        async with Nova() as client:  # NOVA_API from the environment
            mg = (await client.cell().controller(CONTROLLER))[0]
            await lift_if_low(mg)
            await generic.replay(
                [mg],
                episode,
                speed=args.speed,
                on_gripper=on_gripper,
                stop_at_grip=args.show_grasp,
            )
    finally:
        gripper.close()


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--episode", type=Path, required=True, help="training episode parquet")
    parser.add_argument("--speed", type=float, default=SPEED, help="x the recorded rate")
    mode = parser.add_mutually_exclusive_group()
    mode.add_argument("--show-grasp", action="store_true", help="replay up to the grasp, stop")
    mode.add_argument("--execute", action="store_true", help="replay the whole episode")
    asyncio.run(main(parser.parse_args()))
