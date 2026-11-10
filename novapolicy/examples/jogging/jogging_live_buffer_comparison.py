"""Live joint jogging on a virtual UR10e: the live buffer vs. delay_until_braking.

Oscillates joint 0 with each variant, a few rounds in turn, and prints how well the
robot followed:

* ``buffer``: the 500 ms live buffer replays recent targets as a horizon (the workaround).
* ``lone``: each target is sent alone; the robot brakes towards every one of them.
* ``lone+braking``: each target is sent alone with ``delay_until_braking_ms``, so the
  server keeps the velocity until the next target arrives.

If ``lone+braking`` follows as smoothly as ``buffer`` with less lag, the buffer can go.
In Rerun, compare the commanded and measured joint 0 of the runs one after another.

Run against a NOVA instance (26.8.0-112 or newer for delay_until_braking):

    NOVA_API=http://172.31.13.175 PYTHONPATH=. uv run python \
        novapolicy/examples/jogging/jogging_live_buffer_comparison.py
"""

import asyncio
import math
import statistics
import time

import numpy as np

import nova
from nova.cell import virtual_controller
from novapolicy import jog_joints
from novapolicy.types import WaypointConfig

CONTROLLER = "ur10e-live-buffer"
HOME = [1.169, -0.733, 1.745, -3.054, 0.872, 2.094]
DURATION_S, AMPLITUDE, FREQUENCY = 6.0, 0.3, 0.5
ROUNDS = 3

VARIANTS = {
    "buffer": {"buffer_window_ms": 500.0, "config": WaypointConfig()},
    "lone": {"buffer_window_ms": 0.0, "config": WaypointConfig()},
    "lone+braking": {
        "buffer_window_ms": 0.0,
        "config": WaypointConfig(delay_until_braking_ms=200, min_chunk_horizon_ms=0),
    },
}


async def run_variant(mg, kwargs: dict) -> dict:
    """Jog one oscillation from HOME and measure how joint 0 followed it."""
    targets: list[tuple[float, float]] = []
    measured: list[tuple[float, float]] = []
    done = asyncio.Event()

    async def record() -> None:
        async for state in mg.stream_state(response_rate_msecs=10):
            measured.append((time.monotonic(), state.joint_position[0]))
            if done.is_set():
                return

    recorder = asyncio.create_task(record())
    async with jog_joints(mg, start_joint_position=HOME, **kwargs) as jogger:
        start = time.monotonic()
        async for _ in jogger:
            t = jogger.elapsed
            if t >= DURATION_S:
                break
            target = list(HOME)
            target[0] += AMPLITUDE * math.sin(2 * math.pi * FREQUENCY * t)
            jogger.set_target(target)
            targets.append((time.monotonic(), target[0]))
    await asyncio.sleep(1.0)  # let the recorder see the robot settle
    done.set()
    await asyncio.wait_for(recorder, 5)
    return evaluate(np.array(targets), np.array(measured), start)


def evaluate(targets: np.ndarray, measured: np.ndarray, start: float) -> dict:
    """Lag, tracking error and halts of joint 0, skipping the first second."""
    grid = np.arange(start + 1.0, start + DURATION_S - 0.2, 0.01)
    commanded = np.interp(grid, targets[:, 0], targets[:, 1])
    actual = np.interp(grid, measured[:, 0], measured[:, 1])
    errors = [
        np.sqrt(np.mean((actual[lag:] - commanded[: len(commanded) - lag]) ** 2))
        for lag in range(120)
    ]
    lag = int(np.argmin(errors))
    # Halts: the robot stands while the target it is following (lag ago) was moving.
    velocity = np.gradient(actual, 0.01)[lag:]
    moving = np.abs(np.gradient(commanded, 0.01))[: len(commanded) - lag] > 0.3
    return {
        "lag_ms": lag * 10,
        "rms_error_mrad": float(errors[lag]) * 1000,
        "halted_pct": 100 * float(np.sum((np.abs(velocity) < 0.05) & moving)) / moving.sum(),
    }


@nova.program(
    id="jogging_live_buffer_comparison",
    name="Live Buffer vs. Delay Until Braking",
    viewer=nova.viewers.Rerun(),
    preconditions=nova.ProgramPreconditions(
        controllers=[
            virtual_controller(
                name=CONTROLLER,
                manufacturer=nova.api.models.Manufacturer.UNIVERSALROBOTS,
                type="universalrobots-ur10e",
            )
        ],
        cleanup_controllers=False,
    ),
)
async def main(ctx: nova.ProgramContext):
    mg = (await ctx.nova.cell().controller(CONTROLLER))[0]
    results: dict[str, list[dict]] = {name: [] for name in VARIANTS}
    for round_ in range(1, ROUNDS + 1):
        for name, kwargs in VARIANTS.items():
            result = await run_variant(mg, kwargs)
            results[name].append(result)
            print(f"round {round_} {name:<13} {result}", flush=True)

    print(f"\nmedian of {ROUNDS} rounds   lag    rms error   halted")
    for name, runs in results.items():
        lag, error, halted = (
            statistics.median(run[key] for run in runs)
            for key in ("lag_ms", "rms_error_mrad", "halted_pct")
        )
        print(f"{name:<23} {lag:>4.0f}ms  {error:>6.2f} mrad  {halted:>5.1f}%")


if __name__ == "__main__":
    nova.run_program(main)
