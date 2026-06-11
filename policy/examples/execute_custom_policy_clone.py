"""Single-arm custom policy on the virtual ur10e-clone.

A self-contained mock policy that emits overlapping 16-step action chunks
(RTC-style) around the robot's current home pose, executed through the
PolicyExecutor / waypoint-jogging path. Targets the virtual ur10e-clone so it
can be dry-run before pointing it at the real ur10e.

Run:
    NOVA_API=http://wandelbox-hhmnwy PYTHONPATH=. python \
        policy/examples/execute_custom_policy_clone.py
"""

import math
import time
from typing import Any

import nova
from nova import ProgramContext, api, run_program, viewers
from nova.actions import jnt
from nova.types import MotionSettings
from policy import (
    Action,
    ActionChunk,
    EmergencyStopError,
    MotionError,
    Observation,
    PolicyExecutor,
    PolicySchema,
)

# Current pose of the real ur10e / clone (read from get_state on 2026-06-11).
HOME = [0.3202, -1.8691, 1.9472, -1.6528, -1.5776, 1.8531]

CONTROLLER = "ur10e"  # the real robot (use "ur10e-clone" for the virtual dry-run)
MG_ID = f"0@{CONTROLLER}"

_start_time: float | None = None
_action_count = 0


async def elapsed(obs: dict[str, Any]) -> dict[str, Any]:
    """Computed observation: real elapsed seconds since the first step."""
    global _start_time
    if _start_time is None:
        _start_time = time.monotonic()
    return {"elapsed_s": time.monotonic() - _start_time}


async def log_action(action: dict[str, Any]) -> None:
    """Computed action: print every 100th action."""
    global _action_count
    _action_count += 1
    if _action_count % 100 == 0:
        print(f"  [action logger] step={_action_count}")


async def mock_policy(obs: dict[str, Any]) -> ActionChunk:
    """Emit a 16-step sinusoidal chunk around HOME (trajectory-absolute timestamps).

    Each joint follows ``amplitude * sin(2*pi*freq*t)`` with no phase offset, so
    at ``t=0`` every joint is exactly at HOME and the motion grows smoothly from
    there (no initial jump away from the start pose).
    """
    t0 = obs.get("elapsed_s", 0.0)
    steps: list[list[float]] = []
    for step_i in range(16):
        t = t0 + step_i * 0.05  # 50ms per step
        joints = list(HOME)
        for i in range(6):
            # Speed limit: peak joint velocity = amplitude * 2*pi * freq.
            # Worst case here ~= 0.15 * 2*pi * 0.32 ~= 0.30 rad/s (~17 deg/s).
            amplitude = 0.15 if i in (0, 1, 2) else 0.08
            freq = 0.2 + (i + 1) * 0.03
            joints[i] = HOME[i] + amplitude * math.sin(2 * math.pi * freq * t)
        steps.append(joints)
    return ActionChunk(
        joints={MG_ID: steps}, dt_ms=50.0, first_timestamp_ms=int(t0 * 1000)
    )


@nova.program(
    id="custom_policy_clone",
    name="Single-Arm Custom Policy (clone)",
    viewer=viewers.Rerun(),
)
async def main(ctx: ProgramContext):
    cell = ctx.nova.cell()
    mg = (await cell.controller(CONTROLLER))[0]
    tcp = (await mg.tcp_names())[0]

    # PTP to HOME (the current pose). Collision setups are cleared so the cell's
    # safety planes don't reject planning at this exact pose. A slow TCP velocity
    # limit keeps the approach gentle.
    print("Moving to home...")
    settings = MotionSettings(tcp_velocity_limit=50.0)  # mm/s
    setup = await mg.get_setup(tcp)
    setup.collision_setups = api.models.CollisionSetups({})
    traj = await mg.plan([jnt(HOME, settings=settings)], tcp, motion_group_setup=setup)
    await mg.execute(traj, tcp, actions=[jnt(HOME, settings=settings)])

    schema = PolicySchema(
        observations=[
            Observation.joint_positions("arm", source=mg),
            Observation.computed(elapsed),
        ],
        actions=[Action.computed(log_action)],
    )

    executor = PolicyExecutor(
        schema,
        mock_policy,
        timeout_s=60.0,
        policy_rate_hz=20,
        n_action_steps=8,
    )

    print("Running policy for 60s...")
    try:
        result = await executor.run()
        print(f"Done: reason={result.reason} steps={result.steps}")
    except MotionError as e:
        print(f"Motion error: {e}")
    except EmergencyStopError as e:
        print(f"Emergency stop: {e.controller_id}")


if __name__ == "__main__":
    run_program(main)
