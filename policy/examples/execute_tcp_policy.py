"""
Example: Run a mock policy that controls the robot in TCP (Cartesian) space.

The policy receives the current TCP pose [x, y, z, rx, ry, rz] in meters/radians
and returns target TCP poses. The executor uses Cartesian PID jogging to track them.

This example draws a circle in the XY plane while keeping Z and orientation
constant.

Run:
    NOVA_API=http://<instance-ip> PYTHONPATH=. uv run python policy/examples/execute_tcp_policy.py
"""

import math
import time
from typing import Any

import nova
from nova import api, run_program
from nova.actions import joint_ptp
from nova.cell import virtual_controller
from nova.program import ProgramPreconditions
from nova.types import MotionSettings
from policy import (
    EmergencyStopError,
    GuardStopError,
    MotionError,
    Observation,
    PolicyExecutor,
    PolicySchema,
)

HOME = (0.0, -1.571, 0.5, -0.5, -1.571, 0.0)
CIRCLE_RADIUS = 20.0  # mm
CIRCLE_PERIOD = 5.0  # seconds per full circle

_start_time: float = 0.0
_center: dict[str, float] = {}


async def tcp_circle_policy(obs: dict[str, Any]) -> dict[str, float]:
    """Policy that traces a circle in the XY plane.

    Receives:  eef_x, eef_y, eef_z, eef_rx, eef_ry, eef_rz in mm / rad
    Returns:   eef_x..eef_rz = target TCP pose (absolute, mm / rad)
    """
    global _start_time, _center

    # Capture the starting TCP as circle center on first call
    if not _center:
        _center = {
            k: obs[f"eef_{k}"] for k in ("x", "y", "z", "rx", "ry", "rz")
        }
        _start_time = time.monotonic()

    t = time.monotonic() - _start_time
    angle = 2.0 * math.pi * t / CIRCLE_PERIOD

    return {
        "eef_x": _center["x"] + CIRCLE_RADIUS * math.cos(angle),
        "eef_y": _center["y"] + CIRCLE_RADIUS * math.sin(angle),
        "eef_z": _center["z"],
        "eef_rx": _center["rx"],
        "eef_ry": _center["ry"],
        "eef_rz": _center["rz"],
    }


@nova.program(
    id="tcp_policy",
    name="TCP Circle Policy",
    preconditions=ProgramPreconditions(
        controllers=[
            virtual_controller(
                name="ur5e-left",
                manufacturer=api.models.Manufacturer.UNIVERSALROBOTS,
                type="universalrobots-ur5e",
            ),
        ],
        cleanup_controllers=False,
    ),
)
async def tcp_policy(ctx: nova.ProgramContext):
    global _center
    _center = {}

    cell = ctx.nova.cell()
    mg = (await cell.controller("ur5e-left"))[0]

    # Move to home
    tcp = (await mg.tcp_names())[0]
    fast = MotionSettings(tcp_velocity_limit=500.0)
    traj = await mg.plan([joint_ptp(HOME, settings=fast)], tcp)
    await mg.execute(traj, tcp, actions=[joint_ptp(HOME, settings=fast)])

    # Schema: observe and control via TCP pose (mm, rotation vector — Nova native)
    schema = PolicySchema(observations=[
        Observation.tcp("eef", source=mg, action=True),
    ])

    executor = PolicyExecutor(schema, tcp_circle_policy, timeout_s=10.0)

    print("Running TCP circle policy for 10s...")
    try:
        result = await executor.run()
        print(f"Done: reason={result.reason} steps={result.steps} duration={result.duration_s:.1f}s")
    except GuardStopError as e:
        print(f"Safety guard triggered: {e.guard_name}")
    except MotionError as e:
        print(f"Motion error: {e}")
    except EmergencyStopError as e:
        print(f"Emergency stop: {e.controller_id}")


if __name__ == "__main__":
    run_program(tcp_policy)
