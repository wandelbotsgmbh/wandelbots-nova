"""
Example: Run a mock policy on two UR10e robots with cameras.

Demonstrates:
- FeatureMap for LeRobot-compatible flat feature dicts
- WebRTC cameras — images included in every observation
- Safety guards (workspace, speed, IO sensor)
- Time-based execution (timeout_s)

Prerequisites:
    NOVA_API=http://<instance-ip>
    CAMERA_SERVER=http://localhost:9100

Run:
    PYTHONPATH=. python policy/examples/execute_policy_on_dualarm.py
"""

import asyncio
import math
import os
from typing import Any

import numpy as np  # noqa: F401 — used by policy at runtime
from policy import (
    CameraSet,
    EmergencyStopError,
    FeatureGroup,
    FeatureMap,
    GuardState,
    GuardStopError,
    MotionError,
    PolicyExecutor,
)

from nova import Nova
from nova.actions import joint_ptp
from nova.types import MotionSettings

HOME_LEFT = (0.0, -1.571, 1.571, -1.571, -1.571, 0.0)
HOME_RIGHT = (0.0, -1.571, -1.571, -1.571, 1.571, 0.0)

CAMERA_SERVER = os.environ.get("CAMERA_SERVER", "http://localhost:9100")


# ---------------------------------------------------------------------------
# Policy: obs → actions. Stateless pure function.
# ---------------------------------------------------------------------------


async def mock_policy(obs: dict[str, Any]) -> dict[str, float]:
    """Mock policy — replace with your own inference call.

    In production, you would call a remote API here (HTTP, NATS, ZMQ, etc.)
    passing the full observation dict including camera images as numpy arrays.
    The obs contains joint positions, IO values, and camera frames.

    This mock computes sinusoidal offsets from current joint positions.
    """
    features: dict[str, float] = {}
    for role in ("left", "right"):
        role_phase = 0.0 if role == "left" else math.pi
        all_joints_sum = sum(obs.get(f"{role}_joint_position_{i}", 0.0) for i in range(1, 7))
        for i in range(1, 7):
            key = f"{role}_joint_position_{i}"
            current = obs.get(key, 0.0)
            phase = role_phase + i * 0.7
            features[key] = current + 0.3 * math.sin(all_joints_sum * 3.0 + phase)
        # Gripper: based on shoulder joint sign
        shoulder = obs.get(f"{role}_joint_position_2", 0.0)
        features[f"{role}_gripper"] = 1.0 if shoulder > 0 else 0.0

    return features


# ---------------------------------------------------------------------------
# Safety guards
# ---------------------------------------------------------------------------


def workspace_guard(ctx: GuardState) -> bool:
    """Stop if TCP Z drops below -500mm."""
    return ctx.state.pose.position[2] > -500


def speed_guard(ctx: GuardState) -> bool:
    """Stop if TCP moves faster than 5000mm/s."""
    if ctx.prev_state is None or ctx.dt < 0.005:
        return True
    p0 = ctx.prev_state.pose.position
    p1 = ctx.state.pose.position
    dist = sum((a - b) ** 2 for a, b in zip(p1, p0, strict=False)) ** 0.5
    return dist / ctx.dt < 5000.0


def io_guard(ctx: GuardState) -> bool:
    """Stop when conveyor sensor detects a box (digital_in[0] = True)."""
    if ctx.io_values is None:
        return True
    return ctx.io_values.get("digital_in[0]") is not True


# ---------------------------------------------------------------------------
# Move to home
# ---------------------------------------------------------------------------


async def move_to_home(mg1, mg2) -> None:
    """Move both robots to home concurrently."""
    fast = MotionSettings(tcp_velocity_limit=500.0)
    tcp1, tcp2 = (await mg1.tcp_names())[0], (await mg2.tcp_names())[0]
    t1 = await mg1.plan([joint_ptp(HOME_LEFT, settings=fast)], tcp1)
    t2 = await mg2.plan([joint_ptp(HOME_RIGHT, settings=fast)], tcp2)
    await asyncio.gather(
        mg1.execute(t1, tcp1, actions=[joint_ptp(HOME_LEFT, settings=fast)]),
        mg2.execute(t2, tcp2, actions=[joint_ptp(HOME_RIGHT, settings=fast)]),
    )


# ---------------------------------------------------------------------------
# Main
# ---------------------------------------------------------------------------


async def main() -> None:
    async with Nova() as nova:
        cell = nova.cell()
        mg1 = (await cell.controller("ur10e"))[0]
        mg2 = (await cell.controller("ur10e-2"))[0]

        print("Moving to home...")
        await move_to_home(mg1, mg2)

        feature_map = FeatureMap(groups=[
            FeatureGroup(
                motion_group=mg1, name="left",
                ios={"left_gripper": "digital_out[0]", "left_conveyor_sensor": "digital_in[0]"},
            ),
            FeatureGroup(motion_group=mg2, name="right", ios={"right_gripper": "digital_out[0]"}),
        ])

        cameras = CameraSet(
            api_url=CAMERA_SERVER,
            devices={"flange": "315122271048", "left": "314522065367", "right": "319522063360"},
            width=640,
            height=480,
            fps=15,
        )

        executor = PolicyExecutor(
            feature_map=feature_map,
            policy=mock_policy,
            cameras=cameras,
            safety_guards=[workspace_guard, speed_guard, io_guard],
            timeout_s=10.0,
        )

        print("Running policy for 10s...")
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
    asyncio.run(main())
