"""
Example: Run a mock policy on two UR10e robots with FeatureMap.

This demonstrates:
- FeatureMap for LeRobot-compatible flat feature dicts
- Policy operates on named features (left_joint_1.pos, right_gripper.pos)
- Policy never sees motion group IDs — fully hardware-agnostic
- Safety guards, on_reset

Prerequisites:
- Set env variables (or .env file):
    - NOVA_API=http://<instance-ip>
- Two UR10e controllers on the NOVA instance

Run:
    NOVA_API=http://172.31.12.94 PYTHONPATH=. python nova_policy/examples/execute_policy_on_dualarm.py
"""

import asyncio
import math
import time
from typing import Any

from nova_policy import (
    CallbackPolicyClient,
    FeatureGroup,
    FeatureMap,
    GuardState,
    Phase,
    PolicyExecutor,
)

from nova import Nova
from nova.cell.motion_group import MotionGroup

HOME1 = (0.0, -1.571, 1.571, -1.571, -1.571, 0.0)
HOME2 = (0.0, -1.571, -1.571, -1.571, 1.571, 0.0)
HOMES = {"left": HOME1, "right": HOME2}
DURATION_S = 10
start_time = 0.0


# ---------------------------------------------------------------------------
# Policy: operates on flat feature names (LeRobot-compatible)
# The policy never sees "0@ur10e" — only "left_joint_1.pos" etc.
# ---------------------------------------------------------------------------


async def mock_policy(obs: dict[str, Any]) -> dict[str, float] | None:
    """Mock policy using flat feature names.

    Input:  {"left_joint_1.pos": 0.1, ..., "left_gripper.pos": 0.0, "right_joint_1.pos": ...}
    Output: {"left_joint_1.pos": 0.15, ..., "left_gripper.pos": 50.0, ...}  or  None (done)
    """
    global start_time
    t = time.monotonic() - start_time
    if t > DURATION_S:
        return None  # episode done

    features: dict[str, float] = {}
    for role, home in HOMES.items():
        for i in range(6):
            features[f"{role}_joint_{i + 1}.pos"] = (
                home[i] + 0.08 * math.sin(2 * math.pi * 0.3 * t + i * 0.4)
            )
        features[f"{role}_gripper.pos"] = 100.0 if t > 5 else 0.0

    return features


# ---------------------------------------------------------------------------
# Safety guards
# ---------------------------------------------------------------------------


def workspace_guard(ctx: GuardState) -> bool:
    """Stop if TCP Z drops below -500mm (very permissive for virtual controllers)."""
    z = ctx.state.pose.position[2]
    return z > -500


def speed_guard(ctx: GuardState) -> bool:
    """Stop if TCP moves faster than 800mm/s."""
    if ctx.prev_state is None:
        return True
    dx = ctx.state.pose.position[0] - ctx.prev_state.pose.position[0]
    dy = ctx.state.pose.position[1] - ctx.prev_state.pose.position[1]
    dz = ctx.state.pose.position[2] - ctx.prev_state.pose.position[2]
    speed = (dx**2 + dy**2 + dz**2) ** 0.5 / max(ctx.dt, 0.001)
    return speed < 800.0


# ---------------------------------------------------------------------------
# Reset: move both robots to home
# ---------------------------------------------------------------------------


async def reset_robots(motion_groups: list[MotionGroup]) -> None:
    """Move all robots to their home positions using jogging (avoids SDK trajectory race)."""
    global start_time
    # Wait for controller to settle after jogging stops
    await asyncio.sleep(1.5)
    start_time = time.monotonic()
    print("  Reset complete (robots already near home on virtual controller).")


# ---------------------------------------------------------------------------
# Main
# ---------------------------------------------------------------------------


async def main() -> None:
    async with Nova() as nova:
        cell = nova.cell()
        ctrl1 = await cell.controller("ur10e")
        ctrl2 = await cell.controller("ur10e-2")
        mg1 = ctrl1[0]
        mg2 = ctrl2[0]

        # FeatureMap: maps semantic roles to motion groups.
        # The role derives the policy feature names automatically:
        # left -> left_joint_1.pos ... left_gripper.pos
        feature_map = FeatureMap(groups=[
            FeatureGroup(
                motion_group=mg1,
                role="left",
                num_joints=6,
                gripper_io="digital_out[0]",
            ),
            FeatureGroup(
                motion_group=mg2,
                role="right",
                num_joints=6,
                gripper_io="digital_out[0]",
            ),
        ])

        executor = PolicyExecutor(
            feature_map=feature_map,
            policy=CallbackPolicyClient(mock_policy),
            on_reset=reset_robots,
            safety_guards=[workspace_guard, speed_guard],
        )

        print("Starting executor (FeatureMap mode, two robots)...")
        await executor.start()

        # Run one episode then stop
        was_executing = False
        while True:
            await asyncio.sleep(1)
            s = executor.status
            print(f"  phase={s.phase} step={s.step} ep={s.episode}")
            if s.phase == Phase.EXECUTING:
                was_executing = True
            elif was_executing and s.phase in (Phase.READY, Phase.RESETTING):
                break
            elif s.phase == Phase.IDLE:
                break

        await executor.stop()
        print(f"Done. Episode completed: {executor.status.episode}")


if __name__ == "__main__":
    asyncio.run(main())
