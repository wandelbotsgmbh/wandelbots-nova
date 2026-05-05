"""
Example: Run a mock policy on two UR10e robots with FeatureMap.

This demonstrates:
- Move to home before policy execution
- FeatureMap for LeRobot-compatible flat feature dicts
- Policy operates on named features (left_joint_1.pos, right_gripper.pos)
- Policy never sees motion group IDs — fully hardware-agnostic
- Time-based execution (timeout_s)
- Safety guards (workspace, speed, and IO-based sensor trigger)

Prerequisites:
- Set env variables (or .env file):
    - NOVA_API=http://<instance-ip>
- Two UR10e controllers on the NOVA instance

Run:
    NOVA_API=http://172.31.10.42 PYTHONPATH=. python policy/examples/execute_policy_on_dualarm.py
"""

import asyncio
import math
from typing import Any

from policy import (
    CallbackPolicyClient,
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

HOME_LEFT = (0.0, -1.571, 1.571, -1.571, -1.571, 0.0)
HOME_RIGHT = (0.0, -1.571, -1.571, -1.571, 1.571, 0.0)


# ---------------------------------------------------------------------------
# Policy: obs → actions. Pure function, no episode logic.
# ---------------------------------------------------------------------------


async def mock_policy(obs: dict[str, Any]) -> dict[str, float]:
    """Stateless mock policy using flat feature names.

    Computes target joint positions as a small sinusoidal offset from the
    *current* observed positions. The output depends only on the input
    observation — no internal state.

    Input:  {"left_joint_1.pos": 0.1, ..., "left_gripper": 0.0, ...}
    Output: {"left_joint_1.pos": 0.15, ..., "left_gripper": 1.0, ...}
    """
    amplitude = 0.08
    features: dict[str, float] = {}
    for role in ("left", "right"):
        for i in range(6):
            key = f"{role}_joint_{i + 1}.pos"
            current = obs.get(key, 0.0)
            features[key] = current + amplitude * math.sin(current * 5.0 + i * 0.4)
        # Gripper: close if the first joint is positive, open otherwise
        first_joint = obs.get(f"{role}_joint_1.pos", 0.0)
        features[f"{role}_gripper"] = 1.0 if first_joint > 0 else 0.0

    return features


# ---------------------------------------------------------------------------
# Safety guards
# ---------------------------------------------------------------------------


def workspace_guard(ctx: GuardState) -> bool:
    """Stop if TCP Z drops below -500mm."""
    z = ctx.state.pose.position[2]
    return z > -500


def speed_guard(ctx: GuardState) -> bool:
    """Stop if TCP moves faster than 5000mm/s (generous for virtual controllers)."""
    if ctx.prev_state is None or ctx.dt < 0.005:
        return True
    dx = ctx.state.pose.position[0] - ctx.prev_state.pose.position[0]
    dy = ctx.state.pose.position[1] - ctx.prev_state.pose.position[1]
    dz = ctx.state.pose.position[2] - ctx.prev_state.pose.position[2]
    speed = (dx**2 + dy**2 + dz**2) ** 0.5 / ctx.dt
    return speed < 5000.0


def io_guard(ctx: GuardState) -> bool:
    """Stop when the conveyor belt sensor detects a box.

    Reads digital_in[0] from the IO stream cache (no HTTP call).
    Returns False (= stop) when the sensor goes True.
    """
    if ctx.io_values is None:
        return True
    sensor_value = ctx.io_values.get("digital_in[0]")
    if sensor_value is True:
        return False  # Box detected → stop policy
    return True


# ---------------------------------------------------------------------------
# Move to home
# ---------------------------------------------------------------------------


async def move_to_home(mg1, mg2) -> None:
    """Move both robots to their home positions concurrently."""
    tcp1 = (await mg1.tcp_names())[0]
    tcp2 = (await mg2.tcp_names())[0]
    traj1 = await mg1.plan([joint_ptp(HOME_LEFT)], tcp1)
    traj2 = await mg2.plan([joint_ptp(HOME_RIGHT)], tcp2)
    await asyncio.gather(
        mg1.execute(traj1, tcp1, actions=[joint_ptp(HOME_LEFT)]),
        mg2.execute(traj2, tcp2, actions=[joint_ptp(HOME_RIGHT)]),
    )


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

        print("Moving to home...")
        await move_to_home(mg1, mg2)

        # Run policy
        feature_map = FeatureMap(
            groups=[
                FeatureGroup(
                    motion_group=mg1,
                    name="left",
                    ios={"gripper": "digital_out[0]", "conveyor_sensor": "digital_in[0]"},
                ),
                FeatureGroup(
                    motion_group=mg2,
                    name="right",
                    ios={"gripper": "digital_out[0]"},
                ),
            ]
        )

        executor = PolicyExecutor(
            feature_map=feature_map,
            policy=CallbackPolicyClient(mock_policy),
            safety_guards=[workspace_guard, speed_guard, io_guard],
            timeout_s=10.0,
        )

        print("Running policy for 10s (or until conveyor sensor triggers)...")
        try:
            result = await executor.run()
            print(
                f"Done: reason={result.reason} steps={result.steps} duration={result.duration_s:.1f}s"
            )
        except GuardStopError as e:
            print(f"Safety guard triggered: {e.guard_name}")
        except MotionError as e:
            print(f"Motion error (joint limit / collision): {e}")
        except EmergencyStopError as e:
            print(f"Emergency stop on controller: {e.controller_id}")
        except RuntimeError as e:
            print(f"Execution error: {e}")


if __name__ == "__main__":
    asyncio.run(main())
