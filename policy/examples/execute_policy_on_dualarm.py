"""
Example: Run a mock policy on two UR10e robots with cameras.

This demonstrates:
- Move to home before policy execution
- FeatureMap for LeRobot-compatible flat feature dicts
- WebRTC cameras attached — images included in every observation
- Policy operates on named features + camera images
- Policy never sees motion group IDs — fully hardware-agnostic
- Time-based execution (timeout_s)
- Safety guards (workspace, speed, and IO-based sensor trigger)

Prerequisites:
- Set env variables (or .env file):
    - NOVA_API=http://<instance-ip>
    - CAMERA_SERVER=http://localhost:9100  (run mock-camera-server Docker image)
- Two UR10e controllers on the NOVA instance
- Camera server with three cameras (flange, left, right)

Run:
    # Start mock camera server (in another terminal):
    docker run --rm -p 9100:9100 mock-camera-server

    # Run this example:
    NOVA_API=http://172.31.10.95 CAMERA_SERVER=http://localhost:9100 PYTHONPATH=. \
        python policy/examples/execute_policy_on_dualarm.py
"""

import asyncio
import math
import os
from typing import Any

import numpy as np
from policy import (
    CallbackPolicyClient,
    CameraSet,
    EmergencyStopError,
    FeatureGroup,
    FeatureMap,
    GuardState,
    GuardStopError,
    MotionError,
    PolicyExecutor,
    WebRTCCameraConfig,
)

from nova import Nova
from nova.actions import joint_ptp

HOME_LEFT = (0.0, -1.571, 1.571, -1.571, -1.571, 0.0)
HOME_RIGHT = (0.0, -1.571, -1.571, -1.571, 1.571, 0.0)

CAMERA_SERVER = os.environ.get("CAMERA_SERVER", "http://localhost:9100")
CAM_FLANGE = os.environ.get("CAM_FLANGE", "315122271048")
CAM_LEFT = os.environ.get("CAM_LEFT", "314522065367")
CAM_RIGHT = os.environ.get("CAM_RIGHT", "319522063360")


# ---------------------------------------------------------------------------
# Policy: obs → actions. Pure function, no episode logic.
# ---------------------------------------------------------------------------

_step_counter = 0


async def mock_policy(obs: dict[str, Any]) -> dict[str, float]:
    """Stateless mock policy using flat feature names + camera images.

    Receives joint observations and camera images. In a real policy (ACT,
    Diffusion Policy, etc.) the images would be passed through a inference.
    Here we demonstrate the interface and verify images arrive.

    Input:  {"left_joint_1.pos": 0.1, ..., "flange": np.array(480,640,3), ...}
    Output: {"left_joint_1.pos": 0.15, ..., "left_gripper": 1.0, ...}
    """
    global _step_counter
    _step_counter += 1

    # Verify camera images are present and correctly shaped
    for cam_name in ("flange", "left", "right"):
        if cam_name in obs:
            img = obs[cam_name]
            assert isinstance(img, np.ndarray), f"Expected numpy array for '{cam_name}'"
            assert img.shape[2] == 3, f"Expected RGB image for '{cam_name}', got shape {img.shape}"
            if _step_counter % 30 == 1:
                print(
                    f"  [step {_step_counter:>3}] {cam_name:>6}: "
                    f"shape={img.shape} dtype={img.dtype} "
                    f"mean={img.mean():.1f} min={img.min()} max={img.max()}"
                )
        elif _step_counter == 1:
            print(f"  [step {_step_counter:>3}] {cam_name:>6}: NOT IN OBS")

    # Compute actions from joint state (images would inform a real policy)
    amplitude = 0.08
    features: dict[str, float] = {}

    # Sum of all joints creates coupled oscillation that never converges
    all_joints_sum = sum(
        float(obs.get(f"{r}_joint_{j}.pos", 0.0)) for r in ("left", "right") for j in range(1, 7)
    )

    for role in ("left", "right"):
        for i in range(6):
            key = f"{role}_joint_{i + 1}.pos"
            current = obs.get(key, 0.0)
            phase = i * 0.4 + (0.0 if role == "left" else math.pi)
            features[key] = current + amplitude * math.sin(all_joints_sum * 3.0 + phase)
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
                FeatureGroup(motion_group=mg2, name="right", ios={"gripper": "digital_out[0]"}),
            ]
        )

        # Cameras — connect to WebRTC camera server
        cameras = CameraSet(
            configs={
                "flange": WebRTCCameraConfig(
                    api_url=CAMERA_SERVER, device_id=CAM_FLANGE, width=640, height=480, fps=30
                ),
                "left": WebRTCCameraConfig(
                    api_url=CAMERA_SERVER, device_id=CAM_LEFT, width=640, height=480, fps=30
                ),
                "right": WebRTCCameraConfig(
                    api_url=CAMERA_SERVER, device_id=CAM_RIGHT, width=640, height=480, fps=30
                ),
            }
        )

        executor = PolicyExecutor(
            feature_map=feature_map,
            cameras=cameras,
            policy=CallbackPolicyClient(mock_policy),
            safety_guards=[workspace_guard, speed_guard, io_guard],
            timeout_s=10.0,
        )

        print("Running policy for 10s (or until conveyor sensor triggers)...")
        print(f"  Cameras: {list(cameras.configs.keys())} via {CAMERA_SERVER}")
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
