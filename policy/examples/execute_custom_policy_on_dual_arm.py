"""
Example: Run a mock policy on two UR5e robots with cameras.

Demonstrates advanced features:
- Safety guards (workspace boundary)
- Computed observations (step counter)
- Computed actions (logging side effects)
- IO control (gripper open/close)
- Camera image observations

Prerequisites:
    NOVA_API=http://<instance-ip>
    CAMERA_SERVER=http://192.168.1.22:9100  (optional)

Run:
    PYTHONPATH=. python policy/examples/execute_custom_policy_on_dual_arm.py
"""

import asyncio
import math
import os
from typing import Any

import nova
from nova import api, run_program, viewers
from nova.actions import joint_ptp
from nova.cell import virtual_controller
from nova.program import ProgramPreconditions
from nova.types import MotionSettings
from policy import (
    Action,
    BoolMapping,
    EmergencyStopError,
    GuardStopError,
    MotionError,
    Observation,
    PolicyExecutor,
    PolicySchema,
    WebRTCCameras,
)
from policy.types import GuardState

HOME_LEFT = (1.169, -0.733, 1.745, -3.054, 0.872, 2.094)
HOME_RIGHT = (-1.169, -2.3911, -1.8675, 0.0, -0.872, -2.094)

CAMERA_SERVER = os.environ.get("CAMERA_SERVER", "http://172.31.11.129:8011/webrtc-streamer")


async def mock_policy(obs: dict[str, Any]) -> dict[str, float]:
    """Mock policy — replace with your own inference call.

    The observation dict contains all declared entries:
    - ``left_joints_1`` .. ``left_joints_6`` — joint positions
    - ``left_gripper`` — gripper state (0.0 or 100.0)
    - ``elapsed_s`` — from Observation.computed (see below)
    - camera images if configured
    """
    features: dict[str, float] = {}
    for role in ("left", "right"):
        role_phase = 0.0 if role == "left" else math.pi
        all_joints_sum = sum(obs.get(f"{role}_joints_{i}", 0.0) for i in range(1, 7))
        for i in range(1, 7):
            key = f"{role}_joints_{i}"
            current = obs.get(key, 0.0)
            phase = role_phase + i * 0.7
            features[key] = current + 0.05 * math.sin(all_joints_sum * 3.0 + phase)
        shoulder = obs.get(f"{role}_joints_2", 0.0)
        features[f"{role}_gripper"] = 100.0 if shoulder > 0 else 0.0
    return features


# ---------------------------------------------------------------------------
# Computed observation: add custom data to the observation each step.
# The function receives the obs dict built so far and returns extra entries.
# ---------------------------------------------------------------------------

_step_counter = 0


async def count_steps(obs: dict[str, Any]) -> dict[str, Any]:
    """Example Observation.computed — adds a step counter and elapsed seconds."""
    global _step_counter
    _step_counter += 1
    return {"step": _step_counter, "elapsed_s": _step_counter / 30.0}


# ---------------------------------------------------------------------------
# Computed action: trigger side effects when the policy returns.
# The function receives the full action dict from the policy.
# ---------------------------------------------------------------------------


_action_count = 0


async def log_action(action: dict[str, Any]) -> None:
    """Example Action.computed — logs every 100th action."""
    global _action_count
    _action_count += 1
    if _action_count % 100 == 0:
        print(f"  [action logger] step={_action_count}")


def workspace_guard(ctx: GuardState) -> bool:
    return ctx.state.pose.position[2] > -500


def speed_guard(ctx: GuardState) -> bool:
    if ctx.prev_state is None or ctx.dt < 0.005:
        return True
    p0 = ctx.prev_state.pose.position
    p1 = ctx.state.pose.position
    dist = sum((a - b) ** 2 for a, b in zip(p1, p0, strict=False)) ** 0.5
    return dist / ctx.dt < 5000.0


def io_guard(ctx: GuardState) -> bool:
    if ctx.io_values is None:
        return True
    return ctx.io_values.get("digital_in[0]") is not True


async def move_to_home(mg1, mg2) -> None:
    fast = MotionSettings(tcp_velocity_limit=500.0)
    tcp1, tcp2 = (await mg1.tcp_names())[0], (await mg2.tcp_names())[0]
    t1 = await mg1.plan([joint_ptp(HOME_LEFT, settings=fast)], tcp1)
    t2 = await mg2.plan([joint_ptp(HOME_RIGHT, settings=fast)], tcp2)
    await asyncio.gather(
        mg1.execute(t1, tcp1, actions=[joint_ptp(HOME_LEFT, settings=fast)]),
        mg2.execute(t2, tcp2, actions=[joint_ptp(HOME_RIGHT, settings=fast)]),
    )


@nova.program(
    id="dual_arm_policy",
    name="Dual-Arm Policy Execution",
    viewer=viewers.Rerun(),
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
async def dual_arm_policy(ctx: nova.ProgramContext):
    cell = ctx.nova.cell()
    mg1 = (await cell.controller("ur5e-left"))[0]
    mg2 = (await cell.controller("ur5e-right"))[0]

    print("Moving to home...")
    await move_to_home(mg1, mg2)

    observations = [
        Observation.joint_positions("left_joints", source=mg1),
        Observation.joint_positions("right_joints", source=mg2),
        Observation.tcp("left_tcp", source=mg1, tcp="gripper"),
        Observation.tcp("right_tcp", source=mg2, tcp="gripper"),
        Observation.io("left_gripper", source=mg1, io="digital_out[0]",
                       mapping=BoolMapping(on=100.0)),
        Observation.io("right_gripper", source=mg2, io="digital_out[0]",
                       mapping=BoolMapping(on=100.0)),
        Observation.io("left_sensor", source=mg1, io="digital_in[0]", action=False),
        # Computed observations add custom data each step
        Observation.computed(count_steps),
    ]

    if CAMERA_SERVER:
        cameras = WebRTCCameras(api_url=CAMERA_SERVER)
        observations.extend([
            Observation.image("cam_1", source=cameras.device("World_Robot_Robot_0_R__0_00_robotics_usecase_gripper_asm_tn__00_00_robotics_usecase_gripper_asm__tn__01_00_CAMERA_ASMBLY__INTEL_D405_D405_SOLID_right_wrist_camera_env0")),
            Observation.image("cam_2", source=cameras.device("World_Robot_Robot_0_L__0_00_robotics_usecase_gripper_asm_tn__00_00_robotics_usecase_gripper_asm__tn__01_00_CAMERA_ASMBLY__INTEL_D405_D405_SOLID_left_wrist_camera_env0")),
            Observation.image("cam_3", source=cameras.device("World_EnvAssets_rack_env0__3_00_intel_d456_screw_adapter_asm_tn__03_00_intel_d456_screw_adapterasm_io0_D456_Solid_context_camera_rack_env0")),
        ])

    schema = PolicySchema(
        observations=observations,
        # Computed actions trigger side effects when the policy returns
        actions=[Action.computed(log_action)],
    )

    executor = PolicyExecutor(
        schema,
        mock_policy,
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
    run_program(dual_arm_policy)
