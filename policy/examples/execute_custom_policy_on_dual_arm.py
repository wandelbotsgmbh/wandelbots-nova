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

from nova import ProgramContext, api, program, run_program, viewers
from nova.actions import joint_ptp
from nova.cell import virtual_controller
from nova.program import ProgramPreconditions
from nova.types import MotionSettings
from policy import (
    Action,
    ActionChunk,
    BoolMapping,
    EmergencyStopError,
    MotionError,
    Observation,
    PolicyExecutor,
    PolicySchema,
    WebRTCCameras,
)
from policy.types import StopContext

HOME_LEFT = (1.169, -0.733, 1.745, -3.054, 0.872, 2.094)
HOME_RIGHT = (-1.169, -2.3911, -1.8675, 0.0, -0.872, -2.094)


def _first_policy_target(home: tuple[float, ...], role_phase: float) -> list[float]:
    """Compute the policy's first joint target at t=0."""
    targets = []
    for i in range(6):
        phase = role_phase + (i + 1) * 0.8
        amplitude = 0.15 if i in (0, 1, 2) else 0.08
        targets.append(home[i] + amplitude * math.sin(phase))
    return targets


FIRST_LEFT = _first_policy_target(HOME_LEFT, 0.0)
FIRST_RIGHT = _first_policy_target(HOME_RIGHT, math.pi)

CAMERA_SERVER = os.environ.get("CAMERA_SERVER", "http://172.31.11.129:8011/webrtc-streamer")


async def mock_policy(obs: dict[str, Any]) -> ActionChunk:
    """Mock policy — replace with your own inference call.

    Returns multi-step action chunks (16 steps at 50ms) with visible sinusoidal
    motion around the HOME positions. Uses trajectory-absolute timestamps
    so overlapping chunks align correctly.
    """
    # Use elapsed time for smooth continuous motion
    elapsed = obs.get("elapsed_s", 0.0)

    joints: dict[str, list[list[float]]] = {}
    for role, mg_id, home in (
        ("left", "0@ur5e-left", list(HOME_LEFT)),
        ("right", "0@ur5e-right", list(HOME_RIGHT)),
    ):
        role_phase = 0.0 if role == "left" else math.pi
        steps: list[list[float]] = []
        for step_i in range(16):
            t = elapsed + step_i * 0.05  # 50ms per step
            joint_targets = []
            for i in range(6):
                phase = role_phase + (i + 1) * 0.8
                amplitude = 0.15 if i in (0, 1, 2) else 0.08
                freq = 0.2 + (i + 1) * 0.03
                joint_targets.append(home[i] + amplitude * math.sin(2 * math.pi * freq * t + phase))
            steps.append(joint_targets)
        joints[mg_id] = steps

    # Trajectory-absolute timestamp so overlapping chunks align correctly
    first_timestamp_ms = int(elapsed * 1000)
    return ActionChunk(joints=joints, dt_ms=50.0, first_timestamp_ms=first_timestamp_ms)


# ---------------------------------------------------------------------------
# Computed observation: add custom data to the observation each step.
# The function receives the obs dict built so far and returns extra entries.
# ---------------------------------------------------------------------------

_step_counter = 0


_start_time: float | None = None


async def count_steps(obs: dict[str, Any]) -> dict[str, Any]:
    """Example Observation.computed — adds a step counter and real elapsed seconds."""
    global _step_counter, _start_time
    _step_counter += 1
    if _start_time is None:
        import time

        _start_time = time.monotonic()
    import time

    elapsed = time.monotonic() - _start_time
    return {"step": _step_counter, "elapsed_s": elapsed}


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


def stop_on_speed(ctx: StopContext) -> bool:
    """Stop if the TCP moves faster than 5 m/s (runaway guard)."""
    if ctx.prev_state is None or ctx.dt < 0.005:
        return False
    p0 = ctx.prev_state.pose.position
    p1 = ctx.state.pose.position
    dist = sum((a - b) ** 2 for a, b in zip(p1, p0, strict=False)) ** 0.5
    return dist / ctx.dt > 5000.0


def stop_on_io(ctx: StopContext) -> bool:
    """Stop when the operator/PLC raises digital_in[0]."""
    if ctx.io_values is None:
        return False
    return ctx.io_values.get("digital_in[0]") is True


async def move_to_home(mg1, mg2) -> None:
    fast = MotionSettings(tcp_velocity_limit=500.0)
    tcp1, tcp2 = (await mg1.tcp_names())[0], (await mg2.tcp_names())[0]
    # Move to HOME first, then to the policy's first target (avoids initial jump)
    t1 = await mg1.plan(
        [joint_ptp(HOME_LEFT, settings=fast), joint_ptp(FIRST_LEFT, settings=fast)], tcp1
    )
    t2 = await mg2.plan(
        [joint_ptp(HOME_RIGHT, settings=fast), joint_ptp(FIRST_RIGHT, settings=fast)], tcp2
    )
    await asyncio.gather(
        mg1.execute(
            t1,
            tcp1,
            actions=[joint_ptp(HOME_LEFT, settings=fast), joint_ptp(FIRST_LEFT, settings=fast)],
        ),
        mg2.execute(
            t2,
            tcp2,
            actions=[joint_ptp(HOME_RIGHT, settings=fast), joint_ptp(FIRST_RIGHT, settings=fast)],
        ),
    )


@program(
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
async def dual_arm_policy(ctx: ProgramContext):
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
        Observation.io(
            "left_gripper", source=mg1, io="digital_out[0]", mapping=BoolMapping(on=100.0)
        ),
        Observation.io(
            "right_gripper", source=mg2, io="digital_out[0]", mapping=BoolMapping(on=100.0)
        ),
        Observation.io("left_sensor", source=mg1, io="digital_in[0]", action=False),
        # Computed observations add custom data each step
        Observation.computed(count_steps),
    ]

    if CAMERA_SERVER:
        cameras = WebRTCCameras(api_url=CAMERA_SERVER)
        observations.extend(
            [
                Observation.image(
                    "cam_1",
                    source=cameras.device(
                        "World_Robot_Robot_0_R__0_00_robotics_usecase_gripper_asm_tn__00_00_robotics_usecase_gripper_asm__tn__01_00_CAMERA_ASMBLY__INTEL_D405_D405_SOLID_right_wrist_camera_env0"
                    ),
                ),
                Observation.image(
                    "cam_2",
                    source=cameras.device(
                        "World_Robot_Robot_0_L__0_00_robotics_usecase_gripper_asm_tn__00_00_robotics_usecase_gripper_asm__tn__01_00_CAMERA_ASMBLY__INTEL_D405_D405_SOLID_left_wrist_camera_env0"
                    ),
                ),
                Observation.image(
                    "cam_3",
                    source=cameras.device(
                        "World_EnvAssets_rack_env0__3_00_intel_d456_screw_adapter_asm_tn__03_00_intel_d456_screw_adapterasm_io0_D456_Solid_context_camera_rack_env0"
                    ),
                ),
            ]
        )

    schema = PolicySchema(
        observations=observations,
        # Computed actions trigger side effects when the policy returns
        actions=[Action.computed(log_action)],
    )

    executor = PolicyExecutor(
        schema,
        mock_policy,
        stop_conditions=[stop_on_speed, stop_on_io],
        timeout_s=10.0,
        policy_rate_hz=20,
        n_action_steps=8,
    )

    print("Running policy for 10s...")
    try:
        result = await executor.run()
        print(
            f"Done: reason={result.reason} steps={result.steps} duration={result.duration_s:.1f}s"
        )
    except MotionError as e:
        print(f"Motion error: {e}")
    except EmergencyStopError as e:
        print(f"Emergency stop: {e.controller_id}")


if __name__ == "__main__":
    run_program(dual_arm_policy)
