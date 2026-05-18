"""Example: Run a real GR00T inference server on two UR5e arms.

Run:
    NOVA_API=http://172.31.11.129 PYTHONPATH=. uv run --with pyzmq --with msgpack \
        --with aiortc --with requests --with python-statemachine \
        python policy/examples/execute_gr00t_dual_arm.py
"""

from __future__ import annotations

import asyncio
import time

import nova
from nova import api, run_program, viewers
from nova.actions import joint_ptp
from nova.cell import virtual_controller
from nova.program import ProgramPreconditions
from nova.types import MotionSettings
from policy import (
    Gr00tPolicyClient,
    MotionConfig,
    Observation,
    PolicyExecutor,
    PolicySchema,
    WebRTCCameras,
)

GROOT_HOST = "172.31.11.129"
GROOT_PORT = 30555
HOME_LEFT = (1.169, -0.733, 1.745, -3.054, 0.872, 2.094)
HOME_RIGHT = (-1.169, -2.3911, -1.8675, 0.0, -0.872, -2.094)
TIMEOUT_S = 3000.0
CAMERA_SERVER = "http://172.31.11.129:8011/webrtc-streamer"

CAM_RIGHT_WRIST = "World_Robot_Robot_0_R__0_00_robotics_usecase_gripper_asm_tn__00_00_robotics_usecase_gripper_asm__tn__01_00_CAMERA_ASMBLY__INTEL_D405_D405_SOLID_right_wrist_camera_env0"
CAM_LEFT_WRIST = "World_Robot_Robot_0_L__0_00_robotics_usecase_gripper_asm_tn__00_00_robotics_usecase_gripper_asm__tn__01_00_CAMERA_ASMBLY__INTEL_D405_D405_SOLID_left_wrist_camera_env0"
CAM_CONTEXT = "World_EnvAssets_rack_env0__3_00_intel_d456_screw_adapter_asm_tn__03_00_intel_d456_screw_adapterasm_io0_D456_Solid_context_camera_rack_env0"
CAM_TARGET = "World_EnvAssets_rack_env0_d405_halter_stationaer_2_asm_01_tn__d405_halter_stationaer_2asm_ff0_D405_Solid_target_camera_rack_env0"


# ---------------------------------------------------------------------------
# Debug tracking
# ---------------------------------------------------------------------------

_debug_log: list[dict] = []
_t0: float = 0.0


def _log_step(
    step: int,
    obs_joints: dict[str, list[float]],
    action_joints: dict[str, list[list[float]]],
    dt_ms: float,
) -> None:
    """Record one inference step for post-run analysis."""
    entry: dict = {
        "step": step,
        "t": time.monotonic() - _t0,
        "dt_ms": dt_ms,
    }
    for mg_id, joints in obs_joints.items():
        entry[f"obs_{mg_id}"] = list(joints[:3])
    for mg_id, steps in action_joints.items():
        entry[f"act_{mg_id}_first"] = list(steps[0][:3])
        entry[f"act_{mg_id}_last"] = list(steps[-1][:3])
        entry[f"act_{mg_id}_n"] = len(steps)
        obs = obs_joints.get(mg_id, steps[0])
        entry[f"delta_{mg_id}_first"] = [
            round(steps[0][j] - obs[j], 5) for j in range(min(3, len(obs)))
        ]
    _debug_log.append(entry)


def _print_debug_summary() -> None:
    """Print analysis of recorded steps."""
    if not _debug_log:
        return

    print(f"\n{'=' * 80}")
    print("  DEBUG: Inference step log (left arm, first 3 joints)")
    print(f"{'=' * 80}")
    print(
        f"  {'step':>4} {'t':>6} {'n':>2}  {'obs[0:3]':<28} {'act_first[0:3]':<28} {'delta[0:3]'}"
    )
    print(f"  {'-' * 4} {'-' * 6} {'-' * 2}  {'-' * 28} {'-' * 28} {'-' * 24}")

    mg = "0@ur5e-left"
    for e in _debug_log[:20]:
        obs = e.get(f"obs_{mg}", [0, 0, 0])
        act = e.get(f"act_{mg}_first", [0, 0, 0])
        delta = e.get(f"delta_{mg}_first", [0, 0, 0])
        n = e.get(f"act_{mg}_n", 0)
        print(
            f"  {e['step']:>4} {e['t']:>6.2f} {n:>2}  "
            f"{[f'{v:.4f}' for v in obs]!s:<28} "
            f"{[f'{v:.4f}' for v in act]!s:<28} "
            f"{[f'{v:.5f}' for v in delta]}"
        )

    # Direction reversals in action deltas (zig-zag indicator)
    reversals = 0
    for i in range(1, len(_debug_log)):
        prev = _debug_log[i - 1].get(f"delta_{mg}_first", [0])
        curr = _debug_log[i].get(f"delta_{mg}_first", [0])
        if prev and curr and len(prev) > 0 and len(curr) > 0:
            if (prev[0] > 0.001 and curr[0] < -0.001) or (prev[0] < -0.001 and curr[0] > 0.001):
                reversals += 1

    print(f"\n  Direction reversals (left j0 delta sign flips): {reversals}")
    print(f"  Total inference steps: {len(_debug_log)}")

    # Chunk continuity: does observation match where previous chunk ended?
    if len(_debug_log) > 1:
        print("\n  Chunk continuity (obs vs previous chunk's last step):")
        for i in range(1, min(10, len(_debug_log))):
            prev_last = _debug_log[i - 1].get(f"act_{mg}_last", [0, 0, 0])
            curr_obs = _debug_log[i].get(f"obs_{mg}", [0, 0, 0])
            gap = [round(curr_obs[j] - prev_last[j], 4) for j in range(3)]
            gap_deg = [round(g * 57.3, 1) for g in gap]
            print(f"    step {_debug_log[i]['step']}: gap = {gap_deg} deg")


# ---------------------------------------------------------------------------
# Wrapped GR00T client that logs observations and actions
# ---------------------------------------------------------------------------


class DebugGr00tClient:
    """Wraps Gr00tPolicyClient to log obs/action pairs."""

    def __init__(self, client: Gr00tPolicyClient) -> None:
        self._client = client
        self._step = 0

    async def connect(self, mg_ids: list[str]) -> None:
        return await self._client.connect(mg_ids)

    async def close(self) -> None:
        return await self._client.close()

    async def validate_schema(self, schema: PolicySchema) -> None:
        return await self._client.validate_schema(schema)

    async def get_actions(self, states, schema, images, io_values):
        # Extract observation joints
        obs_joints: dict[str, list[float]] = {}
        for mg_id, state in states.items():
            if hasattr(state, "joints"):
                obs_joints[mg_id] = list(state.joints)

        # Call real client
        result = await self._client.get_actions(states, schema, images, io_values)

        # Log
        if hasattr(result, "joints") and result.joints:
            _log_step(self._step, obs_joints, result.joints, result.dt_ms)
        self._step += 1
        return result

    async def ping(self):
        return await self._client.ping()

    async def get_server_info(self):
        return await self._client.get_server_info()


# ---------------------------------------------------------------------------
# Main program
# ---------------------------------------------------------------------------


@nova.program(
    id="gr00t_dual_arm",
    name="GR00T Dual-Arm Policy",
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
async def gr00t_dual_arm(ctx: nova.ProgramContext):
    global _t0
    _t0 = time.monotonic()
    _debug_log.clear()

    cell = ctx.nova.cell()
    mg_left = (await cell.controller("ur5e-left"))[0]
    mg_right = (await cell.controller("ur5e-right"))[0]

    # Move to home
    print("Moving to home...")
    fast = MotionSettings(tcp_velocity_limit=500.0)
    tcp_left = (await mg_left.tcp_names())[0]
    tcp_right = (await mg_right.tcp_names())[0]
    t1 = await mg_left.plan([joint_ptp(HOME_LEFT, settings=fast)], tcp_left)
    t2 = await mg_right.plan([joint_ptp(HOME_RIGHT, settings=fast)], tcp_right)
    await asyncio.gather(
        mg_left.execute(t1, tcp_left, actions=[joint_ptp(HOME_LEFT, settings=fast)]),
        mg_right.execute(t2, tcp_right, actions=[joint_ptp(HOME_RIGHT, settings=fast)]),
    )

    # Cameras
    cameras = WebRTCCameras(api_url=CAMERA_SERVER, frame_history=1, resize=(224, 224))

    # Schema
    schema = PolicySchema(
        observations=[
            Observation.joint_positions("left_joint_positions", source=mg_left),
            Observation.joint_positions("right_joint_positions", source=mg_right),
            Observation.image("exterior_image_1", source=cameras.device(CAM_CONTEXT)),
            Observation.image("exterior_image_2", source=cameras.device(CAM_TARGET)),
            Observation.image("left_wrist_image", source=cameras.device(CAM_LEFT_WRIST)),
            Observation.image("right_wrist_image", source=cameras.device(CAM_RIGHT_WRIST)),
            Observation.constant(
                "language", value="Pick up the box and place it onto the conveyor."
            ),
        ]
    )

    # GR00T client with debug wrapper
    raw_client = Gr00tPolicyClient(
        host=GROOT_HOST,
        port=GROOT_PORT,
        dt_ms=66.7,
    )
    client = DebugGr00tClient(raw_client)

    # Print server config
    info = await raw_client.get_server_info()
    print(
        f"GR00T server: {len(info['state_keys'])} state keys, "
        f"{len(info['video_keys'])} cameras, "
        f"horizon={info['action_horizon']}"
    )

    # Run
    executor = PolicyExecutor(
        schema,
        client,
        timeout_s=TIMEOUT_S,
        motion=MotionConfig(n_action_steps=8),
    )

    print(f"Running GR00T dual-arm policy for {TIMEOUT_S}s...")
    t0 = time.monotonic()
    try:
        result = await executor.run()
        dt = time.monotonic() - t0
        print(f"\nDone: reason={result.reason} steps={result.steps} duration={dt:.1f}s")
        if result.steps > 0:
            print(f"  Inference rate: {result.steps / dt:.1f} Hz")
    except Exception as e:
        dt = time.monotonic() - t0
        print(f"\nError after {dt:.1f}s: {type(e).__name__}: {e}")

    _print_debug_summary()


if __name__ == "__main__":
    run_program(gr00t_dual_arm)
