"""
Example: Run a real GR00T inference server on two UR5e arms.

Run:
    NOVA_API=http://172.31.13.112 PYTHONPATH=. uv run --with pyzmq --with msgpack \
        python policy/examples/execute_gr00t_dual_arm.py
"""

from __future__ import annotations

import asyncio
import time

from policy import Gr00tPolicyClient, Observation, PolicyExecutor, PolicySchema, WebRTCCameras

import nova
from nova import api, run_program
from nova.actions import joint_ptp
from nova.cell import virtual_controller
from nova.program import ProgramPreconditions
from nova.types import MotionSettings

GROOT_HOST = "172.31.11.129"
GROOT_PORT = 30555
HOME_LEFT = (1.047, -0.698, 1.745, -3.142, 0.873, 2.094)
HOME_RIGHT = (-1.047, -2.356, -1.745, 0.0, -0.873, -2.094)
TIMEOUT_S = 120.0
CAMERA_SERVER = "http://172.31.11.129:8080"

CAM_RIGHT_WRIST = "World_Robot_Robot_0_R__0_00_robotics_usecase_gripper_asm_tn__00_00_robotics_usecase_gripperasm_io0_tn__01_00_CAMERA_ASMBLY__INTEL_D405_right_wrist_camera"
CAM_LEFT_WRIST = "World_Robot_Robot_0_L__0_00_robotics_usecase_gripper_asm_tn__00_00_robotics_usecase_gripperasm_io0_tn__01_00_CAMERA_ASMBLY__INTEL_D405_left_wrist_camera"
CAM_CONTEXT = "World_EnvAssets_rack_env0__3_00_intel_d456_screw_adapter_asm_context_camera"
CAM_TARGET = "World_EnvAssets_rack_env0_target_cam_stand_d405_prt_01_tn__target_cam_stand_d405prt_ta0_target_camera"


@nova.program(
    id="gr00t_dual_arm",
    name="GR00T Dual-Arm Policy",
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
    cell = ctx.nova.cell()
    mg_left = (await cell.controller("ur5e-left"))[0]
    mg_right = (await cell.controller("ur5e-right"))[0]

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

    cameras = WebRTCCameras(api_url=CAMERA_SERVER, width=224, height=224, fps=15, frame_history=1)

    schema = PolicySchema(observations=[
        Observation.joint_positions("left_joint_positions", source=mg_left),
        Observation.joint_positions("right_joint_positions", source=mg_right),
        Observation.image("exterior_image_1", source=cameras.device(CAM_CONTEXT)),
        Observation.image("exterior_image_2", source=cameras.device(CAM_TARGET)),
        Observation.image("left_wrist_image", source=cameras.device(CAM_LEFT_WRIST)),
        Observation.image("right_wrist_image", source=cameras.device(CAM_RIGHT_WRIST)),
        Observation.constant("language", value="Pick up the box and place it onto the conveyor."),
    ])

    client = Gr00tPolicyClient(
        host=GROOT_HOST, port=GROOT_PORT,
        language="Pick up the box and place it onto the conveyor.",
        timeout_ms=60000,
    )

    executor = PolicyExecutor(schema, client, timeout_s=TIMEOUT_S)

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


if __name__ == "__main__":
    run_program(gr00t_dual_arm)
