"""
Example: Run a real GR00T inference server on two UR10e arms.

The GR00T server expects a dual-arm embodiment with:
- State: ``left_joint_positions``, ``right_joint_positions`` (6-DOF each)
- Action: ``left_joint_positions``, ``right_joint_positions`` (RELATIVE)
- Video: ``exterior_image_1``, ``exterior_image_2``,
         ``left_wrist_image``, ``right_wrist_image``
- Language: ``annotation.language.language_instruction``

Prerequisites:
    NOVA_API=http://<instance-ip> (with ur10e + ur10e-2 controllers)
    GR00T server running at GROOT_HOST:GROOT_PORT
    Camera server running at CAMERA_SERVER

Run:
    NOVA_API=http://172.31.13.112 PYTHONPATH=. uv run --with pyzmq --with msgpack \
        python policy/examples/execute_groot_dual_arm.py
"""

from __future__ import annotations

import asyncio
import time

from policy import CameraSet, FeatureGroup, FeatureMap, Gr00tPolicyClient, PolicyExecutor

import nova
from nova import api, run_program
from nova.actions import joint_ptp
from nova.cell import virtual_controller
from nova.program import ProgramPreconditions
from nova.types import MotionSettings

GROOT_HOST = "172.31.11.129"
GROOT_PORT = 30555
HOME_LEFT = (0.0, -1.571, 1.571, -1.571, -1.571, 0.0)
HOME_RIGHT = (0.0, -1.571, -1.571, -1.571, 1.571, 0.0)
TIMEOUT_S = 30.0
CAMERA_SERVER = "http://192.168.1.22:9100"
VIDEO_SIZE = 224
CAMERA_FPS = 15


@nova.program(
    id="groot_dual_arm",
    name="GR00T Dual-Arm Policy",
    description="Run GR00T inference on two UR10e arms with 4 cameras.",
    preconditions=ProgramPreconditions(
        controllers=[
            virtual_controller(
                name="ur10e",
                manufacturer=api.models.Manufacturer.UNIVERSALROBOTS,
                type="universalrobots-ur10e",
            ),
            virtual_controller(
                name="ur10e-2",
                manufacturer=api.models.Manufacturer.UNIVERSALROBOTS,
                type="universalrobots-ur10e",
            ),
        ],
        cleanup_controllers=False,
    ),
)
async def groot_dual_arm(ctx: nova.ProgramContext):
    cell = ctx.nova.cell()
    ctrl_left = await cell.controller("ur10e")
    ctrl_right = await cell.controller("ur10e-2")
    mg_left = ctrl_left[0]
    mg_right = ctrl_right[0]

    # Move both arms to home concurrently
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
    print("  At home.")

    # FeatureMap: dual arm, keys match GR00T modality config
    feature_map = FeatureMap(
        groups=[
            FeatureGroup(
                motion_group=mg_left, name="left", joint_key="left_joint_positions", tcp=tcp_left
            ),
            FeatureGroup(
                motion_group=mg_right,
                name="right",
                joint_key="right_joint_positions",
                tcp=tcp_right,
            ),
        ]
    )

    # Cameras: 224x224@15fps, names match GR00T video keys
    cameras = CameraSet(
        api_url=CAMERA_SERVER,
        devices={
            "exterior_image_1": "315122271048",
            "exterior_image_2": "319522063360",
            "left_wrist_image": "314522065367",
            "right_wrist_image": "314522065367",  # same cam — no physical right wrist cam
        },
        width=VIDEO_SIZE,
        height=VIDEO_SIZE,
        fps=CAMERA_FPS,
        frame_history=1,
    )

    # GR00T client
    client = Gr00tPolicyClient(
        host=GROOT_HOST, port=GROOT_PORT, language="Pick up the chest on the table."
    )

    executor = PolicyExecutor(
        feature_map=feature_map, cameras=cameras, policy=client, timeout_s=TIMEOUT_S
    )

    print(f"Running GR00T dual-arm policy for {TIMEOUT_S}s...")
    print(f"  Server: {GROOT_HOST}:{GROOT_PORT}")
    print(f"  Cameras: 4x {VIDEO_SIZE}x{VIDEO_SIZE} via {CAMERA_SERVER}")
    print()

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
    run_program(groot_dual_arm)
