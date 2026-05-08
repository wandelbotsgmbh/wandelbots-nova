"""
Example: Run a real GR00T inference server on a single UR10e.

Prerequisites:
    NOVA_API=http://<instance-ip> (with ur5e-left controller)
    GR00T server running at GROOT_HOST:GROOT_PORT
    Camera server running at CAMERA_SERVER

Run:
    NOVA_API=http://172.31.13.112 PYTHONPATH=. uv run --with pyzmq --with msgpack \
        python policy/examples/execute_groot_single_arm.py
"""

from __future__ import annotations

import time

from policy import (
    CameraSet,
    FeatureGroup,
    FeatureMap,
    Gr00tPolicyClient,
    PolicyExecutor,
)

import nova
from nova import api, run_program
from nova.actions import joint_ptp
from nova.cell import virtual_controller
from nova.program import ProgramPreconditions
from nova.types import MotionSettings

GROOT_HOST = "172.31.11.129"
GROOT_PORT = 30555
HOME = (1.047, -0.698, 1.745, -3.142, 0.873, 2.094)
TIMEOUT_S = 30.0
CAMERA_SERVER = "http://192.168.1.22:9100"
VIDEO_SIZE = 224
CAMERA_FPS = 15


@nova.program(
    id="groot_single_arm",
    name="GR00T Single-Arm Policy",
    description="Run GR00T inference on a single UR10e with 3 cameras.",
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
async def groot_single_arm(ctx: nova.ProgramContext):
    cell = ctx.nova.cell()
    ctrl = await cell.controller("ur5e-left")
    mg = ctrl[0]

    # Move to home
    print("Moving to home...")
    fast = MotionSettings(tcp_velocity_limit=500.0)
    tcp = (await mg.tcp_names())[0]
    traj = await mg.plan([joint_ptp(HOME, settings=fast)], tcp)
    await mg.execute(traj, tcp, actions=[joint_ptp(HOME, settings=fast)])
    print("  At home.")

    # FeatureMap: single arm, key matches GR00T modality config
    feature_map = FeatureMap(groups=[
        FeatureGroup(
            motion_group=mg,
            name="arm",
            joint_key="joint_position",
            tcp=tcp,
        ),
    ])

    # Cameras: 224x224@15fps via WebRTC
    cameras = CameraSet(
        api_url=CAMERA_SERVER,
        devices={
            "exterior_image_1_left": "315122271048",
            "wrist_image_left": "314522065367",
            "exterior_image_2_left": "319522063360",
        },
        width=VIDEO_SIZE,
        height=VIDEO_SIZE,
        fps=CAMERA_FPS,
        frame_history=1,
    )

    # GR00T client
    client = Gr00tPolicyClient(
        host=GROOT_HOST,
        port=GROOT_PORT,
        language="Pick up the object on the table.",
    )

    executor = PolicyExecutor(
        feature_map=feature_map,
        cameras=cameras,
        policy=client,
        timeout_s=TIMEOUT_S,
    )

    print(f"Running GR00T policy for {TIMEOUT_S}s...")
    print(f"  Server: {GROOT_HOST}:{GROOT_PORT}")
    print(f"  Cameras: 3x {VIDEO_SIZE}x{VIDEO_SIZE} via {CAMERA_SERVER}")
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
    run_program(groot_single_arm)
