"""
Example: Run a real GR00T N1.7 inference server against a single UR10e.

The GR00T server at 172.31.11.129:30555 uses the OXE_DROID embodiment (7-DOF).
Since our UR10e has 6 joints, we set ``model_dof=7`` on the FeatureGroup which
causes the Gr00tPolicyClient to pad joints on input and truncate on output.

The camera server delivers 224x224 frames directly via WebRTC (matching GR00T's
expected input resolution). Camera names match OXE_DROID video keys.

Prerequisites:
    NOVA_API=http://<instance-ip> (with ur10e controller)
    GR00T server running at 172.31.11.129:30555
    Camera server running at 172.31.11.80:9100

Run:
    NOVA_API=http://172.31.10.248 PYTHONPATH=. uv run --with pyzmq --with msgpack \
        python policy/examples/execute_groot_single_arm.py
"""

from __future__ import annotations

import asyncio
import time

from policy import (
    CameraSet,
    FeatureGroup,
    FeatureMap,
    Gr00tPolicyClient,
    PolicyExecutor,
    TcpFormat,
    WebRTCCameraConfig,
)

from nova import Nova
from nova.actions import joint_ptp
from nova.types import MotionSettings

GROOT_HOST = "172.31.11.129"
GROOT_PORT = 30555
HOME = (0.0, -1.571, 1.571, -1.571, -1.571, 0.0)
TIMEOUT_S = 10.0
CAMERA_SERVER = "http://192.168.1.8:9100"
VIDEO_SIZE = 224
VIDEO_HISTORY = 2


async def main() -> None:
    nova = Nova()
    nova.nats.connect = lambda **_: asyncio.sleep(0)  # type: ignore[assignment]
    await nova.open()
    try:
        cell = nova.cell()
        ctrl = await cell.controller("ur10e")
        mg = ctrl[0]

        # Move to home
        print("Moving to home...")
        fast = MotionSettings(tcp_velocity_limit=500.0)
        tcp = (await mg.tcp_names())[0]
        traj = await mg.plan([joint_ptp(HOME, settings=fast)], tcp)
        await mg.execute(traj, tcp, actions=[joint_ptp(HOME, settings=fast)])
        print("  At home.")

        # FeatureMap: single arm with gripper + TCP pose
        # Note: model_dof=7 is only needed because OXE_DROID was trained on 7-DOF
        # and our UR10e has 6. For matching-DOF robots, omit it.
        feature_map = FeatureMap(groups=[
            FeatureGroup(
                motion_group=mg,
                name="arm",
                joint_key="joint_position",
                ios={"gripper_position": "digital_out[0]"},
                tcp_format=TcpFormat.ROT6D,
                tcp_key="eef_9d",
                model_dof=7,
            ),
        ])

        # Cameras: 224x224 matching GR00T input, T=2 history
        # Names must match GR00T's expected video keys from get_modality_config()
        cameras = CameraSet(
            configs={
                "exterior_image_1_left": WebRTCCameraConfig(
                    api_url=CAMERA_SERVER, device_id="315122271048",
                    width=VIDEO_SIZE, height=VIDEO_SIZE, fps=30,
                ),
                "wrist_image_left": WebRTCCameraConfig(
                    api_url=CAMERA_SERVER, device_id="314522065367",
                    width=VIDEO_SIZE, height=VIDEO_SIZE, fps=30,
                ),
            },
            frame_history=VIDEO_HISTORY,
        )

        # GR00T client — all mapping is in the FeatureMap
        client = Gr00tPolicyClient(
            host=GROOT_HOST,
            port=GROOT_PORT,
            feature_map=feature_map,
            language="Pick up the object on the table.",
        )

        executor = PolicyExecutor(
            motion_groups=[mg],
            cameras=cameras,
            policy=client,
            tcp=tcp,
            timeout_s=TIMEOUT_S,
        )

        print(f"Running GR00T policy for {TIMEOUT_S}s...")
        print(f"  Server: {GROOT_HOST}:{GROOT_PORT}")
        print(f"  Cameras: 2x {VIDEO_SIZE}x{VIDEO_SIZE} via {CAMERA_SERVER}")
        print()

        t0 = time.monotonic()
        try:
            result = await executor.run()
            dt = time.monotonic() - t0
            print(f"\nDone: reason={result.reason} steps={result.steps} duration={dt:.1f}s")
            print(f"  Inference rate: {result.steps / dt:.1f} Hz")
        except Exception as e:
            dt = time.monotonic() - t0
            print(f"\nError after {dt:.1f}s: {type(e).__name__}: {e}")
    finally:
        await nova.close()


if __name__ == "__main__":
    asyncio.run(main())
