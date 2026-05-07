"""GR00T robot controller — runs policy via ZMQ against a GR00T-compatible server.

Uses the same FeatureMap as the executor to build observations and decode actions.
The Gr00tPolicyClient handles conversion to/from GR00T's numpy format.

Provides:
- GET /status
- POST /start
- POST /stop
"""

from __future__ import annotations

import asyncio
import contextlib
import logging
from typing import TYPE_CHECKING, Any

import uvicorn
from decouple import config
from fastapi import FastAPI
from fastapi.middleware.cors import CORSMiddleware
from fastapi.responses import FileResponse
from policy import (
    CameraSet,
    FeatureGroup,
    FeatureMap,
    Gr00tPolicyClient,
    PolicyExecutor,
    TcpFormat,
    WebRTCCameraConfig,
)
from policy.executor import ExecutorStatus
from pydantic import BaseModel, Field

from nova import Nova
from nova.actions import joint_ptp
from nova.cell.motion_group import MotionGroup
from nova.types import MotionSettings

if TYPE_CHECKING:
    from policy.executor import ExecutionResult

logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)

BASE_PATH = config("BASE_PATH", default="", cast=str)
CAMERA_SERVER = config("CAMERA_SERVER", default="", cast=str)

app = FastAPI(
    title="GR00T Robot Controller",
    version="0.3.0",
    description="Runs dual-arm policy execution against a GR00T ZMQ inference server.",
    root_path=BASE_PATH,
    docs_url="/",
)

app.add_middleware(
    CORSMiddleware,
    allow_origins=["*"],
    allow_credentials=True,
    allow_methods=["*"],
    allow_headers=["*"],
)

_executor: PolicyExecutor | None = None
_nova: Nova | None = None
_run_task: asyncio.Task[ExecutionResult] | None = None


class CameraConfigModel(BaseModel):
    name: str
    device_id: str
    width: int = 640
    height: int = 480
    fps: int = 30


class StartRequest(BaseModel):
    policy_host: str = Field(
        default="app-mock-groot-policy-service",
        description="Hostname of the GR00T ZMQ policy service",
    )
    policy_port: int = Field(default=5555)
    motion_groups: str = Field(default="0@ur10e,0@ur10e-2")
    home_joints: str = Field(
        default="0,-1.571,1.571,-1.571,-1.571,0;0,-1.571,-1.571,-1.571,1.571,0",
    )
    gripper_ios: str = Field(
        default="digital_out[0],digital_out[0]",
        description="Comma-separated gripper IO keys (left, right)",
    )
    language: str = Field(
        default="Coordinate both arms and move smoothly.",
    )
    timeout_s: float = Field(default=10.0, ge=0)
    camera_server: str = Field(
        default=CAMERA_SERVER,
        description="Camera server URL. Empty = no cameras.",
    )
    cameras: list[CameraConfigModel] = Field(
        default=[],
        description="Cameras to include in observations.",
    )


# ---------------------------------------------------------------------------
# Endpoints
# ---------------------------------------------------------------------------


@app.get("/status", response_model=ExecutorStatus)
async def get_status() -> ExecutorStatus:
    if _executor is None:
        return ExecutorStatus()
    return _executor.status


@app.post("/start", response_model=ExecutorStatus)
async def start(req: StartRequest = StartRequest()) -> ExecutorStatus:
    """Move to home, then run GR00T policy until timeout or /stop."""
    global _executor, _nova, _run_task

    if _executor is not None and _executor.phase != "IDLE":
        return _executor.status

    _nova = Nova()
    await _nova.open()
    cell = _nova.cell()

    mg_ids = [mg.strip() for mg in req.motion_groups.split(",") if mg.strip()]
    if len(mg_ids) != 2:
        msg = "Exactly two motion groups required"
        raise ValueError(msg)

    homes_raw = req.home_joints.split(";")
    homes = [tuple(float(v) for v in h.strip().split(",")) for h in homes_raw]

    gripper_io_list = [x.strip() for x in req.gripper_ios.split(",") if x.strip()]

    mgs: list[MotionGroup] = []
    for mg_id in mg_ids:
        ctrl_name = mg_id.rsplit("@", maxsplit=1)[-1]
        ctrl = await cell.controller(ctrl_name)
        mgs.append(ctrl.motion_group(mg_id))

    feature_map = FeatureMap(
        groups=[
            FeatureGroup(
                motion_group=mgs[0],
                name="left",
                joint_key="left_arm",
                tcp_key="left_eef_9d",
                tcp_format=TcpFormat.ROT6D,
                ios={"left_gripper": gripper_io_list[0]},
            ),
            FeatureGroup(
                motion_group=mgs[1],
                name="right",
                joint_key="right_arm",
                tcp_key="right_eef_9d",
                tcp_format=TcpFormat.ROT6D,
                ios={"right_gripper": gripper_io_list[1]},
            ),
        ]
    )

    cameras: Any = None
    if req.camera_server and req.cameras:
        cam_configs = {
            cam.name: WebRTCCameraConfig(
                api_url=req.camera_server,
                device_id=cam.device_id,
                width=cam.width,
                height=cam.height,
                fps=cam.fps,
            )
            for cam in req.cameras
        }
        cameras = CameraSet(configs=cam_configs)

    gr00t_client = Gr00tPolicyClient(
        host=req.policy_host,
        port=req.policy_port,
        feature_map=feature_map,
        language=req.language,
    )

    _executor = PolicyExecutor(
        feature_map=feature_map,
        cameras=cameras,
        policy=gr00t_client,
        timeout_s=req.timeout_s,
    )

    async def run() -> ExecutionResult:
        try:
            # Move to home (fast)
            fast = MotionSettings(tcp_velocity_limit=500.0)
            tasks = []
            for mg, home in zip(mgs, homes, strict=False):
                tcp = (await mg.tcp_names())[0]
                traj = await mg.plan([joint_ptp(home, settings=fast)], tcp)
                tasks.append(mg.execute(traj, tcp, actions=[joint_ptp(home, settings=fast)]))
            await asyncio.gather(*tasks)
            return await _executor.run()
        except Exception as e:
            logger.exception("Run task failed")
            _executor.status.message = f"Error: {e}"
            raise

    _run_task = asyncio.create_task(run())
    return _executor.status


@app.post("/stop", response_model=ExecutorStatus)
async def stop() -> ExecutorStatus:
    global _executor, _nova, _run_task

    if _executor is not None:
        _executor.stop()

    if _run_task is not None:
        with contextlib.suppress(asyncio.CancelledError, Exception):
            await _run_task
        _run_task = None

    if _nova is not None:
        with contextlib.suppress(Exception):
            await _nova.close()
        _nova = None

    _executor = None
    return ExecutorStatus()


@app.get("/app_icon.png", include_in_schema=False)
async def app_icon() -> FileResponse:
    return FileResponse("static/app_icon.png", media_type="image/png")


def main() -> None:
    uvicorn.run(app, host="0.0.0.0", port=3000, log_level="info")  # noqa: S104


if __name__ == "__main__":
    main()
