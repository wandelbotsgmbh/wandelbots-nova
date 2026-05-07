"""GR00T robot controller — runs policy via ZMQ against a GR00T-compatible server.

Registers as a Nova program via novax so it appears in the program operator.
Also provides REST endpoints for manual control:
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
from novax import Novax
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

import nova
from nova import api
from nova.actions import joint_ptp
from nova.cell import virtual_controller
from nova.cell.motion_group import MotionGroup
from nova.events import Cycle
from nova.program import ProgramPreconditions
from nova.types import MotionSettings

if TYPE_CHECKING:
    from policy.executor import ExecutionResult

logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)

BASE_PATH = config("BASE_PATH", default="", cast=str)
CAMERA_SERVER = config("CAMERA_SERVER", default="", cast=str)
GROOT_HOST = config("GROOT_HOST", default="app-mock-groot-policy-service", cast=str)
GROOT_PORT = config("GROOT_PORT", default=5555, cast=int)


# ---------------------------------------------------------------------------
# Nova program (appears in program operator)
# ---------------------------------------------------------------------------


@nova.program(
    id="groot_policy_controller",
    name="GR00T Policy Controller",
    description="Run a GR00T policy on two UR10e robots via ZMQ.",
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
async def groot_policy_controller(
    ctx: nova.ProgramContext,
    policy_host: str = Field(default="172.31.11.129", description="GR00T ZMQ server hostname"),
    policy_port: int = Field(default=30555, description="GR00T ZMQ server port"),
    language: str = Field(
        default="Coordinate both arms and move smoothly.",
        description="Language instruction sent with every observation",
    ),
    timeout_s: float = Field(default=10.0, description="Execution timeout in seconds (0 = unlimited)"),
    motion_groups: str = Field(
        default="0@ur10e,0@ur10e-2",
        description="Comma-separated motion group IDs",
    ),
    home_joints: str = Field(
        default="0,-1.571,1.571,-1.571,-1.571,0;0,-1.571,-1.571,-1.571,1.571,0",
        description="Semicolon-separated home joint positions per motion group",
    ),
    gripper_ios: str = Field(
        default="digital_out[0],digital_out[0]",
        description="Comma-separated gripper IO key per motion group",
    ),
    camera_server: str = Field(
        default="",
        description="WebRTC camera server URL (empty = no cameras)",
    ),
    camera_devices: str = Field(
        default="",
        description="Comma-separated camera entries as name:device_id (e.g. 'exterior_image_1_left:315122271048')",
    ),
    camera_width: int = Field(default=224, description="Camera frame width"),
    camera_height: int = Field(default=224, description="Camera frame height"),
    camera_fps: int = Field(default=15, description="Camera frames per second"),
):
    """Run one GR00T policy episode: home → ZMQ inference → timeout."""
    cell = ctx.nova.cell()
    cycle = Cycle(cell=cell, extra={"program": "groot_policy_controller"})

    mg_ids = [mg.strip() for mg in motion_groups.split(",")]
    homes_raw = home_joints.split(";") if ";" in home_joints else [home_joints] * len(mg_ids)
    homes = [tuple(float(v) for v in h.strip().split(",")) for h in homes_raw]

    mgs: list[MotionGroup] = []
    for mg_id in mg_ids:
        ctrl_name = mg_id.rsplit("@", maxsplit=1)[-1]
        ctrl = await cell.controller(ctrl_name)
        mgs.append(ctrl.motion_group(mg_id))

    # Move to home
    fast = MotionSettings(tcp_velocity_limit=500.0)
    home_tasks = []
    for mg, home in zip(mgs, homes, strict=False):
        tcp = (await mg.tcp_names())[0]
        traj = await mg.plan([joint_ptp(home, settings=fast)], tcp)
        home_tasks.append(mg.execute(traj, tcp, actions=[joint_ptp(home, settings=fast)]))
    await asyncio.gather(*home_tasks)

    gripper_io_list = [x.strip() for x in gripper_ios.split(",") if x.strip()]
    feature_map = FeatureMap(groups=[
        FeatureGroup(
            motion_group=mgs[0], name="left",
            joint_key="left_arm", tcp_key="left_eef_9d", tcp_format=TcpFormat.ROT6D,
            ios={"left_gripper": gripper_io_list[0]} if gripper_io_list else None,
        ),
        FeatureGroup(
            motion_group=mgs[1], name="right",
            joint_key="right_arm", tcp_key="right_eef_9d", tcp_format=TcpFormat.ROT6D,
            ios={"right_gripper": gripper_io_list[1]} if len(gripper_io_list) > 1 else None,
        ),
    ])

    # Build cameras if configured
    cameras: CameraSet | None = None
    if camera_server and camera_devices:
        devices = {}
        for raw_entry in camera_devices.split(","):
            entry = raw_entry.strip()
            if ":" in entry:
                name, device_id = entry.split(":", 1)
                devices[name.strip()] = device_id.strip()
        if devices:
            cameras = CameraSet(
                api_url=camera_server,
                devices=devices,
                width=camera_width,
                height=camera_height,
                fps=camera_fps,
            )

    client = Gr00tPolicyClient(
        host=policy_host, port=policy_port,
        language=language,
    )

    executor = PolicyExecutor(
        feature_map=feature_map,
        cameras=cameras,
        policy=client,
        timeout_s=timeout_s,
    )

    await cycle.start()
    try:
        result = await executor.run()
        await cycle.finish()
        logger.info("Program finished: reason=%s steps=%d", result.reason, result.steps)
    except Exception as e:
        await cycle.fail(e)
        raise


# ---------------------------------------------------------------------------
# FastAPI app with novax
# ---------------------------------------------------------------------------

novax_app = Novax()

app = FastAPI(
    title="GR00T Robot Controller",
    version="0.4.0",
    root_path=BASE_PATH,
    docs_url="/",
)

novax_app.include_programs_router(app)
novax_app.register_program(groot_policy_controller)

app.add_middleware(
    CORSMiddleware,
    allow_origins=["*"],
    allow_credentials=True,
    allow_methods=["*"],
    allow_headers=["*"],
)


# ---------------------------------------------------------------------------
# Manual REST control
# ---------------------------------------------------------------------------

_executor: PolicyExecutor | None = None
_nova_instance: nova.Nova | None = None
_run_task: asyncio.Task[ExecutionResult] | None = None


class CameraConfigModel(BaseModel):
    name: str
    device_id: str
    width: int = 640
    height: int = 480
    fps: int = 30


class StartRequest(BaseModel):
    policy_host: str = Field(default=GROOT_HOST)
    policy_port: int = Field(default=GROOT_PORT)
    motion_groups: str = Field(default="0@ur10e,0@ur10e-2")
    home_joints: str = Field(
        default="0,-1.571,1.571,-1.571,-1.571,0;0,-1.571,-1.571,-1.571,1.571,0",
    )
    gripper_ios: str = Field(default="digital_out[0],digital_out[0]")
    language: str = Field(default="Coordinate both arms and move smoothly.")
    timeout_s: float = Field(default=10.0, ge=0)
    camera_server: str = Field(default=CAMERA_SERVER)
    cameras: list[CameraConfigModel] = Field(default=[])


@app.get("/status", response_model=ExecutorStatus)
async def get_status() -> ExecutorStatus:
    if _executor is None:
        return ExecutorStatus()
    return _executor.status


@app.post("/start", response_model=ExecutorStatus)
async def start(req: StartRequest = StartRequest()) -> ExecutorStatus:
    """Move to home, then run GR00T policy until timeout or /stop."""
    global _executor, _nova_instance, _run_task

    if _executor is not None and _executor.phase != "IDLE":
        return _executor.status

    _nova_instance = nova.Nova()
    await _nova_instance.open()
    cell = _nova_instance.cell()

    mg_ids = [mg.strip() for mg in req.motion_groups.split(",") if mg.strip()]
    homes_raw = req.home_joints.split(";")
    homes = [tuple(float(v) for v in h.strip().split(",")) for h in homes_raw]
    gripper_io_list = [x.strip() for x in req.gripper_ios.split(",") if x.strip()]

    mgs: list[MotionGroup] = []
    for mg_id in mg_ids:
        ctrl_name = mg_id.rsplit("@", maxsplit=1)[-1]
        ctrl = await cell.controller(ctrl_name)
        mgs.append(ctrl.motion_group(mg_id))

    feature_map = FeatureMap(groups=[
        FeatureGroup(
            motion_group=mgs[0], name="left",
            joint_key="left_arm", tcp_key="left_eef_9d", tcp_format=TcpFormat.ROT6D,
            ios={"left_gripper": gripper_io_list[0] if gripper_io_list else "digital_out[0]"},
        ),
        FeatureGroup(
            motion_group=mgs[1], name="right",
            joint_key="right_arm", tcp_key="right_eef_9d", tcp_format=TcpFormat.ROT6D,
            ios={"right_gripper": gripper_io_list[1] if len(gripper_io_list) > 1 else "digital_out[0]"},
        ),
    ])

    cameras: Any = None
    if req.camera_server and req.cameras:
        cam_configs = {
            cam.name: WebRTCCameraConfig(
                api_url=req.camera_server, device_id=cam.device_id,
                width=cam.width, height=cam.height, fps=cam.fps,
            )
            for cam in req.cameras
        }
        cameras = CameraSet(configs=cam_configs)

    _executor = PolicyExecutor(
        feature_map=feature_map,
        cameras=cameras,
        policy=Gr00tPolicyClient(host=req.policy_host, port=req.policy_port, language=req.language),
        timeout_s=req.timeout_s,
    )

    async def run() -> ExecutionResult:
        fast = MotionSettings(tcp_velocity_limit=500.0)
        tasks = []
        for mg, home in zip(mgs, homes, strict=False):
            tcp = (await mg.tcp_names())[0]
            traj = await mg.plan([joint_ptp(home, settings=fast)], tcp)
            tasks.append(mg.execute(traj, tcp, actions=[joint_ptp(home, settings=fast)]))
        await asyncio.gather(*tasks)
        return await _executor.run()

    _run_task = asyncio.create_task(run())
    return _executor.status


@app.post("/stop", response_model=ExecutorStatus)
async def stop() -> ExecutorStatus:
    global _executor, _nova_instance, _run_task

    if _executor is not None:
        _executor.stop()
    if _run_task is not None:
        with contextlib.suppress(asyncio.CancelledError, Exception):
            await _run_task
        _run_task = None
    if _nova_instance is not None:
        with contextlib.suppress(Exception):
            await _nova_instance.close()
        _nova_instance = None
    _executor = None
    return ExecutorStatus()


@app.get("/app_icon.png", include_in_schema=False)
async def app_icon() -> FileResponse:
    return FileResponse("static/app_icon.png", media_type="image/png")


def main() -> None:
    uvicorn.run(app, host="0.0.0.0", port=3000, log_level="info")  # noqa: S104


if __name__ == "__main__":
    main()
