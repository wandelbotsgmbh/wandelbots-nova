"""Policy Robot Controller — runs policy episodes via PID jogging over NATS.

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
from typing import TYPE_CHECKING

import uvicorn
from decouple import config
from fastapi import FastAPI
from fastapi.middleware.cors import CORSMiddleware
from fastapi.responses import FileResponse
from novax import Novax
from policy import (
    BoolMapping,
    NatsPolicyClient,
    Observation,
    PolicyExecutor,
    PolicySchema,
    WebRTCCameras,
)
from policy.executor import ExecutorStatus
from pydantic import BaseModel, Field

import nats
import nova
from nova import api
from nova.actions import joint_ptp
from nova.cell import virtual_controller
from nova.events import Cycle
from nova.program import ProgramPreconditions
from nova.types import MotionSettings

if TYPE_CHECKING:
    from policy.executor import ExecutionResult

logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)

BASE_PATH = config("BASE_PATH", default="", cast=str)
NATS_BROKER = config("NATS_BROKER", default=None)
NATS_SUBJECT = config(
    "NATS_SUBJECT",
    default="nova.v2.cells.cell.apps.mock-policy-service.predict",
    cast=str,
)
CAMERA_SERVER = config("CAMERA_SERVER", default="", cast=str)

HOME_LEFT = "1.047,-0.698,1.745,-3.142,0.873,2.094"
HOME_RIGHT = "-1.047,-2.356,-1.745,0.0,-0.873,-2.094"


# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------


def _parse_homes(home_joints: str, mg_ids: list[str]) -> list[tuple[float, ...]]:
    homes_raw = home_joints.split(";") if ";" in home_joints else [home_joints] * len(mg_ids)
    return [tuple(float(v) for v in h.strip().split(",")) for h in homes_raw]


async def _move_to_home(mgs, homes) -> None:
    fast = MotionSettings(tcp_velocity_limit=500.0)
    tasks = []
    for mg, home in zip(mgs, homes, strict=False):
        tcp = (await mg.tcp_names())[0]
        traj = await mg.plan([joint_ptp(home, settings=fast)], tcp)
        tasks.append(mg.execute(traj, tcp, actions=[joint_ptp(home, settings=fast)]))
    await asyncio.gather(*tasks)


def _build_schema(
    mgs,
    *,
    gripper_ios: str = "",
    camera_server: str = "",
    camera_devices: str = "",
    camera_width: int = 640,
    camera_height: int = 480,
    camera_fps: int = 15,
) -> PolicySchema:
    observations = []

    # Joint positions per arm
    for i, mg in enumerate(mgs):
        observations.append(Observation.joint_positions(f"arm_{i}_joints", source=mg))

    # Gripper IOs
    gripper_io_list = [x.strip() for x in gripper_ios.split(",") if x.strip()]
    for i, mg in enumerate(mgs):
        if i < len(gripper_io_list):
            observations.append(
                Observation.io(
                    f"arm_{i}_gripper", source=mg, io=gripper_io_list[i],
                    mapping=BoolMapping(on=100.0),
                )
            )

    # Cameras
    if camera_server and camera_devices:
        cameras = WebRTCCameras(
            api_url=camera_server, width=camera_width, height=camera_height, fps=camera_fps,
        )
        for raw_entry in camera_devices.split(","):
            entry = raw_entry.strip()
            if ":" in entry:
                name, device_id = entry.split(":", 1)
                observations.append(
                    Observation.image(name.strip(), source=cameras.device(device_id.strip()))
                )

    return PolicySchema(observations=observations)


# ---------------------------------------------------------------------------
# Nova program (appears in program operator)
# ---------------------------------------------------------------------------


@nova.program(
    id="nats_policy_controller",
    name="NATS Policy Controller",
    description="Run a policy via NATS on two UR5e robots with optional cameras.",
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
async def nats_policy_controller(
    ctx: nova.ProgramContext,
    nats_subject: str = Field(
        default="nova.v2.cells.cell.apps.mock-policy-service.predict",
        description="NATS subject the policy service listens on",
    ),
    timeout_s: float = Field(default=10.0, description="Execution timeout in seconds (0 = unlimited)"),
    motion_groups: str = Field(
        default="0@ur5e-left,0@ur5e-right",
        description="Comma-separated motion group IDs",
    ),
    home_joints: str = Field(
        default=f"{HOME_LEFT};{HOME_RIGHT}",
        description="Semicolon-separated home joint positions per motion group",
    ),
    gripper_ios: str = Field(
        default="digital_out[0],digital_out[0]",
        description="Comma-separated gripper IO key per motion group",
    ),
    camera_server: str = Field(default="", description="WebRTC camera server URL (empty = no cameras)"),
    camera_devices: str = Field(
        default="",
        description="Comma-separated camera entries as name:device_id",
    ),
    camera_width: int = Field(default=640, description="Camera frame width"),
    camera_height: int = Field(default=480, description="Camera frame height"),
    camera_fps: int = Field(default=15, description="Camera frames per second"),
):
    """Run one policy episode: home → connect NATS → run policy until timeout."""
    cell = ctx.nova.cell()
    cycle = Cycle(cell=cell, extra={"program": "nats_policy_controller"})

    mg_ids = [mg.strip() for mg in motion_groups.split(",")]
    homes = _parse_homes(home_joints, mg_ids)

    mgs = []
    for mg_id in mg_ids:
        ctrl_name = mg_id.rsplit("@", maxsplit=1)[-1]
        ctrl = await cell.controller(ctrl_name)
        mgs.append(ctrl.motion_group(mg_id))

    await _move_to_home(mgs, homes)

    nats_config = ctx.nova.config.nats_client_config or {}
    nats_url = NATS_BROKER or nats_config.get("servers", "nats://localhost:4222")
    nc = await nats.connect(servers=nats_url, max_reconnect_attempts=10)

    try:
        schema = _build_schema(
            mgs,
            gripper_ios=gripper_ios,
            camera_server=camera_server,
            camera_devices=camera_devices,
            camera_width=camera_width,
            camera_height=camera_height,
            camera_fps=camera_fps,
        )

        executor = PolicyExecutor(
            schema,
            NatsPolicyClient(nc, subject=nats_subject, timeout=5.0),
            timeout_s=timeout_s,
        )

        await cycle.start()
        result = await executor.run()
        await cycle.finish()
        logger.info("Program finished: reason=%s steps=%d", result.reason, result.steps)
    except Exception as e:
        await cycle.fail(e)
        raise
    finally:
        await nc.drain()


# ---------------------------------------------------------------------------
# FastAPI app with novax
# ---------------------------------------------------------------------------

novax_app = Novax()

app = FastAPI(
    title="Policy Robot Controller",
    version="2.0.0",
    root_path=BASE_PATH,
    docs_url="/",
)

novax_app.include_programs_router(app)
novax_app.register_program(nats_policy_controller)

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
_nats_client: nats.aio.client.Client | None = None
_run_task: asyncio.Task[ExecutionResult] | None = None


class CameraConfig(BaseModel):
    name: str
    device_id: str


class StartRequest(BaseModel):
    nats_subject: str = Field(default=NATS_SUBJECT)
    motion_groups: str = Field(default="0@ur5e-left,0@ur5e-right")
    home_joints: str = Field(default=f"{HOME_LEFT};{HOME_RIGHT}")
    timeout_s: float = Field(default=10.0, ge=0)
    gripper_ios: str = Field(default="digital_out[0],digital_out[0]")
    camera_server: str = Field(default=CAMERA_SERVER)
    cameras: list[CameraConfig] = Field(default=[])
    camera_width: int = Field(default=640)
    camera_height: int = Field(default=480)
    camera_fps: int = Field(default=15)


@app.get("/status", response_model=ExecutorStatus)
async def get_status():
    if _executor is None:
        return ExecutorStatus()
    return _executor.status


@app.post("/start", response_model=ExecutorStatus)
async def start(req: StartRequest = StartRequest()):
    """Move to home, connect cameras, then run policy. Returns immediately."""
    global _executor, _nova_instance, _nats_client, _run_task

    if _executor is not None and _executor.phase != "IDLE":
        return _executor.status

    _nova_instance = nova.Nova()
    await _nova_instance.open()
    cell = _nova_instance.cell()

    if NATS_BROKER:
        _nats_client = await nats.connect(servers=NATS_BROKER, max_reconnect_attempts=10)
    else:
        nats_config = _nova_instance.config.nats_client_config or {}
        nats_url = nats_config.get("servers", "nats://localhost:4222")
        _nats_client = await nats.connect(servers=nats_url, max_reconnect_attempts=10)

    mg_ids = [mg.strip() for mg in req.motion_groups.split(",")]
    homes = _parse_homes(req.home_joints, mg_ids)

    mgs = []
    for mg_id in mg_ids:
        ctrl_name = mg_id.rsplit("@", maxsplit=1)[-1]
        ctrl = await cell.controller(ctrl_name)
        mgs.append(ctrl.motion_group(mg_id))

    # Build camera_devices string from camera list
    camera_devices = ",".join(f"{c.name}:{c.device_id}" for c in req.cameras)

    schema = _build_schema(
        mgs,
        gripper_ios=req.gripper_ios,
        camera_server=req.camera_server,
        camera_devices=camera_devices,
        camera_width=req.camera_width,
        camera_height=req.camera_height,
        camera_fps=req.camera_fps,
    )

    _executor = PolicyExecutor(
        schema,
        NatsPolicyClient(_nats_client, subject=req.nats_subject, timeout=5.0),
        timeout_s=req.timeout_s,
    )

    async def run() -> ExecutionResult:
        try:
            await _move_to_home(mgs, homes)
            return await _executor.run()
        except Exception as e:
            logger.exception("Run task failed")
            if _executor is not None:
                _executor.status.message = f"{type(e).__name__}: {e}"
            raise

    _run_task = asyncio.create_task(run())
    return _executor.status


@app.post("/stop", response_model=ExecutorStatus)
async def stop():
    global _executor, _nova_instance, _nats_client, _run_task

    if _executor is not None:
        _executor.stop()
    if _run_task is not None:
        with contextlib.suppress(asyncio.CancelledError, Exception):
            await _run_task
        _run_task = None
    if _nats_client is not None:
        with contextlib.suppress(Exception):
            await _nats_client.drain()
        _nats_client = None
    if _nova_instance is not None:
        with contextlib.suppress(Exception):
            await _nova_instance.close()
        _nova_instance = None
    _executor = None
    return ExecutorStatus()


@app.get("/app_icon.png", include_in_schema=False)
async def app_icon():
    return FileResponse("static/app_icon.png", media_type="image/png")


def main():
    uvicorn.run(app, host="0.0.0.0", port=3000, log_level="info")  # noqa: S104


if __name__ == "__main__":
    main()
