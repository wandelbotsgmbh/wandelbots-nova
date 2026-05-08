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
from typing import TYPE_CHECKING

import uvicorn
from decouple import config
from fastapi import FastAPI
from fastapi.middleware.cors import CORSMiddleware
from fastapi.responses import FileResponse
from novax import Novax
from policy import (
    BoolMapping,
    Gr00tPolicyClient,
    Observation,
    PolicyExecutor,
    PolicySchema,
    TcpFormat,
    WebRTCCameras,
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
    mgs: list[MotionGroup],
    *,
    gripper_ios: str = "",
    language: str = "",
    camera_server: str = "",
    camera_devices: str = "",
    camera_width: int = 224,
    camera_height: int = 224,
    camera_fps: int = 15,
) -> PolicySchema:
    observations = [
        Observation.joint_positions("left_arm", source=mgs[0]),
        Observation.tcp("left_eef_9d", source=mgs[0], format=TcpFormat.ROT6D),
        Observation.joint_positions("right_arm", source=mgs[1]),
        Observation.tcp("right_eef_9d", source=mgs[1], format=TcpFormat.ROT6D),
    ]

    gripper_io_list = [x.strip() for x in gripper_ios.split(",") if x.strip()]
    if gripper_io_list:
        observations.append(
            Observation.io(
                "left_gripper", source=mgs[0], io=gripper_io_list[0],
                mapping=BoolMapping(on=100.0),
            )
        )
    if len(gripper_io_list) > 1:
        observations.append(
            Observation.io(
                "right_gripper", source=mgs[1], io=gripper_io_list[1],
                mapping=BoolMapping(on=100.0),
            )
        )

    if language:
        observations.append(Observation.constant("language", value=language))

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
    id="groot_policy_controller",
    name="GR00T Policy Controller",
    description="Run a GR00T policy on two UR5e robots via ZMQ.",
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
    camera_devices: str = Field(default="", description="Comma-separated camera entries as name:device_id"),
    camera_width: int = Field(default=224, description="Camera frame width"),
    camera_height: int = Field(default=224, description="Camera frame height"),
    camera_fps: int = Field(default=15, description="Camera frames per second"),
):
    """Run one GR00T policy episode: home → ZMQ inference → timeout."""
    cell = ctx.nova.cell()
    cycle = Cycle(cell=cell, extra={"program": "groot_policy_controller"})

    mg_ids = [mg.strip() for mg in motion_groups.split(",")]
    homes = _parse_homes(home_joints, mg_ids)

    mgs: list[MotionGroup] = []
    for mg_id in mg_ids:
        ctrl_name = mg_id.rsplit("@", maxsplit=1)[-1]
        ctrl = await cell.controller(ctrl_name)
        mgs.append(ctrl.motion_group(mg_id))

    await _move_to_home(mgs, homes)

    schema = _build_schema(
        mgs,
        gripper_ios=gripper_ios,
        language=language,
        camera_server=camera_server,
        camera_devices=camera_devices,
        camera_width=camera_width,
        camera_height=camera_height,
        camera_fps=camera_fps,
    )

    client = Gr00tPolicyClient(
        host=policy_host, port=policy_port, language=language,
    )

    executor = PolicyExecutor(schema, client, timeout_s=timeout_s)

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
    version="2.0.0",
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


class StartRequest(BaseModel):
    policy_host: str = Field(default=GROOT_HOST)
    policy_port: int = Field(default=GROOT_PORT)
    motion_groups: str = Field(default="0@ur5e-left,0@ur5e-right")
    home_joints: str = Field(default=f"{HOME_LEFT};{HOME_RIGHT}")
    gripper_ios: str = Field(default="digital_out[0],digital_out[0]")
    language: str = Field(default="Coordinate both arms and move smoothly.")
    timeout_s: float = Field(default=10.0, ge=0)
    camera_server: str = Field(default=CAMERA_SERVER)
    camera_devices: str = Field(default="")
    camera_width: int = Field(default=224)
    camera_height: int = Field(default=224)
    camera_fps: int = Field(default=15)


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
    homes = _parse_homes(req.home_joints, mg_ids)

    mgs: list[MotionGroup] = []
    for mg_id in mg_ids:
        ctrl_name = mg_id.rsplit("@", maxsplit=1)[-1]
        ctrl = await cell.controller(ctrl_name)
        mgs.append(ctrl.motion_group(mg_id))

    schema = _build_schema(
        mgs,
        gripper_ios=req.gripper_ios,
        language=req.language,
        camera_server=req.camera_server,
        camera_devices=req.camera_devices,
        camera_width=req.camera_width,
        camera_height=req.camera_height,
        camera_fps=req.camera_fps,
    )

    _executor = PolicyExecutor(
        schema,
        Gr00tPolicyClient(host=req.policy_host, port=req.policy_port, language=req.language),
        timeout_s=req.timeout_s,
    )

    async def run() -> ExecutionResult:
        await _move_to_home(mgs, homes)
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
