"""GR00T Dual-Arm Controller — runs GR00T policy on two UR5e arms via ZMQ.

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

from decouple import config
from fastapi import FastAPI
from fastapi.middleware.cors import CORSMiddleware
from fastapi.responses import FileResponse
from novax import Novax
from pydantic import BaseModel, Field
import uvicorn

import nova
from nova import api
from nova.actions import joint_ptp
from nova.cell import virtual_controller
from nova.events import Cycle
from nova.program import ProgramPreconditions
from nova.types import MotionSettings
from policy import (
    Gr00tPolicyClient,
    Observation,
    PidConfig,
    PolicyExecutor,
    PolicySchema,
    WebRTCCameras,
)
from policy.executor import ExecutorStatus

if TYPE_CHECKING:
    from policy.executor import ExecutionResult

logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)

BASE_PATH = config("BASE_PATH", default="", cast=str)
GROOT_HOST = config("GROOT_HOST", default="172.31.11.129", cast=str)
GROOT_PORT = config("GROOT_PORT", default=30555, cast=int)
CAMERA_SERVER = config(
    "CAMERA_SERVER", default="http://172.31.11.129:8011/webrtc-streamer", cast=str
)

# Isaac Sim camera device IDs
CAM_CONTEXT = "World_EnvAssets_rack_env0__3_00_intel_d456_screw_adapter_asm_tn__03_00_intel_d456_screw_adapterasm_io0_D456_Solid_context_camera_rack_env0"
CAM_TARGET = "World_EnvAssets_rack_env0_d405_halter_stationaer_2_asm_01_tn__d405_halter_stationaer_2asm_ff0_D405_Solid_target_camera_rack_env0"
CAM_LEFT_WRIST = "World_Robot_Robot_0_L__0_00_robotics_usecase_gripper_asm_tn__00_00_robotics_usecase_gripper_asm__tn__01_00_CAMERA_ASMBLY__INTEL_D405_D405_SOLID_left_wrist_camera_env0"
CAM_RIGHT_WRIST = "World_Robot_Robot_0_R__0_00_robotics_usecase_gripper_asm_tn__00_00_robotics_usecase_gripper_asm__tn__01_00_CAMERA_ASMBLY__INTEL_D405_D405_SOLID_right_wrist_camera_env0"

HOME_LEFT = "1.169,-0.733,1.745,-3.054,0.872,2.094"
HOME_RIGHT = "-1.169,-2.3911,-1.8675,0.0,-0.872,-2.094"
DEFAULT_CAMERAS = f"exterior_image_1:{CAM_CONTEXT},exterior_image_2:{CAM_TARGET},left_wrist_image:{CAM_LEFT_WRIST},right_wrist_image:{CAM_RIGHT_WRIST}"


# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------


def _parse_homes(home_joints: str) -> tuple[tuple[float, ...], tuple[float, ...]]:
    homes_raw = home_joints.split(";")
    return (
        tuple(float(v) for v in homes_raw[0].strip().split(",")),
        tuple(float(v) for v in homes_raw[1].strip().split(",")),
    )


async def _move_to_home(mg_left, mg_right, home_left, home_right) -> None:
    fast = MotionSettings(tcp_velocity_limit=500.0)
    tcp_left = (await mg_left.tcp_names())[0]
    tcp_right = (await mg_right.tcp_names())[0]
    t1 = await mg_left.plan([joint_ptp(home_left, settings=fast)], tcp_left)
    t2 = await mg_right.plan([joint_ptp(home_right, settings=fast)], tcp_right)
    await asyncio.gather(
        mg_left.execute(t1, tcp_left, actions=[joint_ptp(home_left, settings=fast)]),
        mg_right.execute(t2, tcp_right, actions=[joint_ptp(home_right, settings=fast)]),
    )


def _build_schema(
    mg_left,
    mg_right,
    *,
    camera_server: str = "",
    camera_devices: str = "",
    camera_size: int = 224,
    camera_fps: int = 15,
    language: str = "",
) -> PolicySchema:
    observations = [
        Observation.joint_positions("left_joint_positions", source=mg_left),
        Observation.joint_positions("right_joint_positions", source=mg_right),
    ]

    if language:
        observations.append(Observation.constant("language", value=language))

    if camera_server and camera_devices:
        cameras = WebRTCCameras(api_url=camera_server, frame_history=1, resize=(256, 256))
        for raw_entry in camera_devices.split(","):
            entry = raw_entry.strip()
            if ":" in entry:
                name, device_id = entry.split(":", 1)
                observations.append(
                    Observation.image(name.strip(), source=cameras.device(device_id.strip()))
                )

    return PolicySchema(observations=observations)


# ---------------------------------------------------------------------------
# Nova programs (appear in program operator)
# ---------------------------------------------------------------------------


@nova.program(
    id="move_to_home",
    name="Move to Home",
    description="Move both UR5e arms to their home positions.",
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
async def move_to_home(
    ctx: nova.ProgramContext,
    home_joints: str = Field(
        default=f"{HOME_LEFT};{HOME_RIGHT}",
        description="Semicolon-separated home joint positions (left;right) in radians",
    ),
):
    """Move both arms to home and stop."""
    cell = ctx.nova.cell()
    home_left, home_right = _parse_homes(home_joints)
    mg_left = (await cell.controller("ur5e-left"))[0]
    mg_right = (await cell.controller("ur5e-right"))[0]
    await _move_to_home(mg_left, mg_right, home_left, home_right)
    logger.info("Both arms at home.")


@nova.program(
    id="gr00t_dual_arm_controller",
    name="GR00T Dual-Arm Controller",
    description="Run GR00T N1.7 policy on two UR5e arms with 4 cameras via ZMQ.",
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
async def gr00t_dual_arm_controller(
    ctx: nova.ProgramContext,
    groot_host: str = Field(default="172.31.11.129", description="GR00T ZMQ server hostname"),
    groot_port: int = Field(default=30555, description="GR00T ZMQ server port"),
    language: str = Field(
        default="Pick up the box and place it onto the conveyor.",
        description="Language instruction sent with every observation",
    ),
    timeout_s: float = Field(
        default=120.0, description="Execution timeout in seconds (0 = unlimited)"
    ),
    home_joints: str = Field(
        default=f"{HOME_LEFT};{HOME_RIGHT}",
        description="Semicolon-separated home joint positions (left;right) in radians",
    ),
    camera_server: str = Field(
        default="http://172.31.11.129:8011/webrtc-streamer",
        description="WebRTC camera server URL",
    ),
    camera_devices: str = Field(
        default=DEFAULT_CAMERAS,
        description="Comma-separated camera entries as groot_key:device_id",
    ),
    camera_size: int = Field(default=224, description="Camera frame width and height"),
    camera_fps: int = Field(default=15, description="Camera frames per second"),
    p_gain: float = Field(default=3.0, description="PID proportional gain"),
    i_gain: float = Field(default=0.0, description="PID integral gain"),
    d_gain: float = Field(default=0.15, description="PID derivative gain"),
    ff_gain: float = Field(default=1.0, description="Feedforward gain (0=off, 1=full)"),
    velocity_limit: float = Field(default=2.0, description="Joint velocity limit in rad/s"),
    lookahead_ms: float = Field(
        default=50.0, description="Lookahead in ms to compensate network latency"
    ),
):
    """Run one GR00T episode: home → connect cameras → run policy until timeout."""
    cell = ctx.nova.cell()
    cycle = Cycle(cell=cell, extra={"program": "gr00t_dual_arm_controller"})

    home_left, home_right = _parse_homes(home_joints)

    ctrl_left = await cell.controller("ur5e-left")
    ctrl_right = await cell.controller("ur5e-right")
    mg_left = ctrl_left[0]
    mg_right = ctrl_right[0]

    await _move_to_home(mg_left, mg_right, home_left, home_right)

    schema = _build_schema(
        mg_left,
        mg_right,
        camera_server=camera_server,
        camera_devices=camera_devices,
        camera_size=camera_size,
        camera_fps=camera_fps,
        language=language,
    )

    client = Gr00tPolicyClient(
        host=groot_host,
        port=groot_port,
        timeout_ms=60000,
        dt_ms=66.7,  # match training data rate (15 Hz)
    )

    pid = PidConfig(
        p_gain=p_gain,
        i_gain=i_gain,
        d_gain=d_gain,
        ff_gain=ff_gain,
        velocity_limit=velocity_limit,
        lookahead_ms=lookahead_ms,
    )
    executor = PolicyExecutor(schema, client, timeout_s=timeout_s, motion=pid)

    await cycle.start()
    try:
        result = await executor.run()
        await cycle.finish()
        logger.info(
            "Episode finished: reason=%s steps=%d duration=%.1fs",
            result.reason,
            result.steps,
            result.duration_s,
        )
    except Exception as e:
        await cycle.fail(e)
        raise


# ---------------------------------------------------------------------------
# FastAPI app with novax
# ---------------------------------------------------------------------------

novax_app = Novax()

app = FastAPI(
    title="GR00T Dual-Arm Controller",
    version="2.0.0",
    root_path=BASE_PATH,
    docs_url="/",
)

novax_app.include_programs_router(app)
novax_app.register_program(move_to_home)
novax_app.register_program(gr00t_dual_arm_controller)

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
_last_error: str = ""


class StartRequest(BaseModel):
    groot_host: str = Field(default=GROOT_HOST)
    groot_port: int = Field(default=GROOT_PORT)
    language: str = Field(default="Pick up the box and place it onto the conveyor.")
    timeout_s: float = Field(default=120.0, ge=0)
    home_joints: str = Field(default=f"{HOME_LEFT};{HOME_RIGHT}")
    camera_server: str = Field(default=CAMERA_SERVER)
    camera_devices: str = Field(default=DEFAULT_CAMERAS)
    camera_size: int = Field(default=224)
    camera_fps: int = Field(default=15)
    # PID tuning
    p_gain: float = Field(default=3.0, description="PID proportional gain")
    i_gain: float = Field(default=0.0, description="PID integral gain")
    d_gain: float = Field(default=0.15, description="PID derivative gain")
    ff_gain: float = Field(default=1.0, description="Feedforward gain (0=off, 1=full)")
    velocity_limit: float = Field(default=2.0, description="Joint velocity limit in rad/s")
    lookahead_ms: float = Field(
        default=50.0, description="Lookahead in ms to compensate network latency"
    )


@app.get("/status", response_model=ExecutorStatus)
async def get_status():
    if _executor is None:
        status = ExecutorStatus()
        if _last_error:
            status.message = _last_error
        return status
    return _executor.status


@app.post("/home", response_model=ExecutorStatus)
async def home(home_joints: str = f"{HOME_LEFT};{HOME_RIGHT}"):
    """Move both arms to home positions."""
    global _nova_instance, _last_error
    _last_error = ""
    try:
        n = nova.Nova()
        await n.open()
        cell = n.cell()
        home_left, home_right = _parse_homes(home_joints)
        mg_left = (await cell.controller("ur5e-left"))[0]
        mg_right = (await cell.controller("ur5e-right"))[0]
        await _move_to_home(mg_left, mg_right, home_left, home_right)
        with contextlib.suppress(Exception):
            await n.close()
        return ExecutorStatus(message="Both arms at home.")
    except Exception as e:
        _last_error = f"{type(e).__name__}: {e}"
        logger.exception("Home failed")
        return ExecutorStatus(message=_last_error)


@app.post("/start", response_model=ExecutorStatus)
async def start(req: StartRequest = StartRequest()):
    """Move to home, connect cameras, run GR00T policy. Returns immediately."""
    global _executor, _nova_instance, _run_task, _last_error
    _last_error = ""

    if _executor is not None and _executor.phase != "IDLE":
        return _executor.status

    try:
        _nova_instance = nova.Nova()
        await _nova_instance.open()
    except Exception as e:
        _last_error = f"Nova connect failed: {type(e).__name__}: {e}"
        logger.exception("Failed to connect to Nova")
        return ExecutorStatus(message=_last_error)

    cell = _nova_instance.cell()
    home_left, home_right = _parse_homes(req.home_joints)

    ctrl_left = await cell.controller("ur5e-left")
    ctrl_right = await cell.controller("ur5e-right")
    mg_left = ctrl_left[0]
    mg_right = ctrl_right[0]

    schema = _build_schema(
        mg_left,
        mg_right,
        camera_server=req.camera_server,
        camera_devices=req.camera_devices,
        camera_size=req.camera_size,
        camera_fps=req.camera_fps,
        language=req.language,
    )

    client = Gr00tPolicyClient(
        host=req.groot_host,
        port=req.groot_port,
        timeout_ms=60000,
        dt_ms=66.7,  # match training data rate (15 Hz)
    )

    pid = PidConfig(
        p_gain=req.p_gain,
        i_gain=req.i_gain,
        d_gain=req.d_gain,
        ff_gain=req.ff_gain,
        velocity_limit=req.velocity_limit,
        lookahead_ms=req.lookahead_ms,
    )
    _executor = PolicyExecutor(schema, client, timeout_s=req.timeout_s, motion=pid)

    async def run() -> ExecutionResult:
        global _last_error
        try:
            await _move_to_home(mg_left, mg_right, home_left, home_right)
            return await _executor.run()
        except Exception as e:
            _last_error = f"{type(e).__name__}: {e}"
            logger.exception("Run task failed")
            raise

    _run_task = asyncio.create_task(run())
    return _executor.status


@app.post("/stop", response_model=ExecutorStatus)
async def stop():
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
async def app_icon():
    return FileResponse("static/app_icon.png", media_type="image/png")


def main():
    uvicorn.run(app, host="0.0.0.0", port=3000, log_level="info")  # noqa: S104


if __name__ == "__main__":
    main()
