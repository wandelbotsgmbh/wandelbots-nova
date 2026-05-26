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
from pathlib import Path
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
    PolicyExecutor,
    PolicySchema,
    WaypointConfig,
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
    velocity_limit: float = Field(default=2.0, description="Joint velocity limit in rad/s"),
    n_action_steps: int = Field(default=8, description="Number of action chunk steps to execute (0 = all). Later steps have higher uncertainty and are discarded (receding horizon)."),
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

    executor = PolicyExecutor(schema, client, timeout_s=timeout_s, policy_rate_hz=20, motion=WaypointConfig(n_action_steps=n_action_steps))
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
# Replay episode program
# ---------------------------------------------------------------------------

REPLAY_DATA_DIR = Path(__file__).parent / "replay_data"


@nova.program(
    id="replay_episode",
    name="Replay Dataset Episode",
    description="Replays a recorded parquet episode on two UR5e arms via waypoint jogging.",
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
async def replay_episode(
    ctx: nova.ProgramContext,
    episode: int = Field(default=0, description="Episode index to replay"),
):
    """Replay a recorded dataset episode on both arms via waypoint jogging."""
    from typing import Any

    import pyarrow.parquet as pq

    from policy import ActionChunk

    # Load episode (15 fps recording, action = 12 DOF: 6 left + 6 right)
    dt_ms = 1000.0 / 15.0
    path = REPLAY_DATA_DIR / f"episode_{episode:06d}.parquet"
    table = pq.read_table(path, columns=["action", "timestamp"])
    actions = table.column("action").to_pylist()
    timestamps_s = table.column("timestamp").to_pylist()
    duration_s = timestamps_s[-1] - timestamps_s[0]
    logger.info(f"Loaded episode {episode}: {len(actions)} steps, {duration_s:.1f}s")

    # Connect
    cell = ctx.nova.cell()
    mg_left = (await cell.controller("ur5e-left"))[0]
    mg_right = (await cell.controller("ur5e-right"))[0]

    # Move to start position (first action frame)
    start_left = tuple(actions[0][:6])
    start_right = tuple(actions[0][6:])
    await _move_to_home(mg_left, mg_right, start_left, start_right)

    schema = PolicySchema(observations=[
        Observation.joint_positions("left_joints", source=mg_left),
        Observation.joint_positions("right_joints", source=mg_right),
    ])

    chunk_size = 8

    async def replay_policy(obs: dict[str, Any]) -> ActionChunk:
        session = executor._sessions.get(mg_left.id)
        elapsed_ms = session.session_elapsed_ms if session else 0
        elapsed_s = elapsed_ms / 1000.0

        # Find step from elapsed time
        step = 0
        for i, ts in enumerate(timestamps_s):
            if ts - timestamps_s[0] <= elapsed_s:
                step = i
            else:
                break
        step = min(step, len(actions) - 1)

        chunk_end = min(step + chunk_size, len(actions))
        chunk = actions[step:chunk_end] if step < len(actions) else [actions[-1]]
        start_time_ms = int((timestamps_s[step] - timestamps_s[0]) * 1000)

        return ActionChunk(
            joints={
                mg_left.id: [a[:6] for a in chunk],
                mg_right.id: [a[6:] for a in chunk],
            },
            dt_ms=dt_ms,
            start_time_ms=start_time_ms,
        )

    executor = PolicyExecutor(
        schema,
        replay_policy,
        timeout_s=duration_s + 5.0,
        policy_rate_hz=20,
        motion=WaypointConfig(state_rate_ms=10),
    )
    result = await executor.run()
    logger.info(
        "Replay finished: reason=%s steps=%d duration=%.1fs",
        result.reason, result.steps, result.duration_s,
    )


# ---------------------------------------------------------------------------
# FastAPI app with novax
# ---------------------------------------------------------------------------

novax_app = Novax()


@contextlib.asynccontextmanager
async def _app_lifespan(app_instance):
    """Custom lifespan that registers programs and verifies NATS writes."""
    from nova.program.store import ProgramStore
    from novax.config import APP_NAME

    await novax_app._nova.open()
    logger.info(f"Lifespan: NATS connected={novax_app._nova.nats.is_connected}")

    cell = novax_app._cell
    store = ProgramStore(cell=cell)

    programs = await novax_app._program_manager.get_programs()
    logger.info(f"Lifespan: {len(programs)} programs to register")

    for pid, details in programs.items():
        sp = nova.api.models.Program(
            program=details.program,
            name=details.name,
            description=details.description,
            app=APP_NAME,
            preconditions=details.preconditions,
            input_schema=details.input_schema,
        )
        key = f"{APP_NAME}.{pid}"
        await store.put(key, sp)
        logger.info(f"Lifespan: PUT {key} done")

    # Verify
    js = novax_app._nova.nats.jetstream()
    kv = await js.key_value("nova_cells_cell_programs")
    try:
        keys = await kv.keys()
        logger.info(f"Lifespan: Verified keys in NATS: {keys}")
    except Exception as e:
        logger.error(f"Lifespan: Verification failed: {e}")

    yield

    # Don't deregister or close NATS on shutdown - programs should persist


app = FastAPI(
    title="GR00T Dual-Arm Controller",
    version="2.0.0",
    root_path=BASE_PATH,
    docs_url="/",
    lifespan=_app_lifespan,
)

# Manually include programs router endpoints without the duplicate lifespan
from novax.api.programs import router as _programs_router, get_program_manager
app.dependency_overrides[get_program_manager] = lambda: novax_app.program_manager
app.include_router(_programs_router)
novax_app.register_program(move_to_home)
novax_app.register_program(gr00t_dual_arm_controller)
novax_app.register_program(replay_episode)

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
    velocity_limit: float = Field(default=2.0, description="Joint velocity limit in rad/s")
    n_action_steps: int = Field(default=8, description="Steps to execute per chunk (0 = all)")


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

    _executor = PolicyExecutor(schema, client, timeout_s=req.timeout_s, policy_rate_hz=20, motion=WaypointConfig(n_action_steps=req.n_action_steps))

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
