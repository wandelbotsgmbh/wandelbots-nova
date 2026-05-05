"""GR00T robot controller app.

Runs ``PolicyExecutor`` against a GR00T-style ZMQ policy service and maps a
mock dual-arm GR00T action chunk onto two NOVA motion groups.
"""

from __future__ import annotations

import asyncio
import logging

import numpy as np
import uvicorn
from decouple import config
from fastapi import FastAPI
from fastapi.middleware.cors import CORSMiddleware
from fastapi.responses import FileResponse
from policy import Gr00tPolicyClient, PolicyExecutor
from policy.executor import ExecutorStatus
from policy.types import ActionChunk
from pydantic import BaseModel, Field

from nova import Nova
from nova.actions import joint_ptp
from nova.cell.motion_group import MotionGroup
from nova.types import RobotState

logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)

BASE_PATH = config("BASE_PATH", default="", cast=str)

LEFT_ROLE = "left"
RIGHT_ROLE = "right"

app = FastAPI(
    title="GR00T Robot Controller",
    version="0.1.0",
    description="Manages dual-arm robot execution against a GR00T-style ZMQ policy service.",
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


class StartRequest(BaseModel):
    policy_host: str = Field(
        default="app-mock-groot-policy-service",
        description="Hostname of the GR00T ZMQ policy service",
    )
    policy_port: int = Field(default=5555, description="Port of the GR00T ZMQ policy service")
    motion_groups: str = Field(
        default="0@ur10e,0@ur10e-2",
        description="Comma-separated motion group IDs in left,right order",
    )
    home_joints: str = Field(
        default="0,-1.571,1.571,-1.571,-1.571,0;0,-1.571,-1.571,-1.571,1.571,0",
        description="Semicolon-separated home joints per motion group",
    )
    gripper_ios: str = Field(
        default="digital_out[0],digital_out[0]",
        description="Comma-separated gripper IOs in left,right order",
    )
    language: str = Field(
        default="Coordinate both arms and move smoothly.",
        description="Language instruction sent to the mock GR00T policy",
    )
    action_dt_ms: float = Field(
        default=33.0,
        gt=0,
        description="Time spacing for each GR00T chunk step",
    )
    timeout_s: float = Field(
        default=0,
        ge=0,
        description="Timeout per episode (0 = no timeout)",
    )


class DualArmGr00tAdapter:
    def __init__(
        self,
        *,
        motion_groups: list[MotionGroup],
        gripper_ios: dict[str, str],
        language: str,
        dt_ms: float,
    ) -> None:
        if len(motion_groups) != 2:
            msg = "DualArmGr00tAdapter requires exactly two motion groups"
            raise ValueError(msg)
        self._motion_groups = motion_groups
        self._gripper_ios = gripper_ios
        self._language = language
        self._dt_ms = dt_ms
        self._role_to_mg = {
            LEFT_ROLE: motion_groups[0],
            RIGHT_ROLE: motion_groups[1],
        }

    async def build_observation(
        self,
        robot_states: dict[str, RobotState],
        motion_groups: list[MotionGroup],
    ) -> dict[str, object]:
        del motion_groups
        left_mg = self._role_to_mg[LEFT_ROLE]
        right_mg = self._role_to_mg[RIGHT_ROLE]
        left_state = robot_states[left_mg.id]
        right_state = robot_states[right_mg.id]

        left_gripper = await self._read_gripper_state(left_mg.id)
        right_gripper = await self._read_gripper_state(right_mg.id)

        return {
            "state": {
                "left_arm": self._batched(left_state.joints),
                "right_arm": self._batched(right_state.joints),
                "left_gripper": self._batched([left_gripper]),
                "right_gripper": self._batched([right_gripper]),
            },
            "language": {
                "task": [[self._language]],
            },
        }

    def decode_action(self, action: dict[str, object], info: dict[str, object]) -> ActionChunk:
        left_mg = self._role_to_mg[LEFT_ROLE]
        right_mg = self._role_to_mg[RIGHT_ROLE]

        left_arm = self._as_chunk(action, "left_arm")
        right_arm = self._as_chunk(action, "right_arm")
        left_gripper = self._as_chunk(action, "left_gripper")
        right_gripper = self._as_chunk(action, "right_gripper")

        joints = {
            left_mg.id: left_arm[0].tolist(),
            right_mg.id: right_arm[0].tolist(),
        }
        ios = {
            left_mg.id: {
                self._gripper_ios[left_mg.id]: bool(left_gripper[0, 0, 0] >= 50.0),
            },
            right_mg.id: {
                self._gripper_ios[right_mg.id]: bool(right_gripper[0, 0, 0] >= 50.0),
            },
        }
        dt_ms = float(info.get("dt_ms", self._dt_ms))
        return ActionChunk(joints=joints, ios=ios, dt_ms=dt_ms)

    async def _read_gripper_state(self, motion_group_id: str) -> float:
        mg = next(mg for mg in self._motion_groups if mg.id == motion_group_id)
        io_key = self._gripper_ios[motion_group_id]
        try:
            io_values = await mg._api_client.controller_ios_api.list_io_values(
                cell=mg._cell,
                controller=mg._controller_id,
                ios=[io_key],
            )
            closed = bool(io_values[0].root.value) if io_values else False
        except Exception:
            closed = False
        return 100.0 if closed else 0.0

    @staticmethod
    def _batched(values: list[float] | tuple[float, ...]) -> np.ndarray:
        return np.asarray(values, dtype=np.float32)[np.newaxis, np.newaxis, :]

    @staticmethod
    def _as_chunk(action: dict[str, object], key: str) -> np.ndarray:
        value = action.get(key)
        if not isinstance(value, np.ndarray):
            msg = f"GR00T action key '{key}' must be a numpy array"
            raise TypeError(msg)
        if value.ndim != 3:
            msg = f"GR00T action key '{key}' must have shape (B, T, D), got {value.shape}"
            raise ValueError(msg)
        return value.astype(np.float32)


@app.get("/status", response_model=ExecutorStatus)
async def get_status() -> ExecutorStatus:
    if _executor is None:
        return ExecutorStatus()
    return _executor.status


@app.post("/start", response_model=ExecutorStatus)
async def start(req: StartRequest = StartRequest()) -> ExecutorStatus:
    global _executor, _nova

    if _executor is not None and _executor.phase != "IDLE":
        return _executor.status

    _nova = Nova()
    await _nova.open()
    cell = _nova.cell()

    mg_ids = [mg.strip() for mg in req.motion_groups.split(",") if mg.strip()]
    if len(mg_ids) != 2:
        msg = "Exactly two motion groups are required for the GR00T dual-arm example"
        raise ValueError(msg)

    homes_raw = req.home_joints.split(";")
    if len(homes_raw) != 2:
        msg = "Exactly two home joint lists are required"
        raise ValueError(msg)
    homes = [tuple(float(v) for v in home.strip().split(",")) for home in homes_raw]

    gripper_io_list = [item.strip() for item in req.gripper_ios.split(",") if item.strip()]
    if len(gripper_io_list) != 2:
        msg = "Exactly two gripper IO keys are required"
        raise ValueError(msg)

    motion_groups: list[MotionGroup] = []
    home_by_mg_id: dict[str, tuple[float, ...]] = {}
    gripper_io_by_mg_id: dict[str, str] = {}
    for mg_id, home, gripper_io in zip(mg_ids, homes, gripper_io_list, strict=True):
        controller_name = mg_id.split("@")[1] if "@" in mg_id else mg_id
        controller = await cell.controller(controller_name)
        motion_group = controller.motion_group(mg_id)
        motion_groups.append(motion_group)
        home_by_mg_id[mg_id] = home
        gripper_io_by_mg_id[mg_id] = gripper_io

    adapter = DualArmGr00tAdapter(
        motion_groups=motion_groups,
        gripper_ios=gripper_io_by_mg_id,
        language=req.language,
        dt_ms=req.action_dt_ms,
    )

    async def on_reset(groups: list[MotionGroup]) -> None:
        tasks = []
        for motion_group in groups:
            home = home_by_mg_id[motion_group.id]
            tcp = (await motion_group.tcp_names())[0]
            current = await motion_group.joints()
            trajectory = await motion_group.plan(
                [joint_ptp(home)],
                tcp,
                start_joint_position=current,
            )
            tasks.append(motion_group.execute(trajectory, tcp, actions=[joint_ptp(home)]))
        await asyncio.gather(*tasks)

    policy = Gr00tPolicyClient(
        host=req.policy_host,
        port=req.policy_port,
        decode_action=adapter.decode_action,
    )

    _executor = PolicyExecutor(
        motion_groups=motion_groups,
        build_obs=adapter.build_observation,
        policy=policy,
        on_reset=on_reset,
        timeout_s=req.timeout_s,
    )
    await _executor.start()
    return _executor.status


@app.post("/stop", response_model=ExecutorStatus)
async def stop() -> ExecutorStatus:
    global _executor, _nova

    if _executor is not None:
        await _executor.stop()
        _executor = None

    if _nova is not None:
        await _nova.close()
        _nova = None

    return ExecutorStatus()


@app.get("/app_icon.png", include_in_schema=False)
async def app_icon() -> FileResponse:
    return FileResponse("static/app_icon.png", media_type="image/png")


def main() -> None:
    uvicorn.run(
        app,
        host="0.0.0.0",  # noqa: S104
        port=3000,
        log_level="info",
        proxy_headers=True,
        forwarded_allow_ips="*",
    )


if __name__ == "__main__":
    main()
