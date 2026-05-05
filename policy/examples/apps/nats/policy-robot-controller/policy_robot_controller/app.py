"""Policy Robot Controller — runs one policy episode via PID jogging.

Moves robots to home, then runs the policy via NATS until timeout or /stop.

Provides:
- GET /status
- POST /start
- POST /stop
"""

from __future__ import annotations

import asyncio
import contextlib
import logging
from typing import TYPE_CHECKING

import nats
import uvicorn
from decouple import config
from fastapi import FastAPI
from fastapi.middleware.cors import CORSMiddleware
from fastapi.responses import FileResponse
from policy import FeatureGroup, FeatureMap, NatsPolicyClient, PolicyExecutor
from policy.executor import ExecutorStatus
from pydantic import BaseModel, Field

from nova import Nova
from nova.actions import joint_ptp

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

app = FastAPI(
    title="Policy Robot Controller",
    version="1.0.0",
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
_nats_client: nats.aio.client.Client | None = None
_run_task: asyncio.Task[ExecutionResult] | None = None


class StartRequest(BaseModel):
    nats_subject: str = Field(default=NATS_SUBJECT)
    motion_groups: str = Field(default="0@ur10e,0@ur10e-2")
    home_joints: str = Field(default="0,-1.571,1.571,-1.571,-1.571,0;0,-1.571,-1.571,-1.571,1.571,0")
    timeout_s: float = Field(default=10.0, ge=0)


@app.get("/status", response_model=ExecutorStatus)
async def get_status():
    if _executor is None:
        return ExecutorStatus()
    return _executor.status


@app.post("/start", response_model=ExecutorStatus)
async def start(req: StartRequest = StartRequest()):
    """Move to home, then run policy. Returns immediately."""
    global _executor, _nova, _nats_client, _run_task

    if _executor is not None and _executor.phase != "IDLE":
        return _executor.status

    _nova = Nova()
    await _nova.open()
    cell = _nova.cell()

    if NATS_BROKER:
        _nats_client = await nats.connect(servers=NATS_BROKER, max_reconnect_attempts=10)
    else:
        nats_config = _nova.config.nats_client_config or {}
        nats_url = nats_config.get("servers", "nats://localhost:4222")
        _nats_client = await nats.connect(servers=nats_url, max_reconnect_attempts=10)

    mg_ids = [mg.strip() for mg in req.motion_groups.split(",")]
    homes_raw = req.home_joints.split(";") if ";" in req.home_joints else [req.home_joints] * len(mg_ids)
    homes = [tuple(float(v) for v in h.strip().split(",")) for h in homes_raw]

    mgs = []
    for mg_id in mg_ids:
        ctrl_name = mg_id.rsplit("@", maxsplit=1)[-1]
        ctrl = await cell.controller(ctrl_name)
        mgs.append(ctrl.motion_group(mg_id))

    feature_map = FeatureMap(
        groups=[
            FeatureGroup(motion_group=mg, name=f"arm_{i}", ios={"gripper": "digital_out[0]"})
            for i, mg in enumerate(mgs)
        ]
    )

    _executor = PolicyExecutor(
        feature_map=feature_map,
        policy=NatsPolicyClient(_nats_client, subject=req.nats_subject, timeout=5.0),
        timeout_s=req.timeout_s,
    )

    async def run() -> ExecutionResult:
        # Move to home
        home_tasks = []
        for mg, home in zip(mgs, homes, strict=False):
            tcp = (await mg.tcp_names())[0]
            traj = await mg.plan([joint_ptp(home)], tcp)
            home_tasks.append(mg.execute(traj, tcp, actions=[joint_ptp(home)]))
        await asyncio.gather(*home_tasks)
        # Run policy
        return await _executor.run()

    _run_task = asyncio.create_task(run())
    return _executor.status


@app.post("/stop", response_model=ExecutorStatus)
async def stop():
    global _executor, _nova, _nats_client, _run_task

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

    if _nova is not None:
        with contextlib.suppress(Exception):
            await _nova.close()
        _nova = None

    _executor = None
    return ExecutorStatus()


@app.get("/app_icon.png", include_in_schema=False)
async def app_icon():
    return FileResponse("static/app_icon.png", media_type="image/png")


def main():
    uvicorn.run(app, host="0.0.0.0", port=3000, log_level="info", proxy_headers=True, forwarded_allow_ips="*")  # noqa: S104


if __name__ == "__main__":
    main()
