"""Policy Robot Controller — runs one policy episode via PID jogging.

Moves robots to home, then runs the policy via NATS until timeout or /stop.

Provides:
- GET /status
- POST /start — move to home, then begin policy execution
- POST /stop  — signal the executor to stop
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
from policy import (
    FeatureGroup,
    FeatureMap,
    MotionError,
    NatsPolicyClient,
    PolicyExecutor,
)
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

HOME_LEFT = (0.0, -1.571, 1.571, -1.571, -1.571, 0.0)
HOME_RIGHT = (0.0, -1.571, -1.571, -1.571, 1.571, 0.0)

app = FastAPI(
    title="Policy Robot Controller",
    version="1.0.0",
    description="Moves robots to home, then runs policy via PID jogging + NATS.",
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

# ---------------------------------------------------------------------------
# State
# ---------------------------------------------------------------------------

_executor: PolicyExecutor | None = None
_nova: Nova | None = None
_nats_client: nats.aio.client.Client | None = None
_run_task: asyncio.Task[ExecutionResult] | None = None


# ---------------------------------------------------------------------------
# Config
# ---------------------------------------------------------------------------


class StartRequest(BaseModel):
    nats_subject: str = Field(
        default=NATS_SUBJECT,
        description="NATS subject for policy inference",
    )
    motion_groups: str = Field(
        default="0@ur10e,0@ur10e-2",
        description="Comma-separated motion group IDs",
    )
    home_joints: str = Field(
        default="0,-1.571,1.571,-1.571,-1.571,0;0,-1.571,-1.571,-1.571,1.571,0",
        description="Semicolon-separated home joint positions per motion group",
    )
    timeout_s: float = Field(
        default=10.0,
        ge=0,
        description="Duration in seconds (0 = run until /stop)",
    )


# ---------------------------------------------------------------------------
# Endpoints
# ---------------------------------------------------------------------------


@app.get("/status", response_model=ExecutorStatus)
async def get_status():
    """Get the current executor phase."""
    if _executor is None:
        return ExecutorStatus()
    return _executor.status


@app.post("/start", response_model=ExecutorStatus)
async def start(req: StartRequest = StartRequest()):
    """Move robots to home, then start policy execution over NATS.

    Returns immediately — execution runs in background until timeout or /stop.
    """
    global _executor, _nova, _nats_client, _run_task

    if _executor is not None and _executor.phase != "IDLE":
        return _executor.status

    # Connect to Nova
    _nova = Nova()
    await _nova.open()
    cell = _nova.cell()

    # Connect to NATS
    if NATS_BROKER:
        _nats_client = await nats.connect(servers=NATS_BROKER, max_reconnect_attempts=10)
    else:
        nats_config = _nova.config.nats_client_config or {}
        nats_url = nats_config.get("servers", "nats://localhost:4222")
        _nats_client = await nats.connect(servers=nats_url, max_reconnect_attempts=10)

    # Resolve motion groups and home positions
    mg_ids = [mg.strip() for mg in req.motion_groups.split(",")]
    homes_raw = req.home_joints.split(";") if ";" in req.home_joints else [req.home_joints] * len(mg_ids)
    homes = [tuple(float(v) for v in h.strip().split(",")) for h in homes_raw]

    mgs = []
    for mg_id in mg_ids:
        ctrl_name = mg_id.rsplit("@", maxsplit=1)[-1]
        ctrl = await cell.controller(ctrl_name)
        mg = ctrl.motion_group(mg_id)
        mgs.append(mg)

    # Move to home
    logger.info("Moving %d robots to home...", len(mgs))
    home_tasks = []
    for mg, home in zip(mgs, homes, strict=False):
        tcp = (await mg.tcp_names())[0]
        traj = await mg.plan([joint_ptp(home)], tcp)
        home_tasks.append(mg.execute(traj, tcp, actions=[joint_ptp(home)]))
    await asyncio.gather(*home_tasks)
    logger.info("All robots at home.")

    # Build FeatureMap for flat-feature policy communication
    feature_map = FeatureMap(
        groups=[
            FeatureGroup(motion_group=mg, name=f"arm_{i}", ios={"gripper": "digital_out[0]"})
            for i, mg in enumerate(mgs)
        ]
    )

    # Create executor
    _executor = PolicyExecutor(
        feature_map=feature_map,
        policy=NatsPolicyClient(_nats_client, subject=req.nats_subject, timeout=5.0),
        timeout_s=req.timeout_s,
    )

    # Run in background
    _run_task = asyncio.create_task(_run_and_log())
    await asyncio.sleep(0.1)
    return _executor.status


async def _run_and_log() -> ExecutionResult:
    """Run executor and log result."""
    try:
        result = await _executor.run()
        logger.info("Execution finished: reason=%s steps=%d", result.reason, result.steps)
        return result
    except MotionError as e:
        logger.error("Motion error (joint limit / collision): %s", e)
        raise
    except Exception as e:
        logger.error("Execution error: %s", e)
        raise


@app.post("/stop", response_model=ExecutorStatus)
async def stop():
    """Signal the executor to stop, wait for it, and clean up."""
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
