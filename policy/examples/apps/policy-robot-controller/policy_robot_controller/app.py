"""Policy Robot Controller — uses PolicyExecutor with NATS for policy inference."""

import asyncio
import contextlib
import logging

import nats
import uvicorn
from decouple import config
from fastapi import FastAPI
from fastapi.middleware.cors import CORSMiddleware
from fastapi.responses import FileResponse
from policy import NatsPolicyClient, PolicyExecutor
from policy.executor import ExecutorStatus
from pydantic import BaseModel, Field

from nova import Nova

logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)

BASE_PATH = config("BASE_PATH", default="", cast=str)
NATS_BROKER = config("NATS_BROKER", default=None)
NATS_SUBJECT = config("NATS_SUBJECT", default="nova.policy.predict", cast=str)

app = FastAPI(
    title="Policy Robot Controller",
    version="0.4.0",
    description="Runs one policy episode via PID jogging. Uses NATS to query the policy service.",
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
_run_task: asyncio.Task | None = None


# ---------------------------------------------------------------------------
# Config
# ---------------------------------------------------------------------------


class StartRequest(BaseModel):
    nats_subject: str = Field(
        default=NATS_SUBJECT,
        description="NATS subject to send observations and receive actions",
    )
    motion_groups: str = Field(
        default="0@ur10e,0@ur10e-2", description="Comma-separated motion group IDs"
    )
    timeout_s: float = Field(
        default=0, ge=0, description="Duration in seconds (0 = run until /stop)"
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
    """Start one policy episode: open jogging and query the policy service via NATS."""
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
        logger.info("Connected to NATS at %s", NATS_BROKER)
    else:
        nats_config = _nova.config.nats_client_config or {}
        nats_url = nats_config.get("servers", "nats://localhost:4222")
        _nats_client = await nats.connect(servers=nats_url, max_reconnect_attempts=10)
        logger.info("Connected to NATS at %s (derived from Nova config)", nats_url)

    # Parse motion groups
    mg_ids = [mg.strip() for mg in req.motion_groups.split(",")]

    mgs = []
    for mg_id in mg_ids:
        ctrl_name = mg_id.split("@")[1] if "@" in mg_id else mg_id
        ctrl = await cell.controller(ctrl_name)
        mg = ctrl.motion_group(mg_id)
        mgs.append(mg)

    # Create and start executor
    _executor = PolicyExecutor(
        motion_groups=mgs,
        policy=NatsPolicyClient(_nats_client, subject=req.nats_subject, timeout=5.0),
        timeout_s=req.timeout_s,
    )
    # Start run() in background task
    _run_task = asyncio.create_task(_executor.run())
    # Give it a moment to enter EXECUTING
    await asyncio.sleep(0.1)
    return _executor.status


@app.post("/stop", response_model=ExecutorStatus)
async def stop():
    """Stop execution and return to IDLE."""
    global _executor, _nova, _nats_client, _run_task

    if _executor is not None:
        _executor.stop()

    if _run_task is not None:
        with contextlib.suppress(asyncio.CancelledError):
            await _run_task
        _run_task = None

    if _nats_client is not None:
        try:
            await _nats_client.drain()
        finally:
            _nats_client = None

    if _nova is not None:
        try:
            await _nova.close()
        finally:
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
