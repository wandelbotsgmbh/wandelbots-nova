"""Policy Robot Controller — uses PolicyExecutor with NATS for policy inference."""

import asyncio
import logging

import nats
import uvicorn
from decouple import config
from fastapi import FastAPI
from fastapi.middleware.cors import CORSMiddleware
from fastapi.responses import FileResponse
from nova_policy import NatsPolicyClient, PolicyExecutor
from nova_policy.executor import ExecutorStatus
from pydantic import BaseModel, Field

from nova import Nova
from nova.actions import joint_ptp
from nova.cell.motion_group import MotionGroup

logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)

BASE_PATH = config("BASE_PATH", default="", cast=str)
NATS_BROKER = config("NATS_BROKER", default=None)
NATS_SUBJECT = config("NATS_SUBJECT", default="nova.v2.cells.cell.apps.mock-policy-service.predict", cast=str)

app = FastAPI(
    title="Policy Robot Controller",
    version="0.3.0",
    description="Manages robot lifecycle for policy execution via PID jogging. Uses NATS to query the policy service.",
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
    home_joints: str = Field(
        default="0,-1.571,1.571,-1.571,-1.571,0;0,-1.571,-1.571,-1.571,1.571,0",
        description="Semicolon-separated home joints per motion group",
    )
    timeout_s: float = Field(
        default=0,
        ge=0,
        description="Timeout per episode (0 = no timeout, policy controls duration)",
    )


# ---------------------------------------------------------------------------
# Endpoints
# ---------------------------------------------------------------------------


@app.get("/status", response_model=ExecutorStatus)
async def get_status():
    """Get the current executor phase and execution state."""
    if _executor is None:
        return ExecutorStatus()
    return _executor.status


@app.post("/start", response_model=ExecutorStatus)
async def start(req: StartRequest = StartRequest()):
    """Start the executor: connect to robots, open jogging, and query the policy service via NATS.

    The executor is the active side: it sends observations and receives
    predictions via NATS request/reply. Between episodes, the on_reset
    callback moves robots to home.
    """
    global _executor, _nova, _nats_client

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
        # Derive NATS URL from Nova (ws://<host>/api/nats)
        nats_config = _nova.config.nats_client_config or {}
        nats_url = nats_config.get("servers", "nats://localhost:4222")
        _nats_client = await nats.connect(servers=nats_url, max_reconnect_attempts=10)
        logger.info("Connected to NATS at %s (derived from Nova config)", nats_url)

    # Parse motion groups
    mg_ids = [mg.strip() for mg in req.motion_groups.split(",")]
    homes_raw = (
        req.home_joints.split(";") if ";" in req.home_joints else [req.home_joints] * len(mg_ids)
    )
    homes = [tuple(float(v) for v in h.strip().split(",")) for h in homes_raw]

    # Connect controllers + motion groups
    mgs = []
    mg_home_map: dict[str, tuple[float, ...]] = {}
    for mg_id, home in zip(mg_ids, homes, strict=False):
        ctrl_name = mg_id.split("@")[1] if "@" in mg_id else mg_id
        ctrl = await cell.controller(ctrl_name)
        mg = ctrl.motion_group(mg_id)
        mgs.append(mg)
        mg_home_map[mg_id] = home

    # Reset callback: move all robots to home via trajectory execution
    async def on_reset(motion_groups: list[MotionGroup]) -> None:
        tasks = []
        for mg in motion_groups:
            home = mg_home_map[mg.id]
            tcp = (await mg.tcp_names())[0]
            current = await mg.joints()
            traj = await mg.plan([joint_ptp(home)], tcp, start_joint_position=current)
            tasks.append(mg.execute(traj, tcp, actions=[joint_ptp(home)]))
        await asyncio.gather(*tasks)

    # Create NATS policy client
    policy_client = NatsPolicyClient(
        _nats_client,
        subject=req.nats_subject,
        timeout=5.0,
    )

    # Create and start executor
    _executor = PolicyExecutor(
        motion_groups=mgs,
        policy=policy_client,
        on_reset=on_reset,
        timeout_s=req.timeout_s,
    )
    await _executor.start()
    return _executor.status


@app.post("/stop", response_model=ExecutorStatus)
async def stop():
    """Stop the executor and return to IDLE."""
    global _executor, _nova, _nats_client

    if _executor is not None:
        try:
            await _executor.stop()
        finally:
            _executor = None

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
