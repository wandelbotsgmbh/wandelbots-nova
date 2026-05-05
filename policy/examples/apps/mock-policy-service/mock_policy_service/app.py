"""Stateless mock policy service.

This service behaves like an inference endpoint, not like a robot controller:

- it is always ready to serve predictions
- it never starts or stops robot motion itself
- the PolicyExecutor is the active side and drives inference by sending observations
- equal observations produce equal actions

Provides:
- GET /health
- GET /status
- POST /configure
- NATS request/reply on configurable subject (default ``nova.v2.cells.cell.apps.mock-policy-service.predict``)

On the Nova platform, NATS is the preferred transport for app-to-app
communication.
"""

from __future__ import annotations

import json
import logging
import math
from contextlib import asynccontextmanager
from typing import TYPE_CHECKING, Any

import uvicorn
from decouple import config
from fastapi import FastAPI
from fastapi.middleware.cors import CORSMiddleware
from fastapi.responses import FileResponse
from pydantic import BaseModel, Field

if TYPE_CHECKING:
    from collections.abc import AsyncIterator

logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)

BASE_PATH = config("BASE_PATH", default="", cast=str)
NATS_BROKER = config("NATS_BROKER", default=None)
NATS_SUBJECT = config("NATS_SUBJECT", default="nova.v2.cells.cell.apps.mock-policy-service.predict", cast=str)


# ---------------------------------------------------------------------------
# Config model
# ---------------------------------------------------------------------------

class PolicyConfig(BaseModel):
    amplitude: float = Field(default=0.08, description="Joint offset amplitude in radians")
    joint_phase: float = Field(default=0.4, description="Per-joint phase offset in radians")
    step_phase: float = Field(default=0.2, description="Per-step phase offset for chunked output")
    chunk_size: int = Field(default=16, ge=1, description="Chunk size for /predict_chunked")
    dt_ms: float = Field(default=33.0, gt=0, description="Step spacing for /predict_chunked")


class PolicyState:
    def __init__(self) -> None:
        self.config: PolicyConfig = PolicyConfig()
        self.connections: int = 0
        self.nats_requests: int = 0

    def configure(self, cfg: PolicyConfig) -> None:
        self.config = cfg
        logger.info(
            "Policy configured: amplitude=%.3f joint_phase=%.3f step_phase=%.3f chunk=%d dt_ms=%.1f",
            cfg.amplitude,
            cfg.joint_phase,
            cfg.step_phase,
            cfg.chunk_size,
            cfg.dt_ms,
        )


policy_state = PolicyState()


# ---------------------------------------------------------------------------
# NATS subscriber
# ---------------------------------------------------------------------------

_nats_sub: object | None = None  # nats subscription handle
_nats_client: object | None = None


async def _start_nats() -> None:
    """Connect to NATS and subscribe to the policy subject."""
    global _nats_client, _nats_sub

    if not NATS_BROKER:
        logger.info("NATS_BROKER not set — NATS inference disabled")
        return

    try:
        import nats
    except ImportError:
        logger.warning("nats-py not installed — NATS inference disabled")
        return

    try:
        _nats_client = await nats.connect(servers=NATS_BROKER, max_reconnect_attempts=10)
        _nats_sub = await _nats_client.subscribe(NATS_SUBJECT, cb=_nats_handler)  # type: ignore[union-attr]
        logger.info("NATS subscribed on subject %r via %s", NATS_SUBJECT, NATS_BROKER)
    except Exception:
        logger.exception("Failed to connect to NATS at %s", NATS_BROKER)


async def _stop_nats() -> None:
    global _nats_client, _nats_sub
    if _nats_sub is not None:
        await _nats_sub.unsubscribe()  # type: ignore[union-attr]
        _nats_sub = None
    if _nats_client is not None:
        await _nats_client.drain()  # type: ignore[union-attr]
        _nats_client = None
    logger.info("NATS disconnected")


async def _nats_handler(msg: Any) -> None:
    """Handle a NATS request/reply message."""
    policy_state.nats_requests += 1
    try:
        obs = json.loads(msg.data.decode())

        if obs.get("executor_stopped"):
            logger.info("Executor stopped via NATS (reason=%s)", obs.get("reason", "?"))
            return

        reply = _predict_from_obs(obs)
        if msg.reply:
            nc = _nats_client
            if nc is not None:
                await nc.publish(msg.reply, json.dumps(reply).encode())  # type: ignore[union-attr]
    except Exception:
        logger.exception("NATS handler error")
        if msg.reply and _nats_client is not None:
            await _nats_client.publish(msg.reply, json.dumps({"waiting": True}).encode())  # type: ignore[union-attr]


# ---------------------------------------------------------------------------
# Shared inference logic
# ---------------------------------------------------------------------------

def _phase_from_motion_group_id(motion_group_id: str) -> float:
    return float((sum(ord(ch) for ch in motion_group_id) % 360) * math.pi / 180.0)


def generate_target(
    current: list[float],
    motion_group_id: str,
    cfg: PolicyConfig,
    *,
    step_index: int = 0,
) -> list[float]:
    """Generate a deterministic target from the current observation.

    Stateless contract: same input observation + same config => same output.
    """
    phase = _phase_from_motion_group_id(motion_group_id)
    return [
        current[joint_index]
        + cfg.amplitude
        * math.sin(phase + joint_index * cfg.joint_phase + step_index * cfg.step_phase)
        for joint_index in range(len(current))
    ]


def _predict_from_obs(obs: dict[str, Any]) -> dict[str, Any]:
    """Produce a chunked response from the observation dict.

    Works for both structured (per-MG) and single-MG observations.
    """
    cfg = policy_state.config

    # Single motion group observation (flat dict with "joints")
    if "joints" in obs and isinstance(obs["joints"], list):
        mg_id = str(obs.get("motion_group_id", "0@ur10e"))
        steps = [
            generate_target(obs["joints"], mg_id, cfg, step_index=i)
            for i in range(cfg.chunk_size)
        ]
        return {"joints": {mg_id: steps}, "dt_ms": cfg.dt_ms}

    # Multi motion group observation (keyed by MG id)
    all_joints: dict[str, list[list[float]]] = {}
    for key, value in obs.items():
        if isinstance(value, dict) and "joints" in value:
            mg_id = str(value.get("motion_group_id", key))
            steps = [
                generate_target(value["joints"], mg_id, cfg, step_index=i)
                for i in range(cfg.chunk_size)
            ]
            all_joints[mg_id] = steps

    if all_joints:
        return {"joints": all_joints, "dt_ms": cfg.dt_ms}

    return {"waiting": True}


# ---------------------------------------------------------------------------
# FastAPI lifespan
# ---------------------------------------------------------------------------

@asynccontextmanager
async def lifespan(app: FastAPI) -> AsyncIterator[None]:
    await _start_nats()
    yield
    await _stop_nats()


# ---------------------------------------------------------------------------
# FastAPI app
# ---------------------------------------------------------------------------

app = FastAPI(
    title="Mock Policy Service",
    version="0.3.0",
    description="Stateless NATS inference service for PolicyExecutor examples.",
    root_path=BASE_PATH,
    docs_url="/",
    lifespan=lifespan,
)

app.add_middleware(
    CORSMiddleware,
    allow_origins=["*"],
    allow_credentials=True,
    allow_methods=["*"],
    allow_headers=["*"],
)


class StatusResponse(BaseModel):
    ready: bool
    connections: int
    nats_connected: bool
    nats_subject: str
    nats_requests: int
    config: PolicyConfig


@app.get("/health")
async def health() -> dict[str, str]:
    return {"status": "ok", "service": "mock-policy-service"}


@app.get("/status", response_model=StatusResponse)
async def status() -> StatusResponse:
    return StatusResponse(
        ready=True,
        connections=policy_state.connections,
        nats_connected=_nats_client is not None,
        nats_subject=NATS_SUBJECT,
        nats_requests=policy_state.nats_requests,
        config=policy_state.config,
    )


@app.post("/configure", response_model=StatusResponse)
async def configure_policy(cfg: PolicyConfig) -> StatusResponse:
    """Update inference parameters."""
    policy_state.configure(cfg)
    return StatusResponse(
        ready=True,
        connections=policy_state.connections,
        nats_connected=_nats_client is not None,
        nats_subject=NATS_SUBJECT,
        nats_requests=policy_state.nats_requests,
        config=cfg,
    )


@app.get("/app_icon.png", include_in_schema=False)
async def app_icon() -> FileResponse:
    return FileResponse("static/app_icon.png", media_type="image/png")


# ---------------------------------------------------------------------------
# Entry point
# ---------------------------------------------------------------------------

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
