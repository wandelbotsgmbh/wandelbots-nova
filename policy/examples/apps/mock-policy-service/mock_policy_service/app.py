"""Stateless mock policy service.

A pure inference endpoint: observation in, action out.

- Always ready to serve predictions
- Never starts or stops robot motion
- The PolicyExecutor drives inference by sending observations via NATS
- Equal observations produce equal actions (deterministic)

Provides:
- GET /health
- GET /status
- POST /configure
- NATS request/reply on configurable subject
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
NATS_SUBJECT = config(
    "NATS_SUBJECT",
    default="nova.v2.cells.cell.apps.mock-policy-service.predict",
    cast=str,
)


# ---------------------------------------------------------------------------
# Config
# ---------------------------------------------------------------------------


class PolicyConfig(BaseModel):
    amplitude: float = Field(default=0.08, description="Joint offset amplitude in radians")
    joint_phase: float = Field(default=0.4, description="Per-joint phase offset in radians")
    step_phase: float = Field(default=0.2, description="Per-step phase offset for chunked output")
    chunk_size: int = Field(default=16, ge=1, description="Number of steps per action chunk")
    dt_ms: float = Field(default=33.0, gt=0, description="Step spacing in milliseconds")


class PolicyState:
    def __init__(self) -> None:
        self.config: PolicyConfig = PolicyConfig()
        self.nats_requests: int = 0

    def configure(self, cfg: PolicyConfig) -> None:
        self.config = cfg
        logger.info("Policy configured: %s", cfg.model_dump())


policy_state = PolicyState()


# ---------------------------------------------------------------------------
# NATS subscriber
# ---------------------------------------------------------------------------

_nats_sub: object | None = None
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
    """Handle a NATS request/reply: obs in → action out."""
    policy_state.nats_requests += 1
    try:
        obs = json.loads(msg.data.decode())
        reply = predict(obs)
        if msg.reply and _nats_client is not None:
            await _nats_client.publish(msg.reply, json.dumps(reply).encode())  # type: ignore[union-attr]
    except Exception:
        logger.exception("NATS handler error")


# ---------------------------------------------------------------------------
# Inference logic (pure function)
# ---------------------------------------------------------------------------


def _phase_from_motion_group_id(motion_group_id: str) -> float:
    return float((sum(ord(ch) for ch in motion_group_id) % 360) * math.pi / 180.0)


def _generate_target(
    current: list[float],
    motion_group_id: str,
    cfg: PolicyConfig,
    *,
    step_index: int = 0,
) -> list[float]:
    """Deterministic target: same input + same config => same output."""
    phase = _phase_from_motion_group_id(motion_group_id)
    return [
        current[i] + cfg.amplitude * math.sin(phase + i * cfg.joint_phase + step_index * cfg.step_phase)
        for i in range(len(current))
    ]


def predict(obs: dict[str, Any]) -> dict[str, Any]:
    """Produce an action response from an observation.

    Supports:
    - Flat features: {"arm_0_joint_1.pos": 0.1, ..., "arm_0_gripper.pos": 50.0}
    - Single-MG: {"joints": [...], "motion_group_id": "0@ur10e"}
    - Multi-MG: {"0@ur10e": {"joints": [...], ...}, ...}
    """
    cfg = policy_state.config

    # Flat feature observation (FeatureMap mode)
    # Keys look like "arm_0_joint_1.pos", "left_joint_2.pos", etc.
    if any(k.endswith(".pos") for k in obs):
        features: dict[str, float] = {}
        for key, value in obs.items():
            if not key.endswith(".pos"):
                continue
            current = float(value)
            if "gripper" in key:
                # Toggle gripper based on the first joint of the same arm
                # Find the arm prefix (e.g. "arm_0") and check joint_1
                prefix = key.rsplit("_gripper", maxsplit=1)[0]
                j1_key = f"{prefix}_joint_1.pos"
                j1_val = float(obs.get(j1_key, 0.0))
                features[key] = 100.0 if j1_val > 0 else 0.0
            else:
                phase = float(sum(ord(ch) for ch in key) % 360) * math.pi / 180.0
                features[key] = current + cfg.amplitude * math.sin(current * 5.0 + phase * 0.3)
        return {"features": features}

    # Single motion group observation
    if "joints" in obs and isinstance(obs["joints"], list):
        mg_id = str(obs.get("motion_group_id", "0@ur10e"))
        steps = [
            _generate_target(obs["joints"], mg_id, cfg, step_index=i)
            for i in range(cfg.chunk_size)
        ]
        return {"joints": {mg_id: steps}, "dt_ms": cfg.dt_ms}

    # Multi motion group observation
    all_joints: dict[str, list[list[float]]] = {}
    for key, value in obs.items():
        if isinstance(value, dict) and "joints" in value:
            mg_id = str(value.get("motion_group_id", key))
            steps = [
                _generate_target(value["joints"], mg_id, cfg, step_index=i)
                for i in range(cfg.chunk_size)
            ]
            all_joints[mg_id] = steps

    if all_joints:
        return {"joints": all_joints, "dt_ms": cfg.dt_ms}

    # Unknown format — return zeros so the robot holds position
    return {"joints": {}, "dt_ms": cfg.dt_ms}


# ---------------------------------------------------------------------------
# FastAPI
# ---------------------------------------------------------------------------


@asynccontextmanager
async def lifespan(app: FastAPI) -> AsyncIterator[None]:
    await _start_nats()
    yield
    await _stop_nats()


app = FastAPI(
    title="Mock Policy Service",
    version="1.0.0",
    description="Stateless NATS inference endpoint for policy execution examples.",
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
        nats_connected=_nats_client is not None,
        nats_subject=NATS_SUBJECT,
        nats_requests=policy_state.nats_requests,
        config=policy_state.config,
    )


@app.post("/configure", response_model=StatusResponse)
async def configure_policy(cfg: PolicyConfig) -> StatusResponse:
    """Update inference parameters (does not affect robot motion)."""
    policy_state.configure(cfg)
    return StatusResponse(
        ready=True,
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
