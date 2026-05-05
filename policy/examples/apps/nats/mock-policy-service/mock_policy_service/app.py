"""Stateless mock policy service — observation in, action out via NATS."""

from __future__ import annotations

import json
import logging
import math
from contextlib import asynccontextmanager
from typing import Any

import uvicorn
from decouple import config
from fastapi import FastAPI
from fastapi.responses import FileResponse

logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)

BASE_PATH = config("BASE_PATH", default="", cast=str)
NATS_BROKER = config("NATS_BROKER", default=None)
NATS_SUBJECT = config(
    "NATS_SUBJECT",
    default="nova.v2.cells.cell.apps.mock-policy-service.predict",
    cast=str,
)

AMPLITUDE = 0.08

_nats_client: object | None = None
_nats_sub: object | None = None
_nats_requests: int = 0


# ---------------------------------------------------------------------------
# Inference (pure function)
# ---------------------------------------------------------------------------


def predict(obs: dict[str, Any]) -> dict[str, Any]:
    """obs → action. Deterministic: same input produces same output."""
    features: dict[str, float] = {}
    for key, value in obs.items():
        if not isinstance(value, (int, float)):
            continue
        current = float(value)
        if "gripper" in key:
            prefix = key.rsplit("_gripper", maxsplit=1)[0]
            j1_val = float(obs.get(f"{prefix}_joint_1.pos", 0.0))
            features[key] = 1.0 if j1_val > 0 else 0.0
        else:
            phase = float(sum(ord(ch) for ch in key) % 360) * math.pi / 180.0
            features[key] = current + AMPLITUDE * math.sin(current * 5.0 + phase * 0.3)
    return {"features": features}


# ---------------------------------------------------------------------------
# NATS
# ---------------------------------------------------------------------------


async def _start_nats() -> None:
    global _nats_client, _nats_sub
    if not NATS_BROKER:
        logger.info("NATS_BROKER not set — NATS disabled")
        return
    import nats

    _nats_client = await nats.connect(servers=NATS_BROKER, max_reconnect_attempts=10)
    _nats_sub = await _nats_client.subscribe(NATS_SUBJECT, cb=_nats_handler)  # type: ignore[union-attr]
    logger.info("NATS subscribed on %r", NATS_SUBJECT)


async def _stop_nats() -> None:
    global _nats_client, _nats_sub
    if _nats_sub is not None:
        await _nats_sub.unsubscribe()  # type: ignore[union-attr]
        _nats_sub = None
    if _nats_client is not None:
        await _nats_client.drain()  # type: ignore[union-attr]
        _nats_client = None


async def _nats_handler(msg: Any) -> None:
    global _nats_requests
    _nats_requests += 1
    try:
        obs = json.loads(msg.data.decode())
        reply = predict(obs)
        if msg.reply and _nats_client is not None:
            await _nats_client.publish(msg.reply, json.dumps(reply).encode())  # type: ignore[union-attr]
    except Exception:
        logger.exception("NATS handler error")


# ---------------------------------------------------------------------------
# FastAPI
# ---------------------------------------------------------------------------


@asynccontextmanager
async def lifespan(app: FastAPI):
    await _start_nats()
    yield
    await _stop_nats()


app = FastAPI(title="Mock Policy Service", root_path=BASE_PATH, docs_url="/", lifespan=lifespan)


@app.get("/health")
async def health():
    return {"status": "ok"}


@app.get("/status")
async def status():
    return {
        "nats_connected": _nats_client is not None,
        "nats_subject": NATS_SUBJECT,
        "nats_requests": _nats_requests,
    }


@app.get("/app_icon.png", include_in_schema=False)
async def app_icon():
    return FileResponse("static/app_icon.png", media_type="image/png")


def main() -> None:
    uvicorn.run(app, host="0.0.0.0", port=3000, log_level="info")  # noqa: S104


if __name__ == "__main__":
    main()
