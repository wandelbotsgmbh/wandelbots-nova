"""Stateless mock policy service — observation in, action out via NATS.

Subscribes to:
- ``NATS_SUBJECT`` for scalar observations (request/reply)
- ``NATS_SUBJECT.images.*`` for camera images (publish, latest-frame semantics)

When a prediction request arrives, merges the latest images with the
scalar observation and runs inference.
"""

from __future__ import annotations

import logging
import math
from contextlib import asynccontextmanager
from typing import Any

import numpy as np
import uvicorn
from decouple import config
from fastapi import FastAPI
from fastapi.responses import FileResponse

from mock_policy_service.nats_wire import pack, unpack, unpack_image

logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)

BASE_PATH = config("BASE_PATH", default="", cast=str)
NATS_BROKER = config("NATS_BROKER", default=None)
NATS_SUBJECT = config(
    "NATS_SUBJECT",
    default="nova.v2.cells.cell.apps.mock-policy-service.predict",
    cast=str,
)

AMPLITUDE = 0.3

_nats_client: object | None = None
_nats_subs: list[object] = []
_nats_requests: int = 0
_latest_images: dict[str, np.ndarray] = {}
_last_image_shapes: dict[str, tuple[int, ...]] = {}
_start_time: float | None = None


# ---------------------------------------------------------------------------
# Inference (pure function)
# ---------------------------------------------------------------------------


def predict(obs: dict[str, Any]) -> dict[str, Any]:
    """obs → action. Time-based oscillation around current position.

    Uses server-side clock for smooth sinusoidal motion that visibly
    oscillates without drifting.
    """
    import time

    global _start_time
    if _start_time is None:
        _start_time = time.monotonic()
    t = time.monotonic() - _start_time
    freq = 2.0  # Hz

    features: dict[str, float] = {}
    for key, value in obs.items():
        if isinstance(value, np.ndarray):
            _last_image_shapes[key] = value.shape
            continue
        if not isinstance(value, (int, float)):
            continue
        current = float(value)
        if "gripper" in key:
            # Slow toggle at 0.25 Hz
            features[key] = 1.0 if math.sin(0.5 * math.pi * t) > 0 else 0.0
        else:
            phase = float(sum(ord(ch) for ch in key) % 360) * math.pi / 180.0
            features[key] = current + AMPLITUDE * math.sin(2.0 * math.pi * freq * t + phase)
    return {"features": features}


# ---------------------------------------------------------------------------
# NATS
# ---------------------------------------------------------------------------


async def _start_nats() -> None:
    global _nats_client
    if not NATS_BROKER:
        logger.info("NATS_BROKER not set — NATS disabled")
        return
    import nats

    _nats_client = await nats.connect(servers=NATS_BROKER, max_reconnect_attempts=10)

    # Subscribe to scalar observations (request/reply)
    sub = await _nats_client.subscribe(NATS_SUBJECT, cb=_nats_handler)
    _nats_subs.append(sub)

    # Subscribe to image publications
    img_sub = await _nats_client.subscribe(f"{NATS_SUBJECT}.images.*", cb=_image_handler)
    _nats_subs.append(img_sub)

    logger.info("NATS subscribed on %r (+ images.*)", NATS_SUBJECT)


async def _stop_nats() -> None:
    global _nats_client
    for sub in _nats_subs:
        await sub.unsubscribe()  # type: ignore[union-attr]
    _nats_subs.clear()
    if _nats_client is not None:
        await _nats_client.drain()  # type: ignore[union-attr]
        _nats_client = None


async def _image_handler(msg: Any) -> None:
    """Store latest image per camera name."""
    # Subject: nova.v2...predict.images.<camera_name>
    camera_name = msg.subject.rsplit(".", maxsplit=1)[-1]
    try:
        _latest_images[camera_name] = unpack_image(msg.data)
    except Exception:
        logger.exception("Image decode error for %s", camera_name)


async def _nats_handler(msg: Any) -> None:
    """Handle scalar observation request, merge with latest images, reply."""
    global _nats_requests
    _nats_requests += 1
    try:
        obs = unpack(msg.data)

        # Merge latest images into observation
        image_names = obs.pop("__images__", [])
        for name in image_names:
            if name in _latest_images:
                obs[name] = _latest_images[name]

        reply = predict(obs)
        if msg.reply and _nats_client is not None:
            await _nats_client.publish(msg.reply, pack(reply))  # type: ignore[union-attr]
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
        "last_image_shapes": {k: list(v) for k, v in _last_image_shapes.items()},
        "active_cameras": list(_latest_images.keys()),
    }


@app.get("/app_icon.png", include_in_schema=False)
async def app_icon():
    return FileResponse("static/app_icon.png", media_type="image/png")


def main() -> None:
    uvicorn.run(app, host="0.0.0.0", port=3000, log_level="info")  # noqa: S104


if __name__ == "__main__":
    main()
