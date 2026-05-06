"""Mock WebRTC camera server — loops video from HuggingFace over WebRTC."""

from __future__ import annotations

import asyncio
import os
import time
import uuid
from pathlib import Path
from typing import Any

from aiortc import RTCPeerConnection, RTCSessionDescription
from aiortc.contrib.media import MediaPlayer
from fastapi import FastAPI, HTTPException
from fastapi.responses import FileResponse
from fastapi.staticfiles import StaticFiles

VIDEOS_DIR = Path(os.environ.get("VIDEOS_DIR", str(Path(__file__).parent.parent / "videos")))
BASE_PATH = os.environ.get("BASE_PATH", "")
DEVICE_MAP = {"315122271048": "flange", "314522065367": "left", "319522063360": "right"}
STATIC_DIR = Path(__file__).parent.parent / "static"

app = FastAPI(root_path=BASE_PATH)
_sessions: dict[str, dict[str, Any]] = {}


@app.get("/api/devices/")
async def list_devices() -> list[dict[str, Any]]:
    return [{"serial_number": d, "name": n, "connected": True} for d, n in DEVICE_MAP.items()]


@app.get("/api/devices/{device_id}/sensors/")
async def get_sensors(device_id: str) -> list[dict[str, Any]]:
    if device_id not in DEVICE_MAP:
        raise HTTPException(404)
    return [{"sensor_id": 0, "name": "RGB", "supported_stream_profiles": [
        {"stream_type": "color", "format": "rgb8", "resolution": {"width": 640, "height": 480}, "framerate": 15}
    ]}]


@app.post("/api/devices/{device_id}/stream/start")
async def start_stream(device_id: str, req: Any = None) -> dict[str, str]:
    return {"status": "streaming"}


@app.post("/api/devices/{device_id}/stream/stop")
async def stop_stream(device_id: str) -> dict[str, str]:
    return {"status": "stopped"}


@app.post("/api/webrtc/offer")
async def webrtc_offer(req: dict[str, Any]) -> dict[str, Any]:
    device_id = req["device_id"]
    cam_name = DEVICE_MAP.get(device_id)
    if not cam_name:
        raise HTTPException(404)

    video_file = VIDEOS_DIR / f"{cam_name}.mp4"
    if not video_file.exists():
        raise HTTPException(503, f"Video not found: {video_file}")

    player = MediaPlayer(str(video_file), options={"stream_loop": "-1"})
    pc = RTCPeerConnection()
    pc.addTrack(player.video)

    offer = await pc.createOffer()
    await pc.setLocalDescription(offer)
    while pc.iceGatheringState != "complete":
        await asyncio.sleep(0.05)

    session_id = str(uuid.uuid4())
    _sessions[session_id] = {"pc": pc, "player": player, "created_at": time.time()}
    return {"session_id": session_id, "sdp": pc.localDescription.sdp, "type": pc.localDescription.type}


@app.post("/api/webrtc/answer")
async def webrtc_answer(req: dict[str, Any]) -> dict[str, str]:
    session = _sessions.get(req["session_id"])
    if not session:
        raise HTTPException(404)
    await session["pc"].setRemoteDescription(RTCSessionDescription(sdp=req["sdp"], type=req["type"]))
    return {"status": "connected"}


@app.get("/status")
async def status() -> dict[str, Any]:
    return {"active_sessions": len(_sessions), "cameras": list(DEVICE_MAP.values()), "host_ip": _get_local_ip()}


def _get_local_ip() -> str:
    """Get the machine's LAN IP."""
    import socket
    try:
        s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        s.connect(("8.8.8.8", 80))
        ip = s.getsockname()[0]
        s.close()
        return ip
    except OSError:
        return "localhost"


@app.get("/", include_in_schema=False)
async def ui() -> FileResponse:
    return FileResponse(STATIC_DIR / "index.html", media_type="text/html")


app.mount("/static", StaticFiles(directory=str(STATIC_DIR)), name="static")


if __name__ == "__main__":
    import uvicorn
    uvicorn.run(app, host="0.0.0.0", port=9100)  # noqa: S104
