"""WebRTC camera connection internals.

Manages a single WebRTC peer connection with background frame reception.
This module is an implementation detail — users interact with ``CameraSet``.
"""

from __future__ import annotations

import asyncio
import contextlib
import logging
import threading
import time
from typing import TYPE_CHECKING, Any

if TYPE_CHECKING:
    from numpy.typing import NDArray

    from policy.cameras._cameras import WebRTCCameraConfig

logger = logging.getLogger(__name__)

# Optional imports — aiortc is heavy and not always needed
_aiortc_available = False
try:
    import av.logging
    from aiortc import RTCPeerConnection, RTCSessionDescription

    av.logging.set_level(av.logging.ERROR)
    _aiortc_available = True
except ImportError:
    pass


def require_aiortc() -> None:
    """Raise if aiortc is not installed."""
    if not _aiortc_available:
        msg = "aiortc is required for WebRTC cameras. Install with: pip install aiortc"
        raise ModuleNotFoundError(msg)


class WebRTCConnection:
    """Manages a single WebRTC camera connection with background frame reception.

    Handles the full lifecycle:
    1. Start camera hardware stream via REST API
    2. WebRTC offer/answer exchange
    3. Background frame reception in a dedicated event loop thread
    4. Thread-safe frame access with staleness tracking
    """

    def __init__(self, name: str, config: WebRTCCameraConfig) -> None:
        self._name = name
        self._config = config
        self._pc: Any = None
        self._frame: NDArray[Any] | None = None
        self._frame_time: float = 0.0
        self._frame_lock = threading.Lock()
        self._frame_event = asyncio.Event()
        self._receive_task: asyncio.Task[None] | None = None

    def latest_frame(self) -> NDArray[Any] | None:
        """Get the most recent frame (thread-safe)."""
        with self._frame_lock:
            return self._frame

    def frame_age_s(self) -> float:
        """Seconds since the last frame was received. Returns inf if no frame yet."""
        with self._frame_lock:
            if self._frame is None:
                return float("inf")
            return time.monotonic() - self._frame_time

    async def connect(self, timeout_s: float = 30.0) -> None:
        """Establish WebRTC connection, start stream, wait for first frame."""
        cfg = self._config
        api_url = cfg.api_url.rstrip("/")

        # 1. Start camera stream
        await asyncio.to_thread(self._start_stream, api_url, cfg)

        # 2. Set up WebRTC peer connection in the current event loop
        await self._setup_webrtc(api_url, cfg)

        # 3. Wait for first frame
        try:
            await asyncio.wait_for(self._frame_event.wait(), timeout=timeout_s)
        except TimeoutError as e:
            await self.disconnect()
            msg = f"Camera '{self._name}' timed out waiting for first frame"
            raise RuntimeError(msg) from e

        logger.info("Camera '%s' connected (device=%s)", self._name, cfg.device_id)

    async def disconnect(self) -> None:
        """Close peer connection."""
        if self._receive_task is not None:
            self._receive_task.cancel()
            with contextlib.suppress(asyncio.CancelledError):
                await self._receive_task
            self._receive_task = None

        if self._pc is not None:
            with contextlib.suppress(TimeoutError, OSError):
                await self._pc.close()
            self._pc = None

        # Stop camera stream (best-effort)
        try:
            import requests  # noqa: PLC0415

            api_url = self._config.api_url.rstrip("/")
            requests.post(
                f"{api_url}/api/devices/{self._config.device_id}/stream/stop", timeout=5
            )
        except (OSError, RuntimeError):
            pass

        logger.info("Camera '%s' disconnected", self._name)

    # ------------------------------------------------------------------
    # REST API — camera hardware stream control
    # ------------------------------------------------------------------

    @staticmethod
    def _start_stream(api_url: str, cfg: WebRTCCameraConfig) -> None:
        """Start the camera hardware stream via REST API."""
        import requests  # noqa: PLC0415

        with contextlib.suppress(OSError, RuntimeError):
            requests.post(f"{api_url}/api/devices/{cfg.device_id}/stream/stop", timeout=5)

        sensors_url = f"{api_url}/api/devices/{cfg.device_id}/sensors/"
        resp = requests.get(sensors_url, timeout=10)
        resp.raise_for_status()
        sensors = resp.json()

        sensor_id = None
        for sensor in sensors:
            for profile in sensor.get("supported_stream_profiles", []):
                if profile.get("stream_type") == cfg.stream_type:
                    sensor_id = sensor.get("sensor_id")
                    break
            if sensor_id is not None:
                break

        if sensor_id is None:
            msg = f"No sensor supports stream_type='{cfg.stream_type}' on device {cfg.device_id}"
            raise RuntimeError(msg)

        fmt = "rgb8" if cfg.stream_type == "color" else "z16"
        payload = {
            "configs": [
                {
                    "stream_type": cfg.stream_type,
                    "format": fmt,
                    "resolution": {"width": cfg.width, "height": cfg.height},
                    "framerate": cfg.fps,
                    "sensor_id": sensor_id,
                }
            ]
        }
        resp = requests.post(
            f"{api_url}/api/devices/{cfg.device_id}/stream/start", json=payload, timeout=30
        )
        resp.raise_for_status()
        logger.info(
            "Camera '%s' stream started (%dx%d@%dfps)",
            cfg.device_id, cfg.width, cfg.height, cfg.fps,
        )

    # ------------------------------------------------------------------
    # WebRTC signaling + frame reception
    # ------------------------------------------------------------------

    async def _setup_webrtc(self, api_url: str, cfg: WebRTCCameraConfig) -> None:
        """Create WebRTC connection and start receiving frames."""
        import requests  # noqa: PLC0415
        from aiortc import RTCConfiguration  # noqa: PLC0415

        # No STUN/TURN — only host candidates on the local network.
        self._pc = RTCPeerConnection(
            configuration=RTCConfiguration(iceServers=[]),
        )

        @self._pc.on("track")
        def on_track(track: object) -> None:
            if hasattr(track, "kind") and track.kind == "video":
                self._receive_task = asyncio.ensure_future(self._receive_frames(track))

        resp = await asyncio.to_thread(
            requests.post,
            f"{api_url}/api/webrtc/offer",
            json={"device_id": cfg.device_id, "stream_types": [cfg.stream_type]},
            timeout=30,
        )
        resp.raise_for_status()
        offer_data = resp.json()

        # Filter remote SDP to only keep candidates matching the server IP.
        # Other interfaces (Docker bridge, VPN, IPv6) are unreachable from
        # Nova instances and cause 30-60s ICE connectivity check timeouts.
        from urllib.parse import urlparse  # noqa: PLC0415

        server_ip = urlparse(api_url).hostname or ""
        sdp = _filter_sdp_candidates(offer_data["sdp"], server_ip)
        offer = RTCSessionDescription(sdp=sdp, type=offer_data["type"])
        await self._pc.setRemoteDescription(offer)

        answer = await self._pc.createAnswer()
        await self._pc.setLocalDescription(answer)

        resp = await asyncio.to_thread(
            requests.post,
            f"{api_url}/api/webrtc/answer",
            json={
                "session_id": offer_data.get("session_id"),
                "sdp": self._pc.localDescription.sdp,
                "type": self._pc.localDescription.type,
            },
            timeout=30,
        )
        resp.raise_for_status()

    async def _receive_frames(self, track: object) -> None:
        """Continuously receive frames and store the latest one."""
        from aiortc.mediastreams import MediaStreamError  # noqa: PLC0415

        try:
            while True:
                frame = await track.recv()  # type: ignore[union-attr]
                img = frame.to_ndarray(format="bgr24")
                img_rgb = img[:, :, ::-1].copy()
                with self._frame_lock:
                    self._frame = img_rgb
                    self._frame_time = time.monotonic()
                if not self._frame_event.is_set():
                    self._frame_event.set()
        except (asyncio.CancelledError, MediaStreamError):
            pass
        except (OSError, RuntimeError) as e:
            logger.debug("Camera '%s' frame receiver stopped: %s", self._name, e)


_CANDIDATE_IP_INDEX = 4


def _filter_sdp_candidates(sdp: str, server_ip: str) -> str:
    """Keep only ICE candidates matching the server's IP.

    Remote ICE candidates (srflx, IPv6, other interfaces) cause long
    ICE connectivity check timeouts when unreachable.
    """
    lines = []
    for line in sdp.split("\r\n"):
        if line.startswith("a=candidate:"):
            parts = line.split()
            # parts[4] is the candidate IP address
            if len(parts) > _CANDIDATE_IP_INDEX and parts[_CANDIDATE_IP_INDEX] == server_ip:
                lines.append(line)
        else:
            lines.append(line)
    return "\r\n".join(lines)
