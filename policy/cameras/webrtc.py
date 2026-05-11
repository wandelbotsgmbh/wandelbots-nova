"""WebRTC camera implementation.

Provides ``WebRTCCameras`` factory and ``WebRTCDevice`` — a ``CameraSource``
implementation that connects to cameras via WebRTC signaling.
"""

from __future__ import annotations

import asyncio
import contextlib
from dataclasses import dataclass
import logging
import threading
import time
from typing import TYPE_CHECKING, Any

import numpy as np
import requests

if TYPE_CHECKING:
    from numpy.typing import NDArray

logger = logging.getLogger(__name__)

# Optional imports — aiortc is heavy and not always needed
_aiortc_available = False
try:
    from aiortc import RTCConfiguration, RTCPeerConnection, RTCSessionDescription
    from aiortc.mediastreams import MediaStreamError
    import av.logging

    av.logging.set_level(av.logging.ERROR)
    _aiortc_available = True
except ImportError:
    pass


def require_aiortc() -> None:
    """Raise if aiortc is not installed."""
    if not _aiortc_available:
        msg = "aiortc is required for WebRTC cameras. Install with: pip install aiortc"
        raise ModuleNotFoundError(msg)


# ---------------------------------------------------------------------------
# Config
# ---------------------------------------------------------------------------


@dataclass
class WebRTCCameraConfig:
    """Configuration for a single WebRTC camera.

    Attributes:
        api_url: Base URL of the camera REST API (e.g. "http://172.31.11.129:8000").
        device_id: Camera device identifier (e.g. serial number "315122271048").
        width: Frame width in pixels.
        height: Frame height in pixels.
        fps: Desired frames per second.
        stream_type: Stream type to request ("color" or "depth").
    """

    api_url: str
    device_id: str
    width: int = 640
    height: int = 480
    fps: int = 30
    stream_type: str = "color"


# ---------------------------------------------------------------------------
# WebRTC connection
# ---------------------------------------------------------------------------


class WebRTCConnection:
    """Manages a single WebRTC camera connection with background frame reception.

    Handles the full lifecycle:
    1. Start camera hardware stream via REST API
    2. WebRTC offer/answer exchange
    3. Background frame reception task
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

        # 2. Set up WebRTC peer connection
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

        offer = RTCSessionDescription(sdp=offer_data["sdp"], type=offer_data["type"])
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


# ---------------------------------------------------------------------------
# WebRTCDevice — CameraSource implementation
# ---------------------------------------------------------------------------


class WebRTCDevice:
    """A single WebRTC camera device that implements the CameraSource protocol.

    Created by ``WebRTCCameras.device()``. Do not instantiate directly.
    """

    def __init__(self, config: WebRTCCameraConfig, *, frame_history: int = 1) -> None:
        self._config = config
        self._frame_history = frame_history
        self._connection: WebRTCConnection | None = None
        self._buffer: list[NDArray[Any]] = []

    async def connect(self, timeout_s: float = 30.0) -> None:
        """Connect this camera via WebRTC."""
        require_aiortc()
        self._connection = WebRTCConnection(self._config.device_id, self._config)
        await self._connection.connect(timeout_s=timeout_s)
        logger.info("WebRTCDevice '%s' connected", self._config.device_id)

    def read(self, max_age_s: float = 30.0) -> NDArray[Any]:
        """Read the latest frame (or stacked frames if frame_history > 1).

        Returns:
            ``(H, W, 3)`` uint8 RGB if ``frame_history == 1``.
            ``(T, H, W, 3)`` uint8 RGB if ``frame_history > 1``.

        Raises:
            RuntimeError: If no frame available or frame is stale.
        """
        if self._connection is None:
            msg = f"Camera '{self._config.device_id}' not connected"
            raise RuntimeError(msg)

        frame = self._connection.latest_frame()
        if frame is None:
            msg = f"Camera '{self._config.device_id}' has no frame"
            raise RuntimeError(msg)

        age = self._connection.frame_age_s()
        if age > max_age_s:
            msg = f"Camera '{self._config.device_id}' frame stale ({age:.1f}s > {max_age_s:.1f}s)"
            raise RuntimeError(msg)

        if self._frame_history <= 1:
            return frame

        self._buffer.append(frame)
        if len(self._buffer) > self._frame_history:
            self._buffer.pop(0)
        while len(self._buffer) < self._frame_history:
            self._buffer.insert(0, self._buffer[0])
        return np.stack(self._buffer, axis=0)

    async def disconnect(self) -> None:
        """Disconnect this camera."""
        if self._connection is not None:
            await self._connection.disconnect()
            self._connection = None
        self._buffer.clear()


# ---------------------------------------------------------------------------
# WebRTCCameras factory
# ---------------------------------------------------------------------------


class WebRTCCameras:
    """Factory for WebRTC camera sources.

    Shared camera settings (server URL, resolution, fps) are configured once.
    Call ``.device(id)`` to create a per-device ``CameraSource``.

    Usage::

        webrtc = WebRTCCameras(
            api_url="http://192.168.1.22:9100",
            width=224, height=224, fps=15,
        )

        schema = PolicySchema(
            observations=[
                Observation.image("cam_left", source=webrtc.device("315122271048")),
                Observation.image("cam_right", source=webrtc.device("319522063360")),
            ],
        )
    """

    def __init__(
        self,
        api_url: str,
        *,
        width: int = 640,
        height: int = 480,
        fps: int = 30,
        frame_history: int = 1,
    ) -> None:
        self._api_url = api_url
        self._width = width
        self._height = height
        self._fps = fps
        self._frame_history = frame_history

    def device(self, device_id: str, *, frame_history: int | None = None) -> WebRTCDevice:
        """Create a camera source for a specific device.

        Args:
            device_id: Camera device identifier (serial number, Isaac Sim path, etc.).
            frame_history: Override the factory default for this device.
        """
        cfg = WebRTCCameraConfig(
            api_url=self._api_url,
            device_id=device_id,
            width=self._width,
            height=self._height,
            fps=self._fps,
        )
        return WebRTCDevice(cfg, frame_history=frame_history or self._frame_history)


# ---------------------------------------------------------------------------
