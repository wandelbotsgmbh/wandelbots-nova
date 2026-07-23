"""WebRTC camera implementation.

Provides ``WebRTCCameras`` factory and ``WebRTCDevice`` — a ``CameraSource``
implementation that connects to cameras via WebRTC signaling.

The policy never starts or stops hardware streams — it only creates WebRTC
sessions to receive frames.  If the stream is not running, an exception is
raised after the connect timeout.
"""

from __future__ import annotations

import asyncio
import contextlib
from dataclasses import dataclass
import importlib
import logging
import threading
import time
from typing import TYPE_CHECKING, Any, cast

import numpy as np
import requests

if TYPE_CHECKING:
    from types import ModuleType

    from aiortc import MediaStreamTrack
    from av import VideoFrame
    from numpy.typing import NDArray

logger = logging.getLogger(__name__)

# Optional imports — aiortc is heavy and not always needed
_aiortc: ModuleType | None
try:
    _aiortc = importlib.import_module("aiortc")
    import av.logging

    av.logging.set_level(av.logging.ERROR)
except ImportError:
    # aiortc is an optional dependency; WebRTC camera support stays disabled.
    _aiortc = None


def _require_aiortc() -> None:
    """Raise if aiortc is not installed."""
    if _aiortc is None:
        msg = "aiortc is required for WebRTC cameras. Install with: pip install aiortc"
        raise ModuleNotFoundError(msg)


# ---------------------------------------------------------------------------
# Config
# ---------------------------------------------------------------------------


@dataclass
class WebRTCCameraConfig:
    """Configuration for a single WebRTC camera.

    Attributes:
        api_url: Base URL of the camera signaling API
            (e.g. ``"http://172.31.11.129:8011/webrtc-streamer"``).
        device_id: Camera device identifier.
        stream_type: Stream type to request (``"color"`` or ``"depth"``).
    """

    api_url: str
    device_id: str
    stream_type: str = "color"


# ---------------------------------------------------------------------------
# WebRTC connection
# ---------------------------------------------------------------------------


class WebRTCConnection:
    """Manages a single WebRTC peer connection with background frame reception.

    Only performs WebRTC signaling (offer/answer).  Never calls
    ``stream/start`` or ``stream/stop`` — the hardware stream is managed
    externally by the camera server.
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
        """Establish WebRTC connection and wait for first frame."""
        cfg = self._config
        api_url = cfg.api_url.rstrip("/")

        await self._setup_webrtc(api_url, cfg)

        try:
            await asyncio.wait_for(self._frame_event.wait(), timeout=timeout_s)
        except TimeoutError as e:
            await self.disconnect()
            msg = f"Camera '{self._name}' timed out waiting for first frame"
            raise RuntimeError(msg) from e

        logger.info("Camera '%s' connected (device=%s)", self._name, cfg.device_id)

    async def disconnect(self) -> None:
        """Close the WebRTC peer connection."""
        if self._receive_task is not None:
            self._receive_task.cancel()
            with contextlib.suppress(asyncio.CancelledError):
                await self._receive_task
            self._receive_task = None

        if self._pc is not None:
            with contextlib.suppress(TimeoutError, OSError):
                await self._pc.close()
            self._pc = None

        logger.info("Camera '%s' disconnected", self._name)

    # ------------------------------------------------------------------
    # WebRTC signaling
    # ------------------------------------------------------------------

    async def _setup_webrtc(self, api_url: str, cfg: WebRTCCameraConfig) -> None:
        """WebRTC offer/answer exchange."""
        _require_aiortc()
        from aiortc import (  # noqa: PLC0415
            RTCConfiguration,
            RTCPeerConnection,
            RTCSessionDescription,
        )

        self._pc = RTCPeerConnection(
            configuration=RTCConfiguration(iceServers=[]),
        )

        @self._pc.on("track")
        def on_track(track: MediaStreamTrack) -> None:
            if track.kind == "video":
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

    # ------------------------------------------------------------------
    # Frame reception
    # ------------------------------------------------------------------

    async def _receive_frames(self, track: MediaStreamTrack) -> None:
        """Receive frames and store the latest (thread-safe)."""
        from aiortc.mediastreams import MediaStreamError  # noqa: PLC0415

        try:  # noqa: PLW0717
            while True:
                frame = cast("VideoFrame", await track.recv())
                img = frame.to_ndarray(format="rgb24")

                with self._frame_lock:
                    self._frame = img
                    self._frame_time = time.monotonic()
                if not self._frame_event.is_set():
                    self._frame_event.set()
        except (asyncio.CancelledError, MediaStreamError):
            # Expected when the track ends or the task is cancelled on shutdown.
            pass
        except (OSError, RuntimeError) as e:
            logger.debug("Camera '%s' frame receiver stopped: %s", self._name, e)


# ---------------------------------------------------------------------------
# WebRTCDevice — CameraSource implementation
# ---------------------------------------------------------------------------


def _resize_frame(frame: NDArray[Any], width: int, height: int) -> NDArray[Any]:
    """Resize a single (H, W, 3) frame using Pillow (no system lib dependencies)."""
    from PIL import Image  # noqa: PLC0415

    img = Image.fromarray(frame)
    img = img.resize((width, height), Image.Resampling.LANCZOS)
    return np.asarray(img)


class WebRTCDevice:
    """A single WebRTC camera device implementing the CameraSource protocol.

    Created by ``WebRTCCameras.device()``.  Do not instantiate directly.
    """

    def __init__(
        self,
        config: WebRTCCameraConfig,
        *,
        frame_history: int = 1,
        resize: tuple[int, int] | None = None,
    ) -> None:
        self._config = config
        self._frame_history = frame_history
        self._resize = resize  # (width, height) or None
        self._connection: WebRTCConnection | None = None
        self._buffer: list[NDArray[Any]] = []

    async def connect(self, timeout_s: float = 30.0) -> None:
        """Connect to the camera stream via WebRTC."""
        _require_aiortc()
        self._connection = WebRTCConnection(self._config.device_id, self._config)
        await self._connection.connect(timeout_s=timeout_s)

    def read(self, max_age_s: float = 30.0) -> NDArray[Any]:
        """Read the latest frame.

        Returns:
            ``(H, W, 3)`` uint8 RGB if ``frame_history == 1``.
            ``(T, H, W, 3)`` uint8 RGB if ``frame_history > 1``.

        Raises:
            RuntimeError: If not connected, no frame available, or frame is stale.
        """
        frame = self._read_latest_frame(max_age_s)
        if self._frame_history <= 1:
            return frame

        self._buffer.append(frame)
        if len(self._buffer) > self._frame_history:
            self._buffer.pop(0)
        while len(self._buffer) < self._frame_history:
            self._buffer.insert(0, self._buffer[0])
        return np.stack(self._buffer, axis=0)

    def get_latest_frame(self, max_age_s: float = 30.0) -> NDArray[Any]:
        """Return the latest cached frame without advancing policy frame history."""
        return self._read_latest_frame(max_age_s)

    def _read_latest_frame(self, max_age_s: float) -> NDArray[Any]:
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

        if self._resize is not None:
            frame = _resize_frame(frame, self._resize[0], self._resize[1])
        return frame

    async def disconnect(self) -> None:
        """Disconnect from the camera stream."""
        if self._connection is not None:
            await self._connection.disconnect()
            self._connection = None
        self._buffer.clear()


# ---------------------------------------------------------------------------
# WebRTCCameras factory
# ---------------------------------------------------------------------------


class WebRTCCameras:
    """Factory for WebRTC camera sources.

    The camera server controls resolution, fps, and stream lifecycle.
    This factory only creates WebRTC sessions to receive frames.

    If ``resize`` is set, frames are resized on read using ``cv2.INTER_AREA``
    (best quality for downscaling).  This adds ~0.3ms per frame.

    Usage::

        cameras = WebRTCCameras(
            api_url="http://192.168.1.22:9100",
            resize=(256, 256),
        )

        schema = PolicySchema(
            observations=[
                Observation.image("cam_left", source=cameras.device("315122271048")),
                Observation.image("cam_right", source=cameras.device("319522063360")),
            ],
        )
    """

    def __init__(
        self,
        api_url: str,
        *,
        resize: tuple[int, int] | None = None,
        frame_history: int = 1,
    ) -> None:
        self._api_url = api_url
        self._resize = resize
        self._frame_history = frame_history

    def device(self, device_id: str, *, frame_history: int | None = None) -> WebRTCDevice:
        """Create a camera source for a specific device.

        Args:
            device_id: Camera device identifier.
            frame_history: Override the factory default for this device.
        """
        cfg = WebRTCCameraConfig(
            api_url=self._api_url,
            device_id=device_id,
        )
        return WebRTCDevice(
            cfg,
            frame_history=frame_history or self._frame_history,
            resize=self._resize,
        )
