"""WebRTC camera implementation.

Provides ``WebRTCCameras`` factory and ``WebRTCDevice`` — a ``CameraSource``
implementation that connects to cameras via WebRTC signaling.

The policy never starts or stops hardware streams — it only creates WebRTC
sessions to receive frames.  If the stream is not running or delivers the
wrong resolution, an exception is raised.
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


def _require_aiortc() -> None:
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
        api_url: Base URL of the camera signaling API
            (e.g. ``"http://172.31.11.129:8011/webrtc-streamer"``).
        device_id: Camera device identifier.
        width: Expected frame width.  If the stream delivers a different
            resolution, a ``RuntimeError`` is raised on the first frame.
            ``None`` = accept any resolution.
        height: Expected frame height.
        fps: Expected frames per second.  Validated by measuring the
            interval between the first frames.  ``None`` = accept any fps.
        stream_type: Stream type to request (``"color"`` or ``"depth"``).
    """

    api_url: str
    device_id: str
    width: int | None = None
    height: int | None = None
    fps: int | None = None
    stream_type: str = "color"


# ---------------------------------------------------------------------------
# Stream validation
# ---------------------------------------------------------------------------

_FPS_SAMPLE_FRAMES = 10
_FPS_TOLERANCE = 0.25  # 25% deviation allowed


class _StreamValidator:
    """Validates resolution on first frame and fps over first N frames."""

    def __init__(self, name: str, config: WebRTCCameraConfig) -> None:
        self._name = name
        self._expected_w = config.width
        self._expected_h = config.height
        self._expected_fps = config.fps
        self._resolution_ok = self._expected_w is None and self._expected_h is None
        self._fps_ok = self._expected_fps is None
        self._first_frame_time: float | None = None
        self._frame_count = 0

    def check(self, img: NDArray[Any]) -> str | None:
        """Check a frame. Returns error message or None."""
        self._frame_count += 1

        if not self._resolution_ok:
            h, w = img.shape[0], img.shape[1]
            if (self._expected_w and w != self._expected_w) or (
                self._expected_h and h != self._expected_h
            ):
                return (
                    f"Camera '{self._name}' streams {w}x{h} "
                    f"but policy expects {self._expected_w}x{self._expected_h}. "
                    f"Configure the camera server to stream at the required resolution."
                )
            self._resolution_ok = True

        if not self._fps_ok:
            if self._first_frame_time is None:
                self._first_frame_time = time.monotonic()
            elif self._frame_count >= _FPS_SAMPLE_FRAMES:
                elapsed = time.monotonic() - self._first_frame_time
                measured = (self._frame_count - 1) / elapsed if elapsed > 0 else 0
                if abs(measured - self._expected_fps) > self._expected_fps * _FPS_TOLERANCE:
                    return (
                        f"Camera '{self._name}' streams at {measured:.1f} fps "
                        f"but policy expects {self._expected_fps} fps. "
                        f"Configure the camera server to stream at the required fps."
                    )
                self._fps_ok = True

        return None


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
        self._resolution_error: str | None = None
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

        if self._resolution_error:
            await self.disconnect()
            raise RuntimeError(self._resolution_error)

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

    # ------------------------------------------------------------------
    # Frame reception
    # ------------------------------------------------------------------

    async def _receive_frames(self, track: object) -> None:
        """Receive frames, validate resolution and fps."""
        validator = _StreamValidator(self._name, self._config)

        try:
            while True:
                frame = await track.recv()  # type: ignore[union-attr]
                img = frame.to_ndarray(format="rgb24")

                error = validator.check(img)
                if error:
                    self._resolution_error = error
                    self._frame_event.set()
                    return

                with self._frame_lock:
                    self._frame = img
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


def _resize_frame(frame: NDArray[Any], width: int, height: int) -> NDArray[Any]:
    """Resize a single (H, W, 3) frame using INTER_AREA (best for downscaling)."""
    import cv2  # noqa: PLC0415 — lazy import, cv2 is heavy

    return cv2.resize(frame, (width, height), interpolation=cv2.INTER_AREA)


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

        if self._frame_history <= 1:
            return frame

        self._buffer.append(frame)
        if len(self._buffer) > self._frame_history:
            self._buffer.pop(0)
        while len(self._buffer) < self._frame_history:
            self._buffer.insert(0, self._buffer[0])
        return np.stack(self._buffer, axis=0)

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
        width: int | None = None,
        height: int | None = None,
        fps: int | None = None,
        frame_history: int = 1,
    ) -> None:
        self._api_url = api_url
        self._resize = resize
        self._width = width
        self._height = height
        self._fps = fps
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
            width=self._width,
            height=self._height,
            fps=self._fps,
        )
        return WebRTCDevice(
            cfg,
            frame_history=frame_history or self._frame_history,
            resize=self._resize,
        )
