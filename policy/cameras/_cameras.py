"""Camera sources for policy observations.

Defines the ``CameraSource`` protocol and the ``CameraSet`` implementation
that connects to cameras via WebRTC.

The policy receives images as numpy arrays with shape ``(H, W, 3)`` in RGB
format, keyed by camera name — the same format LeRobot uses.
"""

from __future__ import annotations

import asyncio
import logging
from dataclasses import dataclass, field
from typing import TYPE_CHECKING, Any, Protocol, runtime_checkable

import numpy as np

if TYPE_CHECKING:
    from numpy.typing import NDArray

logger = logging.getLogger(__name__)


# ---------------------------------------------------------------------------
# Protocol — what the executor needs from any camera source
# ---------------------------------------------------------------------------


@runtime_checkable
class CameraSource(Protocol):
    """Protocol for camera sources used by the PolicyExecutor.

    Any object implementing these three methods can provide images to the
    executor. The built-in ``CameraSet`` uses WebRTC, but you can implement
    this with OpenCV, ROS, RealSense SDK, dataset replay, etc.

    Example (minimal OpenCV implementation)::

        class USBCamera:
            def __init__(self, names_to_indices: dict[str, int]):
                self._caps = {name: idx for name, idx in names_to_indices.items()}
                self._streams: dict[str, cv2.VideoCapture] = {}

            async def connect(self) -> None:
                import cv2
                for name, idx in self._caps.items():
                    self._streams[name] = cv2.VideoCapture(idx)

            def read(self) -> dict[str, NDArray]:
                import cv2
                frames = {}
                for name, cap in self._streams.items():
                    ret, bgr = cap.read()
                    frames[name] = cv2.cvtColor(bgr, cv2.COLOR_BGR2RGB)
                return frames

            async def disconnect(self) -> None:
                for cap in self._streams.values():
                    cap.release()
                self._streams.clear()
    """

    async def connect(self) -> None:
        """Connect to camera hardware. Called once before execution starts."""
        ...

    def read(self, max_age_s: float = 5.0) -> dict[str, NDArray[Any]]:
        """Read the latest frame from each camera.

        Args:
            max_age_s: Maximum acceptable frame age in seconds.
                Implementations should raise if a frame is older than this.

        Returns:
            Dict mapping camera name → numpy array.
            Typically ``(H, W, 3)`` uint8 RGB, but shape depends on implementation.

        Raises:
            RuntimeError: If a camera has no frame available or frame is stale.
        """
        ...

    async def disconnect(self) -> None:
        """Release camera resources. Called after execution ends."""
        ...


# ---------------------------------------------------------------------------
# WebRTC config
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
# CameraSet — named collection of WebRTC cameras
# ---------------------------------------------------------------------------


@dataclass
class CameraSet:
    """A named collection of cameras to connect and read from.

    Usage (verbose)::

        cameras = CameraSet(configs={
            "flange": WebRTCCameraConfig(api_url="...", device_id="315122271048"),
        })

    Usage (compact — shared api_url/resolution/fps)::

        cameras = CameraSet(
            api_url="http://localhost:9100",
            devices={"flange": "315122271048", "left": "314522065367"},
        )

    When ``frame_history > 1``, each camera maintains a buffer of the last N frames.
    ``read()`` then returns arrays with shape ``(T, H, W, 3)`` instead of ``(H, W, 3)``.
    """

    configs: dict[str, WebRTCCameraConfig] = field(default_factory=dict)
    frame_history: int = 1

    def __init__(
        self,
        configs: dict[str, WebRTCCameraConfig] | None = None,
        *,
        api_url: str = "",
        devices: dict[str, str] | None = None,
        width: int = 640,
        height: int = 480,
        fps: int = 30,
        frame_history: int = 1,
    ) -> None:
        self.frame_history = frame_history
        self._connections: dict[str, Any] = {}  # name → WebRTCConnection
        self._buffers: dict[str, list[NDArray[Any]]] = {}

        if configs:
            self.configs = configs
        elif devices and api_url:
            self.configs = {
                name: WebRTCCameraConfig(
                    api_url=api_url, device_id=device_id, width=width, height=height, fps=fps,
                )
                for name, device_id in devices.items()
            }
        else:
            self.configs = {}

    async def connect(self, timeout_s: float = 30.0) -> None:
        """Connect all cameras and wait for first frames.

        Raises RuntimeError if any camera fails to connect or produce a frame
        within ``timeout_s`` seconds.
        """
        from policy.cameras.webrtc import WebRTCConnection, require_aiortc  # noqa: PLC0415

        require_aiortc()

        tasks = []
        for name, cfg in self.configs.items():
            conn = WebRTCConnection(name, cfg)
            self._connections[name] = conn
            tasks.append(conn.connect(timeout_s=timeout_s))

        await asyncio.gather(*tasks)
        logger.info("All %d cameras connected and producing frames", len(self.configs))

    def read(self, max_age_s: float = 5.0) -> dict[str, NDArray[Any]]:
        """Read frames from each camera.

        Args:
            max_age_s: Maximum age of a frame in seconds before raising.

        Returns:
            Dict mapping camera name → numpy array.
            - If ``frame_history == 1``: shape ``(H, W, 3)`` uint8 RGB.
            - If ``frame_history > 1``: shape ``(T, H, W, 3)`` uint8 RGB.

        Raises:
            RuntimeError: If a camera has no frame or frame is stale.
        """
        frames: dict[str, NDArray[Any]] = {}
        for name, conn in self._connections.items():
            frame = conn.latest_frame()
            if frame is None:
                msg = f"Camera '{name}' has no frame available"
                raise RuntimeError(msg)

            age = conn.frame_age_s()
            if age > max_age_s:
                msg = f"Camera '{name}' frame is stale ({age:.1f}s old, limit {max_age_s:.1f}s)"
                raise RuntimeError(msg)

            if self.frame_history <= 1:
                frames[name] = frame
            else:
                buf = self._buffers.setdefault(name, [])
                buf.append(frame)
                if len(buf) > self.frame_history:
                    buf.pop(0)
                while len(buf) < self.frame_history:
                    buf.insert(0, buf[0])
                frames[name] = np.stack(buf, axis=0)

        return frames

    def is_ready(self) -> bool:
        """Check if all cameras have at least one frame available."""
        return all(conn.latest_frame() is not None for conn in self._connections.values())

    async def disconnect(self) -> None:
        """Disconnect all cameras and release resources."""
        tasks = [conn.disconnect() for conn in self._connections.values()]
        await asyncio.gather(*tasks, return_exceptions=True)
        self._connections.clear()
        logger.info("All cameras disconnected")


# ---------------------------------------------------------------------------
# WebRTCCameras factory — creates per-device CameraSource objects
# ---------------------------------------------------------------------------


class WebRTCDevice:
    """A single WebRTC camera device that implements the CameraSource protocol.

    Created by ``WebRTCCameras.device()``. Do not instantiate directly.
    """

    def __init__(self, config: WebRTCCameraConfig, *, frame_history: int = 1) -> None:
        self._config = config
        self._frame_history = frame_history
        self._connection: Any = None
        self._buffer: list[NDArray[Any]] = []

    async def connect(self, timeout_s: float = 30.0) -> None:
        """Connect this camera via WebRTC."""
        from policy.cameras.webrtc import WebRTCConnection, require_aiortc  # noqa: PLC0415

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
