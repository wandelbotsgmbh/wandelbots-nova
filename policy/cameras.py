"""WebRTC camera client for policy observations.

Connects to camera devices via a WebRTC REST API and continuously receives
frames in a background thread. The latest frame is always available for
the policy executor to include in observations.

Camera config format (matching LeRobot conventions)::

    cameras = {
        "flange": WebRTCCameraConfig(
            api_url="http://172.31.11.129:8000",
            device_id="315122271048",
            width=640,
            height=480,
            fps=30,
        ),
        "left": WebRTCCameraConfig(
            api_url="http://172.31.11.129:8000",
            device_id="314522065367",
            width=640,
            height=480,
            fps=30,
        ),
    }

The policy receives images as numpy arrays with shape (H, W, 3) in RGB format,
keyed by camera name — the same format LeRobot uses.
"""

from __future__ import annotations

import asyncio
import contextlib
import logging
import threading
from dataclasses import dataclass, field
from typing import TYPE_CHECKING, Any

if TYPE_CHECKING:
    from numpy.typing import NDArray

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


@dataclass
class CameraSet:
    """A named collection of cameras to connect and read from.

    Usage::

        cameras = CameraSet(configs={
            "flange": WebRTCCameraConfig(api_url="...", device_id="315122271048"),
            "left": WebRTCCameraConfig(api_url="...", device_id="314522065367"),
        })
        await cameras.connect()
        frames = cameras.read()  # {"flange": np.array(...), "left": np.array(...)}
        await cameras.disconnect()
    """

    configs: dict[str, WebRTCCameraConfig] = field(default_factory=dict)
    _connections: dict[str, _WebRTCConnection] = field(default_factory=dict, repr=False)

    async def connect(self, timeout_s: float = 30.0) -> None:
        """Connect all cameras and wait for first frames.

        Raises RuntimeError if any camera fails to connect or produce a frame
        within ``timeout_s`` seconds.
        """
        if not _aiortc_available:
            msg = "aiortc is required for WebRTC cameras. Install with: pip install aiortc"
            raise ModuleNotFoundError(msg)

        tasks = []
        for name, cfg in self.configs.items():
            conn = _WebRTCConnection(name, cfg)
            self._connections[name] = conn
            tasks.append(conn.connect(timeout_s=timeout_s))

        await asyncio.gather(*tasks)
        logger.info("All %d cameras connected and producing frames", len(self.configs))

    def read(self) -> dict[str, NDArray[Any]]:
        """Read the latest frame from each camera.

        Returns:
            Dict mapping camera name → numpy array (H, W, 3) uint8 RGB.

        Raises:
            RuntimeError: If a camera has no frame available.
        """
        frames: dict[str, NDArray[Any]] = {}
        for name, conn in self._connections.items():
            frame = conn.latest_frame()
            if frame is None:
                msg = f"Camera '{name}' has no frame available"
                raise RuntimeError(msg)
            frames[name] = frame
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


class _WebRTCConnection:
    """Manages a single WebRTC camera connection with background frame reception."""

    def __init__(self, name: str, config: WebRTCCameraConfig) -> None:
        self._name = name
        self._config = config
        self._pc: Any = None
        self._frame: NDArray[Any] | None = None
        self._frame_lock = threading.Lock()
        self._frame_event = asyncio.Event()
        self._loop: asyncio.AbstractEventLoop | None = None
        self._loop_thread: threading.Thread | None = None
        self._receive_task: asyncio.Task[None] | None = None

    def latest_frame(self) -> NDArray[Any] | None:
        """Get the most recent frame (thread-safe)."""
        with self._frame_lock:
            return self._frame

    async def connect(self, timeout_s: float = 30.0) -> None:
        """Establish WebRTC connection, start stream, wait for first frame."""
        cfg = self._config
        api_url = cfg.api_url.rstrip("/")

        # 1. Start camera stream
        await asyncio.to_thread(self._start_stream, api_url, cfg)

        # 2. Create dedicated event loop for WebRTC
        self._loop = asyncio.new_event_loop()
        self._loop_thread = threading.Thread(
            target=self._loop.run_forever, daemon=True, name=f"camera-{self._name}"
        )
        self._loop_thread.start()

        # 3. Set up WebRTC peer connection in the background loop
        future = asyncio.run_coroutine_threadsafe(self._setup_webrtc(api_url, cfg), self._loop)
        try:
            future.result(timeout=timeout_s)
        except Exception as e:
            await self.disconnect()
            msg = f"Camera '{self._name}' WebRTC setup failed: {e}"
            raise RuntimeError(msg) from e

        # 4. Wait for first frame
        try:
            await asyncio.wait_for(self._frame_event.wait(), timeout=timeout_s)
        except TimeoutError as e:
            await self.disconnect()
            msg = f"Camera '{self._name}' timed out waiting for first frame"
            raise RuntimeError(msg) from e

        logger.info("Camera '%s' connected (device=%s)", self._name, cfg.device_id)

    async def disconnect(self) -> None:
        """Close peer connection and stop background loop."""
        # Cancel the frame receiver task first so it doesn't raise MediaStreamError
        if self._receive_task is not None and self._loop is not None:
            self._loop.call_soon_threadsafe(self._receive_task.cancel)
            await asyncio.sleep(0.1)  # Give it a moment to cancel
            self._receive_task = None

        if self._pc is not None and self._loop is not None:
            try:
                future = asyncio.run_coroutine_threadsafe(self._pc.close(), self._loop)
                future.result(timeout=5.0)
            except (TimeoutError, OSError):
                pass
            self._pc = None

        if self._loop is not None:
            self._loop.call_soon_threadsafe(self._loop.stop)
            if self._loop_thread is not None:
                self._loop_thread.join(timeout=2.0)
            self._loop = None
            self._loop_thread = None

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

    @staticmethod
    def _start_stream(api_url: str, cfg: WebRTCCameraConfig) -> None:
        """Start the camera hardware stream via REST API."""
        import requests  # noqa: PLC0415

        # Stop any existing stream
        with contextlib.suppress(OSError, RuntimeError):
            requests.post(f"{api_url}/api/devices/{cfg.device_id}/stream/stop", timeout=5)

        # Get sensor info
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

        # Start stream
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
        logger.info("Camera '%s' stream started (%dx%d@%dfps)", cfg.device_id, cfg.width, cfg.height, cfg.fps)

    async def _setup_webrtc(self, api_url: str, cfg: WebRTCCameraConfig) -> None:
        """Create WebRTC connection and start receiving frames."""
        import requests  # noqa: PLC0415

        self._pc = RTCPeerConnection()

        @self._pc.on("track")
        def on_track(track: object) -> None:
            if hasattr(track, "kind") and track.kind == "video":
                self._receive_task = asyncio.ensure_future(self._receive_frames(track))

        # Request offer from server
        resp = await asyncio.to_thread(
            requests.post,
            f"{api_url}/api/webrtc/offer",
            json={"device_id": cfg.device_id, "stream_types": [cfg.stream_type]},
            timeout=30,
        )
        resp.raise_for_status()
        offer_data = resp.json()

        # Set remote description (server's offer)
        offer = RTCSessionDescription(sdp=offer_data["sdp"], type=offer_data["type"])
        await self._pc.setRemoteDescription(offer)

        # Create and send answer
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
        import cv2  # noqa: PLC0415
        from aiortc.mediastreams import MediaStreamError  # noqa: PLC0415

        try:
            while True:
                frame = await track.recv()  # type: ignore[union-attr]
                # Convert to numpy BGR then to RGB
                img = frame.to_ndarray(format="bgr24")
                img_rgb = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
                with self._frame_lock:
                    self._frame = img_rgb
                # Signal first frame
                if not self._frame_event.is_set():
                    self._frame_event.set()
        except (asyncio.CancelledError, MediaStreamError):
            pass  # Normal shutdown — track closed by disconnect()
        except (OSError, RuntimeError) as e:
            logger.debug("Camera '%s' frame receiver stopped: %s", self._name, e)
