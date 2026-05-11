"""Camera sources for policy observations.

Defines the ``CameraSource`` protocol for plugging any camera backend
(USB, ROS, RealSense, dataset replay, WebRTC, etc.) into the policy executor.
"""

from __future__ import annotations

from typing import TYPE_CHECKING, Any, Protocol, runtime_checkable

if TYPE_CHECKING:
    from numpy.typing import NDArray


@runtime_checkable
class CameraSource(Protocol):
    """Protocol for camera sources used by the PolicyExecutor.

    Any object implementing these three methods can provide images to the
    executor.

    Example (minimal OpenCV implementation)::

        class USBCamera:
            def __init__(self, device_index: int):
                self._index = device_index
                self._cap = None

            async def connect(self) -> None:
                import cv2
                self._cap = cv2.VideoCapture(self._index)

            def read(self, max_age_s: float = 5.0) -> NDArray:
                ret, bgr = self._cap.read()
                return bgr[:, :, ::-1].copy()  # BGR→RGB

            async def disconnect(self) -> None:
                if self._cap:
                    self._cap.release()
    """

    async def connect(self) -> None:
        """Connect to camera hardware. Called once before execution starts."""
        ...

    def read(self, max_age_s: float = 5.0) -> NDArray[Any]:
        """Read the latest frame from this camera.

        Args:
            max_age_s: Maximum acceptable frame age in seconds.
                Implementations should raise if a frame is older than this.

        Returns:
            Numpy array, typically ``(H, W, 3)`` uint8 RGB.
            May be ``(T, H, W, 3)`` if frame history is enabled.

        Raises:
            RuntimeError: If no frame available or frame is stale.
        """
        ...

    async def disconnect(self) -> None:
        """Release camera resources. Called after execution ends."""
        ...
