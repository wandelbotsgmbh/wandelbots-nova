"""The ``CameraSource`` protocol: any camera backend that feeds policy observations."""

from __future__ import annotations

from typing import TYPE_CHECKING, Protocol, runtime_checkable

if TYPE_CHECKING:
    from typing import TypeAlias

    import numpy as np
    from numpy.typing import NDArray

    CameraFrame: TypeAlias = NDArray[np.uint8]


@runtime_checkable
class LatestFrameSource(Protocol):
    """Camera source that exposes its latest frame without updating history."""

    def get_latest_frame(self, max_age_s: float = 5.0) -> CameraFrame:
        """Return the latest frame cached by the camera source."""
        raise NotImplementedError


@runtime_checkable
class CameraSource(Protocol):
    """Protocol for camera sources used by the PolicyExecutor: these three methods."""

    async def connect(self) -> None:
        """Connect to camera hardware. Called once before execution starts."""

    def read(self, max_age_s: float = 5.0) -> CameraFrame:
        """Read the latest frame from this camera.

        Args:
            max_age_s: Maximum acceptable frame age in seconds; older frames raise.

        Returns:
            ``(H, W, 3)`` uint8 RGB, or ``(T, H, W, 3)`` with frame history.

        Raises:
            RuntimeError: If no frame available or frame is stale.
        """
        raise NotImplementedError

    async def disconnect(self) -> None:
        """Release camera resources. Called after execution ends."""
