"""Camera image logging for Rerun."""

from __future__ import annotations

import time
from typing import TYPE_CHECKING, Any

from novapolicy.rerun.constants import CAMERA_JPEG_QUALITY, TEMPORAL_FRAME_NDIM
import rerun as rr

if TYPE_CHECKING:
    from rerun import RecordingStream


def log_images(
    images: dict[str, Any],
    *,
    start_time: float,
    recording: RecordingStream | None,
) -> None:
    """Log camera images to Rerun."""
    elapsed = time.monotonic() - start_time
    rr.set_time("policy_time", duration=elapsed, recording=recording)

    for name, frame in images.items():
        if frame is None:
            continue
        if hasattr(frame, "ndim"):
            img = frame[-1] if frame.ndim == TEMPORAL_FRAME_NDIM else frame
            rr.log(
                f"policy/cameras/{name}",
                rr.Image(img).compress(jpeg_quality=CAMERA_JPEG_QUALITY),
                recording=recording,
            )
