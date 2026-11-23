"""Camera image logging for Rerun."""

from __future__ import annotations

from typing import TYPE_CHECKING, Any

from novapolicy.rerun.constants import CAMERA_JPEG_QUALITY, TEMPORAL_FRAME_NDIM
import rerun as rr

if TYPE_CHECKING:
    from novapolicy.rerun.context import RerunContext


def log_images(ctx: RerunContext, images: dict[str, Any]) -> None:
    """Log camera images to Rerun, the newest frame of a frame history."""
    ctx.set_time()
    for name, frame in images.items():
        if frame is None or not hasattr(frame, "ndim"):
            continue
        img = frame[-1] if frame.ndim == TEMPORAL_FRAME_NDIM else frame
        rr.log(
            f"policy/cameras/{name}",
            rr.Image(img).compress(jpeg_quality=CAMERA_JPEG_QUALITY),
            recording=ctx.recording,
        )
