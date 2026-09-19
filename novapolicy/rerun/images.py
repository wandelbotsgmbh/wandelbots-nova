"""Camera image logging for Rerun."""

from __future__ import annotations

import time
from typing import Any

from novapolicy.rerun.constants import TEMPORAL_FRAME_NDIM
import rerun as rr


def log_images(images: dict[str, Any], *, start_time: float) -> None:
    """Log camera images to Rerun."""
    elapsed = time.monotonic() - start_time
    rr.set_time("policy_time", duration=elapsed)

    for name, frame in images.items():
        if frame is None:
            continue
        if hasattr(frame, "ndim"):
            img = frame[-1] if frame.ndim == TEMPORAL_FRAME_NDIM else frame
            rr.log(f"policy/cameras/{name}", rr.Image(img))
