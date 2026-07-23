"""Shared constants and color utilities for Rerun visualization."""

from __future__ import annotations

MIN_LINE_STEPS = 2
MIN_TCP_COMPONENTS = 3
TEMPORAL_FRAME_NDIM = 4
CAMERA_JPEG_QUALITY = 90
TCP_TRAIL_COLOR = (50, 220, 100)  # green — actual TCP path
TCP_TARGET_TRAIL_COLOR = (255, 180, 40)  # amber — commanded TCP path
TCP_ERROR_VECTOR_COLOR = (255, 80, 120)  # rose — actual-to-target error

# Action chunk gradient: orange (start) → yellow (end)
CHUNK_COLOR_START = (255, 80, 20)
CHUNK_COLOR_END = (255, 240, 60)

# Interpolated bridge: Nova Violet (separate from policy output)
BRIDGE_COLOR_START = (142, 86, 252)  # nova-violet5 / primary-main
BRIDGE_COLOR_END = (194, 183, 248)  # nova-violet7
BRIDGE_ENDPOINT_COLOR = BRIDGE_COLOR_END

# Screen-space line widths (UI points, zoom-independent)
TRAIL_WIDTH_UI = 2.0
CHUNK_WIDTH_UI = 3.0
BRIDGE_WIDTH_UI = 3.0
BRIDGE_ENDPOINT_RADIUS_UI = 5.0

# Discarded chunk tail: dim gray (predicted but not executed)
CHUNK_TAIL_COLOR = (100, 100, 100)
CHUNK_TAIL_WIDTH_UI = 1.5


def lerp_color(
    start: tuple[int, int, int], end: tuple[int, int, int], t: float
) -> tuple[int, int, int]:
    """Linearly interpolate between two RGB colors. t in [0, 1]."""
    return (
        int(start[0] + (end[0] - start[0]) * t),
        int(start[1] + (end[1] - start[1]) * t),
        int(start[2] + (end[2] - start[2]) * t),
    )
