"""Shared constants and color utilities for Rerun visualization."""

from __future__ import annotations

MIN_LINE_STEPS = 2
MIN_TCP_COMPONENTS = 3
TEMPORAL_FRAME_NDIM = 4
TCP_TRAIL_COLOR = (50, 220, 100)  # green — actual TCP path
TCP_TARGET_TRAIL_COLOR = (255, 220, 40)  # yellow — commanded TCP path
TCP_ERROR_VECTOR_COLOR = (255, 60, 60)  # red — actual→commanded TCP error

# Action chunk gradient: orange (start) → yellow (end)
CHUNK_COLOR_START = (255, 80, 20)
CHUNK_COLOR_END = (255, 240, 60)

# Screen-space line widths (UI points, zoom-independent)
TRAIL_WIDTH_UI = 2.0
CHUNK_WIDTH_UI = 3.0

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
