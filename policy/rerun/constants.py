"""Shared constants and color utilities for Rerun visualization."""

from __future__ import annotations

__all__ = [
    "_CHUNK_COLOR_END",
    "_CHUNK_COLOR_START",
    "_CHUNK_TAIL_COLOR",
    "_CHUNK_TAIL_WIDTH_UI",
    "_CHUNK_WIDTH_UI",
    "_MIN_LINE_STEPS",
    "_MIN_TCP_COMPONENTS",
    "_TCP_TRAIL_COLOR",
    "_TEMPORAL_FRAME_NDIM",
    "_TRAIL_WIDTH_UI",
    "lerp_color",
]

_MIN_LINE_STEPS = 2
_MIN_TCP_COMPONENTS = 3
_TEMPORAL_FRAME_NDIM = 4
_TCP_TRAIL_COLOR = (50, 220, 100)  # green — actual TCP path

# Action chunk gradient: orange (start) → yellow (end)
_CHUNK_COLOR_START = (255, 80, 20)
_CHUNK_COLOR_END = (255, 240, 60)

# Screen-space line widths (UI points, zoom-independent)
_TRAIL_WIDTH_UI = 2.0
_CHUNK_WIDTH_UI = 3.0

# Discarded chunk tail: dim gray (predicted but not executed)
_CHUNK_TAIL_COLOR = (100, 100, 100)
_CHUNK_TAIL_WIDTH_UI = 1.5


def lerp_color(
    start: tuple[int, int, int], end: tuple[int, int, int], t: float
) -> tuple[int, int, int]:
    """Linearly interpolate between two RGB colors. t in [0, 1]."""
    return (
        int(start[0] + (end[0] - start[0]) * t),
        int(start[1] + (end[1] - start[1]) * t),
        int(start[2] + (end[2] - start[2]) * t),
    )
