"""
Viewer implementations for Nova programs.

This module provides different viewer backends that can be used
to visualize and monitor Nova programs during execution.
"""

from __future__ import annotations

# Public API exports
from .base import Viewer
from .manager import ViewerManager, get_viewer_manager
from .manager import cleanup_active_viewers as _cleanup_active_viewers
from .manager import configure_active_viewers as _configure_active_viewers
from .manager import log_planning_error_to_viewers as _log_planning_error_to_viewers
from .manager import log_planning_results_to_viewers as _log_planning_results_to_viewers
from .manager import (
    setup_active_viewers_after_preconditions as _setup_active_viewers_after_preconditions,
)
from .protocol import NovaRerunBridgeProtocol
from .rerun import Rerun
from .utils import extract_collision_scenes_from_actions as _extract_collision_scenes_from_actions


# Legacy API compatibility - keep the underscore prefix for internal functions
def _register_viewer(viewer: Viewer) -> None:
    """Register a viewer as active. (Legacy function for backward compatibility)"""
    from .manager import register_viewer

    register_viewer(viewer)


# For convenience, allow direct import of Rerun
__all__ = [
    "Viewer",
    "ViewerManager",
    "Rerun",
    "NovaRerunBridgeProtocol",
    "get_viewer_manager",
    "_configure_active_viewers",
    "_setup_active_viewers_after_preconditions",
    "_cleanup_active_viewers",
    "_log_planning_results_to_viewers",
    "_log_planning_error_to_viewers",
    "_extract_collision_scenes_from_actions",
    "_register_viewer",
]
