"""Viewer manager for coordinating multiple viewers."""

from __future__ import annotations

import logging
from typing import TYPE_CHECKING, Sequence
from weakref import WeakSet

if TYPE_CHECKING:
    from nova.actions import Action
    from nova.api import models
    from nova.core.motion_group import MotionGroup
    from nova.core.nova import Nova

from .base import Viewer

logger = logging.getLogger(__name__)


class ViewerManager:
    """Manages the lifecycle and coordination of all active viewers."""

    def __init__(self) -> None:
        self._viewers: WeakSet[Viewer] = WeakSet()

    def register_viewer(self, viewer: Viewer) -> None:
        """Register a viewer as active."""
        self._viewers.add(viewer)

    def configure_viewers(self, nova: Nova) -> None:
        """Configure all active viewers with the Nova instance."""
        for viewer in self._viewers:
            viewer.configure(nova)

    async def setup_viewers_after_preconditions(self) -> None:
        """Setup all active viewers after preconditions are satisfied."""
        for viewer in self._viewers:
            await viewer.setup_after_preconditions()

    def cleanup_viewers(self) -> None:
        """Clean up all active viewers."""
        for viewer in list(self._viewers):  # Copy to avoid modification during iteration
            viewer.cleanup()
        self._viewers.clear()

    async def log_planning_success(
        self,
        actions: Sequence[Action],
        trajectory: models.JointTrajectory,
        tcp: str,
        motion_group: MotionGroup,
    ) -> None:
        """Log successful planning results to all active viewers."""
        for viewer in self._viewers:
            try:
                await viewer.log_planning_success(actions, trajectory, tcp, motion_group)
            except Exception as e:
                # Don't fail planning if logging fails
                logger.warning("Failed to log planning results to viewer: %s", e)

    async def log_planning_failure(
        self, actions: Sequence[Action], error: Exception, tcp: str, motion_group: MotionGroup
    ) -> None:
        """Log planning failure to all active viewers."""
        for viewer in self._viewers:
            try:
                await viewer.log_planning_failure(actions, error, tcp, motion_group)
            except Exception as e:
                # Don't fail planning if logging fails
                logger.warning("Failed to log planning error to viewer: %s", e)

    @property
    def has_active_viewers(self) -> bool:
        """Check if there are any active viewers."""
        return len(self._viewers) > 0


# Global viewer manager instance
_viewer_manager = ViewerManager()


def get_viewer_manager() -> ViewerManager:
    """Get the global viewer manager instance."""
    return _viewer_manager


# Legacy functions for backward compatibility
def register_viewer(viewer: Viewer) -> None:
    """Register a viewer as active. (Legacy function for backward compatibility)"""
    _viewer_manager.register_viewer(viewer)


def configure_active_viewers(nova: Nova) -> None:
    """Configure all active viewers with the Nova instance. (Legacy function for backward compatibility)"""
    _viewer_manager.configure_viewers(nova)


async def setup_active_viewers_after_preconditions() -> None:
    """Setup all active viewers after preconditions are satisfied. (Legacy function for backward compatibility)"""
    await _viewer_manager.setup_viewers_after_preconditions()


def cleanup_active_viewers() -> None:
    """Clean up all active viewers. (Legacy function for backward compatibility)"""
    _viewer_manager.cleanup_viewers()


async def log_planning_results_to_viewers(
    actions: Sequence[Action],
    trajectory: models.JointTrajectory,
    tcp: str,
    motion_group: MotionGroup,
) -> None:
    """Log successful planning results to all active viewers. (Legacy function for backward compatibility)"""
    await _viewer_manager.log_planning_success(actions, trajectory, tcp, motion_group)


async def log_planning_error_to_viewers(
    actions: Sequence[Action], error: Exception, tcp: str, motion_group: MotionGroup
) -> None:
    """Log planning failure to all active viewers. (Legacy function for backward compatibility)"""
    await _viewer_manager.log_planning_failure(actions, error, tcp, motion_group)
