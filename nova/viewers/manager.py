"""Viewer manager for coordinating multiple viewers."""

from __future__ import annotations

import contextvars
import logging
from typing import TYPE_CHECKING, Sequence, TypeVar
from weakref import WeakSet

if TYPE_CHECKING:
    from nova.actions import Action
    from nova.api import models
    from nova.cell.motion_group import MotionGroup
    from nova.core.nova import Nova

from .base import Viewer

logger = logging.getLogger(__name__)

ViewerT = TypeVar("ViewerT", bound=Viewer)


class ViewerManager:
    """Manages the lifecycle and coordination of all active viewers."""

    def __init__(self) -> None:
        self._viewers: WeakSet[Viewer] = WeakSet()
        # Viewers a program activated for one run, visible only in that run's context.
        self._run_viewers: contextvars.ContextVar[tuple[Viewer, ...]] = contextvars.ContextVar(
            f"run_viewers_{id(self)}", default=()
        )

    @property
    def active_viewers(self) -> tuple[Viewer, ...]:
        """Registered viewers plus the ones activated for the current run."""
        viewers = list(self._viewers)
        viewers += [v for v in self._run_viewers.get() if not any(v is a for a in viewers)]
        return tuple(viewers)

    def register_viewer(self, viewer: Viewer) -> None:
        """Register a viewer as active."""
        self._viewers.add(viewer)

    def unregister_viewer(self, viewer: Viewer) -> None:
        """Remove a registered viewer without cleaning it up."""
        self._viewers.discard(viewer)

    def activate_viewer(self, viewer: Viewer) -> None:
        """Make a viewer active for the current run only."""
        viewers = self._run_viewers.get()
        if not any(viewer is v for v in viewers):
            self._run_viewers.set((*viewers, viewer))

    def cleanup_viewer(self, viewer: Viewer) -> None:
        """Clean up one viewer and deactivate it, leaving the others active."""
        try:
            viewer.cleanup()
        finally:
            self.unregister_viewer(viewer)
            self._run_viewers.set(tuple(v for v in self._run_viewers.get() if v is not viewer))

    def get_viewer(self, viewer_type: type[ViewerT]) -> ViewerT | None:
        """Return the active viewer of the given type, if any."""
        return next((v for v in self.active_viewers if isinstance(v, viewer_type)), None)

    def configure_viewers(self, nova: Nova) -> None:
        """Configure all active viewers with the Nova instance."""
        for viewer in self.active_viewers:
            viewer.configure(nova)

    async def setup_viewers_after_preconditions(self) -> None:
        """Setup all active viewers after preconditions are satisfied."""
        for viewer in self.active_viewers:
            await viewer.setup_after_preconditions()

    def cleanup_viewers(self) -> None:
        """Clean up all active viewers."""
        for viewer in self.active_viewers:
            viewer.cleanup()
        self._viewers.clear()
        self._run_viewers.set(())

    async def log_planning_success(
        self,
        actions: Sequence[Action],
        trajectory: models.JointTrajectory,
        tcp: str | None,
        motion_group: MotionGroup,
    ) -> None:
        """Log successful planning results to all active viewers."""
        for viewer in self.active_viewers:
            try:
                await viewer.log_planning_success(
                    actions=actions, trajectory=trajectory, tcp=tcp, motion_group=motion_group
                )
            except Exception as e:
                # Don't fail planning if logging fails
                logger.warning("Failed to log planning results to viewer: %s", e)

    async def log_planning_failure(
        self,
        actions: Sequence[Action],
        error: Exception,
        tcp: str | None,
        motion_group: MotionGroup,
    ) -> None:
        """Log planning failure to all active viewers."""
        for viewer in self.active_viewers:
            try:
                await viewer.log_planning_failure(
                    actions=actions, error=error, tcp=tcp, motion_group=motion_group
                )
            except Exception as e:
                # Don't fail planning if logging fails
                logger.warning("Failed to log planning error to viewer: %s", e)

    @property
    def has_active_viewers(self) -> bool:
        """Check if there are any active viewers."""
        return len(self.active_viewers) > 0


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
    tcp: str | None,
    motion_group: MotionGroup,
) -> None:
    """Log successful planning results to all active viewers. (Legacy function for backward compatibility)"""
    await _viewer_manager.log_planning_success(actions, trajectory, tcp, motion_group)


async def log_planning_error_to_viewers(
    actions: Sequence[Action], error: Exception, tcp: str | None, motion_group: MotionGroup
) -> None:
    """Log planning failure to all active viewers. (Legacy function for backward compatibility)"""
    await _viewer_manager.log_planning_failure(actions, error, tcp, motion_group)
