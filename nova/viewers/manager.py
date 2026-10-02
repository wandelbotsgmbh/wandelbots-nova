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
    """Coordinates legacy global viewers and execution-scoped program viewers."""

    def __init__(self) -> None:
        self._registered_viewers: WeakSet[Viewer] = WeakSet()
        self._scoped_viewers: contextvars.ContextVar[tuple[Viewer, ...]] = contextvars.ContextVar(
            f"scoped_viewers_{id(self)}", default=()
        )

    @property
    def active_viewers(self) -> tuple[Viewer, ...]:
        """Return global and execution-scoped viewers visible in the current context."""
        viewers = list(self._registered_viewers)
        for viewer in self._scoped_viewers.get():
            if not any(active is viewer for active in viewers):
                viewers.append(viewer)
        return tuple(viewers)

    def register_viewer(self, viewer: Viewer) -> None:
        """Register a legacy global viewer."""
        self._registered_viewers.add(viewer)

    def unregister_viewer(self, viewer: Viewer) -> None:
        """Unregister a global viewer without cleaning it up."""
        self._registered_viewers.discard(viewer)

    def activate_viewer(self, viewer: Viewer) -> None:
        """Activate a viewer only in the current execution context."""
        viewers = self._scoped_viewers.get()
        if not any(active is viewer for active in viewers):
            self._scoped_viewers.set((*viewers, viewer))

    def cleanup_viewer(self, viewer: Viewer) -> None:
        """Clean up and remove one viewer from global and scoped activation."""
        try:
            viewer.cleanup()
        finally:
            self.unregister_viewer(viewer)
            self._scoped_viewers.set(
                tuple(active for active in self._scoped_viewers.get() if active is not viewer)
            )

    def configure_viewers(self, nova: Nova) -> None:
        """Configure all active viewers with the Nova instance."""
        for viewer in self.active_viewers:
            viewer.configure(nova)

    async def setup_viewers_after_preconditions(self) -> None:
        """Setup all active viewers after preconditions are satisfied."""
        for viewer in self.active_viewers:
            await viewer.setup_after_preconditions()

    def cleanup_viewers(self) -> None:
        """Clean up all global and current-context viewers."""
        viewers = self.active_viewers
        try:
            for viewer in viewers:
                viewer.cleanup()
        finally:
            self._registered_viewers.clear()
            self._scoped_viewers.set(())

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
        """Log planning failure results to all active viewers."""
        for viewer in self.active_viewers:
            try:
                await viewer.log_planning_failure(
                    actions=actions, error=error, tcp=tcp, motion_group=motion_group
                )
            except Exception as e:
                # Don't fail planning if logging fails
                logger.warning("Failed to log planning error to viewer: %s", e)

    def get_viewer(self, viewer_type: type[ViewerT]) -> ViewerT | None:
        """Return the active viewer of the requested type, if present."""
        return next(
            (viewer for viewer in self.active_viewers if isinstance(viewer, viewer_type)), None
        )

    @property
    def has_active_viewers(self) -> bool:
        """Check whether the current context has any visible viewers."""
        return bool(self.active_viewers)


# Global manager. Legacy viewers remain global; program viewers are context-local.
_viewer_manager = ViewerManager()


def get_viewer_manager() -> ViewerManager:
    """Get the global viewer manager instance."""
    return _viewer_manager


# Legacy functions for backward compatibility
def register_viewer(viewer: Viewer) -> None:
    """Register a legacy global viewer."""
    _viewer_manager.register_viewer(viewer)


def configure_active_viewers(nova: Nova) -> None:
    """Configure all active viewers."""
    _viewer_manager.configure_viewers(nova)


async def setup_active_viewers_after_preconditions() -> None:
    """Set up all active viewers."""
    await _viewer_manager.setup_viewers_after_preconditions()


def cleanup_active_viewers() -> None:
    """Clean up all active viewers."""
    _viewer_manager.cleanup_viewers()


async def log_planning_results_to_viewers(
    actions: Sequence[Action],
    trajectory: models.JointTrajectory,
    tcp: str | None,
    motion_group: MotionGroup,
) -> None:
    """Log successful planning results to all active viewers."""
    await _viewer_manager.log_planning_success(actions, trajectory, tcp, motion_group)


async def log_planning_error_to_viewers(
    actions: Sequence[Action], error: Exception, tcp: str | None, motion_group: MotionGroup
) -> None:
    """Log planning failure to all active viewers."""
    await _viewer_manager.log_planning_failure(actions, error, tcp, motion_group)
