"""Abstract base class for Nova program viewers."""

from __future__ import annotations

from abc import ABC, abstractmethod
from typing import TYPE_CHECKING, Sequence

if TYPE_CHECKING:
    from nova.actions import Action
    from nova.api import models
    from nova.core.motion_group import MotionGroup
    from nova.core.nova import Nova


class Viewer(ABC):
    """Abstract base class for Nova program viewers."""

    @abstractmethod
    def configure(self, nova: Nova) -> None:
        """Configure the viewer for program execution."""
        pass

    async def setup_after_preconditions(self) -> None:
        """Setup viewer components after preconditions are satisfied.

        Override this method in subclasses that need to wait for preconditions
        like active controllers before setting up visualization components.
        """
        pass

    @abstractmethod
    def cleanup(self) -> None:
        """Clean up the viewer after program execution."""
        pass

    async def log_planning_success(
        self,
        actions: Sequence[Action],
        trajectory: models.JointTrajectory,
        tcp: str,
        motion_group: MotionGroup,
    ) -> None:
        """Log successful planning results.

        Args:
            actions: List of actions that were planned
            trajectory: The resulting trajectory
            tcp: TCP used for planning
            motion_group: The motion group used for planning
        """
        pass

    async def log_planning_failure(
        self, actions: Sequence[Action], error: Exception, tcp: str, motion_group: MotionGroup
    ) -> None:
        """Log planning failure results.

        Args:
            actions: List of actions that failed to plan
            error: The planning error that occurred
            tcp: TCP used for planning
            motion_group: The motion group used for planning
        """
        pass
