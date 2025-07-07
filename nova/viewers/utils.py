"""Utility functions for Nova viewer implementations."""

from __future__ import annotations

from typing import TYPE_CHECKING, Sequence

if TYPE_CHECKING:
    from nova.actions import Action
    from nova.api import models


def extract_collision_scenes_from_actions(
    actions: Sequence[Action],
) -> dict[str, models.CollisionScene]:
    """Extract unique collision scenes from a list of actions.

    Args:
        actions: List of actions to extract collision scenes from

    Returns:
        Dictionary mapping collision scene IDs to CollisionScene objects
    """
    from nova.actions.motions import CollisionFreeMotion, Motion

    collision_scenes: dict[str, models.CollisionScene] = {}

    for i, action in enumerate(actions):
        # Check if action is a motion with collision_scene attribute
        if isinstance(action, (Motion, CollisionFreeMotion)) and action.collision_scene is not None:
            # Generate a deterministic ID based on action index and type
            scene_id = f"action_{i}_{type(action).__name__}_scene"
            collision_scenes[scene_id] = action.collision_scene

    return collision_scenes
