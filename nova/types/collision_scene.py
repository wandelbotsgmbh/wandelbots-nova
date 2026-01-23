# TODO Why is this in nova.types? It offers no type, only a utility function.
import warnings

import wandelbots_api_client as wb


def compare_collision_scenes(
    scene1: wb.models.CollisionScene, scene2: wb.models.CollisionScene
) -> bool:
    """Compare two collision scenes for equality.

    Args:
        scene1: First collision scene to compare.
        scene2: Second collision scene to compare.

    Returns:
        True if the scenes are equal, False otherwise.
    """
    if scene1.colliders != scene2.colliders:
        return False

    # Compare motion groups
    if scene1.motion_groups != scene2.motion_groups:
        return False

    return True


def compare_collition_scenes(
    scene1: wb.models.CollisionScene, scene2: wb.models.CollisionScene
) -> bool:
    """Deprecated: Use compare_collision_scenes instead (typo fix)."""
    warnings.warn(
        "compare_collition_scenes is deprecated due to typo, use compare_collision_scenes instead",
        DeprecationWarning,
        stacklevel=2,
    )
    return compare_collision_scenes(scene1, scene2)
