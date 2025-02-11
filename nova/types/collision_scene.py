import pydantic

from nova.api import models


class CollisionScene(pydantic.BaseModel):
    """A wrapper around the Wandelbots collision scene model with additional functionality"""

    collision_scene: models.CollisionScene

    def are_equal(self, other: "CollisionScene") -> bool:
        """Compare two collision scenes for equality.

        Args:
            other: Another collision scene to compare with

        Returns:
            bool: True if scenes are equal, False otherwise
        """
        # Compare colliders
        if self.collision_scene.colliders != other.collision_scene.colliders:
            return False

        # Compare motion groups
        if self.collision_scene.motion_groups != other.collision_scene.motion_groups:
            return False

        return True

    def __eq__(self, other: object) -> bool:
        """Implement equality operator.

        Args:
            other: Object to compare with

        Returns:
            bool: True if objects are equal, False otherwise
        """
        if not isinstance(other, CollisionScene):
            return NotImplemented
        return self.are_equal(other)

    model_config = pydantic.ConfigDict(arbitrary_types_allowed=True)


__all__ = ["CollisionScene"]
