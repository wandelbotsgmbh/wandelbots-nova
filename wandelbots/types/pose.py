from wandelbots_api_client.models import Pose as PoseBase
from wandelbots.types.vector3d import Vector3d
from typing import Any
import numpy as np


class Position(Vector3d):
    """A position

    Example:
    >>> Position(x=10, y=20, z=30)
    Position(x=10.0, y=20.0, z=30.0)
    """

    def __add__(self, other: Any) -> "Position":
        """Add two positions

        Example:
        >>> a = Position(x=10, y=20, z=30)
        >>> b = Position(x=1, y=1, z=1)
        >>> a + b == Position(x=11, y=21, z=31)
        True
        """
        if not isinstance(other, Position):
            return NotImplemented
        return Position(x=self.x + other.x, y=self.y + other.y, z=self.z + other.z)

    def __sub__(self, other: Any) -> "Position":
        """Subtract two positions

        Example:
        >>> a = Position(x=10, y=20, z=30)
        >>> b = Position(x=1, y=1, z=1)
        >>> a - b == Position(x=9, y=19, z=29)
        True
        """
        if not isinstance(other, Position):
            return NotImplemented
        return Position(x=self.x - other.x, y=self.y - other.y, z=self.z - other.z)


class Orientation(Vector3d):
    def to_quaternion(self):
        values = np.asarray(self)
        half_angle = np.linalg.norm(values) / 2
        return np.concatenate([np.cos(half_angle)[None], values * np.sinc(half_angle / np.pi) / 2])


class Pose(PoseBase):
    """A pose (position and orientation)

    Example:
    >>> Pose(position=Position(x=10, y=20, z=30), orientation=Orientation(x=1, y=2, z=3))
    Pose(position=Position(x=10.0, y=20.0, z=30.0), orientation=Orientation(x=1.0, y=2.0, z=3.0), coordinate_system=None)
    """

    position: Position
    orientation: Orientation

    def __str__(self):
        return str(round(self).to_tuple())

    def __round__(self, n=None):
        if n is not None:
            raise NotImplementedError("Setting precision is not supported yet")
        pos_and_rot_vector = self.to_tuple()
        return Pose.from_tuple(
            tuple([round(a, 1) for a in pos_and_rot_vector[:3]] + [round(a, 3) for a in pos_and_rot_vector[3:]])
        )

    def to_tuple(self) -> tuple[float, float, float, float, float, float]:
        """Return the pose as a tuple

        Example:
        >>> Pose(position=Position(x=1, y=2, z=3), orientation=Orientation(x=4, y=5, z=6)).to_tuple()
        (1.0, 2.0, 3.0, 4.0, 5.0, 6.0)
        """
        return self.position.to_tuple() + self.orientation.to_tuple()

    @classmethod
    def from_tuple(cls, value: tuple[float, float, float, float, float, float]):
        """Create a new Pose from tuple

        Args:
            value: tuple with values (x, y, z, rx, ry, rz)

        Returns: the new Pose

        Examples:
        >>> Pose.from_tuple((1, 2, 3, 4, 5, 6))
        Pose(position=Position(x=1.0, y=2.0, z=3.0), orientation=Orientation(x=4.0, y=5.0, z=6.0), coordinate_system=None)
        """
        return cls(
            position=Position(x=value[0], y=value[1], z=value[2]),
            orientation=Orientation(x=value[3], y=value[4], z=value[5]),
        )

    def __getitem__(self, item):
        return self.to_tuple()[item]
