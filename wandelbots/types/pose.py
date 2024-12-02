from wandelbots_api_client.models import Pose as PoseBase
from wandelbots.types.vector3d import Vector3d
from typing import Any
import numpy as np

from scipy.spatial.transform import Rotation as R

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


    def __matmul__(self, other):
        if isinstance(other, Pose):
            transformed_matrix = np.dot(self.matrix, other.matrix)
            return self._matrix_to_pose(transformed_matrix)
        elif isinstance(other, np.ndarray):
            assert other.shape == (4, 4)
            transformed_matrix = np.dot(self.matrix, other)
            return self._matrix_to_pose(transformed_matrix)
        else:
            raise ValueError(f"Cannot multiply Pose with {type(other)}")


    def _to_homogenous_transformation_matrix(self):
        """Converts the pose (position and rotation vector) to a 4x4 homogeneous transformation matrix."""
        rotation_vec = [self.orientation.x, self.orientation.y, self.orientation.z]
        rotation_matrix = R.from_rotvec(rotation_vec).as_matrix()
        mat = np.eye(4)
        mat[:3, :3] = rotation_matrix
        mat[:3, 3] = [self.position.x, self.position.y, self.position.z]
        return mat



    def _matrix_to_pose(self, matrix: np.ndarray) -> "Pose":
        """Converts a homogeneous transformation matrix to a Pose."""
        rotation_matrix = matrix[:3, :3]
        position = matrix[:3, 3]
        rotation_vec = R.from_matrix(rotation_matrix).as_rotvec()
        return Pose.from_tuple(
            (position[0], position[1], position[2], rotation_vec[0], rotation_vec[1], rotation_vec[2])
        )


    @property
    def matrix(self) -> np.ndarray:
        """Returns the homogeneous transformation matrix."""
        return self._to_homogenous_transformation_matrix()