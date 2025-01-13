from typing import Iterable, Sized

import numpy as np
import pydantic
import wandelbots_api_client as wb
from scipy.spatial.transform import Rotation

from nova.types.vector3d import Vector3d


def _parse_args(*args):
    if len(args) == 1 and (
        isinstance(args[0], wb.models.Pose) or isinstance(args[0], wb.models.TcpPose)
    ):
        pos = args[0].position
        ori = args[0].orientation
        return {
            "position": Vector3d(x=pos.x, y=pos.y, z=pos.z),
            "orientation": Vector3d(x=ori.x, y=ori.y, z=ori.z),
        }
    if len(args) == 1 and isinstance(args[0], wb.models.Pose2):
        x1, y1, z1 = args[0].position
        x2, y2, z2 = args[0].orientation
        return {"position": Vector3d(x=x1, y=y1, z=z1), "orientation": Vector3d(x=x2, y=y2, z=z2)}
    if len(args) == 1 and isinstance(args[0], tuple):
        args = args[0]
    if len(args) == 6:
        x1, y1, z1, x2, y2, z2 = args
        return {"position": Vector3d(x=x1, y=y1, z=z1), "orientation": Vector3d(x=x2, y=y2, z=z2)}
    elif len(args) == 3:
        x1, y1, z1 = args
        return {"position": Vector3d(x=x1, y=y1, z=z1), "orientation": Vector3d(x=0, y=0, z=0)}
    else:
        raise ValueError("Invalid number of arguments for Pose")


class Pose(pydantic.BaseModel, Sized):
    """A pose (position and orientation)

    Example:
    >>> Pose(position=Vector3d(x=10, y=20, z=30), orientation=Vector3d(x=1, y=2, z=3))
    Pose(position=Vector3d(x=10, y=20, z=30), orientation=Vector3d(x=1, y=2, z=3))
    """

    position: Vector3d
    orientation: Vector3d

    def __init__(self, *args, **kwargs):
        """Parse a tuple into a dict

        Examples:
        >>> Pose((1, 2, 3, 4, 5, 6))
        Pose(position=Vector3d(x=1, y=2, z=3), orientation=Vector3d(x=4, y=5, z=6))
        >>> Pose((1, 2, 3))
        Pose(position=Vector3d(x=1, y=2, z=3), orientation=Vector3d(x=0, y=0, z=0))
        >>> Pose(wb.models.Pose(position=wb.models.Vector3d(x=1, y=2, z=3), orientation=wb.models.Vector3d(x=4, y=5, z=6)))
        Pose(position=Vector3d(x=1, y=2, z=3), orientation=Vector3d(x=4, y=5, z=6))
        >>> Pose(wb.models.TcpPose(position=wb.models.Vector3d(x=1, y=2, z=3), orientation=wb.models.Vector3d(x=4, y=5, z=6), coordinate_system=None, tcp='Flange'))
        Pose(position=Vector3d(x=1, y=2, z=3), orientation=Vector3d(x=4, y=5, z=6))
        >>> Pose(wb.models.Pose2(position=[1, 2, 3], orientation=[4, 5, 6]))
        Pose(position=Vector3d(x=1, y=2, z=3), orientation=Vector3d(x=4, y=5, z=6))
        """
        if args:
            values = _parse_args(*args)
            super().__init__(**values)
        else:
            super().__init__(**kwargs)

    def __str__(self):
        return str(round(self).to_tuple())

    def __round__(self, n=None):
        if n is not None:
            raise NotImplementedError("Setting precision is not supported yet")
        pos_and_rot_vector = self.to_tuple()
        return Pose(
            tuple(
                [round(a, 1) for a in pos_and_rot_vector[:3]]
                + [round(a, 3) for a in pos_and_rot_vector[3:]]
            )
        )

    def __len__(self):
        return 6

    def to_tuple(self) -> tuple[float, float, float, float, float, float]:
        """Return the pose as a tuple

        Example:
        >>> Pose(position=Vector3d(x=10, y=20, z=30), orientation=Vector3d(x=1, y=2, z=3)).to_tuple()
        (10, 20, 30, 1, 2, 3)
        """
        return self.position.to_tuple() + self.orientation.to_tuple()

    def __getitem__(self, item):
        return self.to_tuple()[item]

    def __matmul__(self, other):
        """
        Pose concatenation combines two poses into a single pose that represents the cumulative effect of both
        transformations applied sequentially.

        Args:
            other: can be a Pose, or an iterable with 6 elements

        Returns:
            Pose: the result of the concatenation

        Examples:
        >>> Pose((1, 2, 3, 0, 0, 0)) @ Pose((1, 2, 3, 0, 0, 0))
        Pose(position=Vector3d(x=2.0, y=4.0, z=6.0), orientation=Vector3d(x=0.0, y=0.0, z=0.0))
        >>> Pose((1, 2, 3, 0, 0, 0)) @ [1, 2, 3, 0, 0, 0]
        Pose(position=Vector3d(x=2.0, y=4.0, z=6.0), orientation=Vector3d(x=0.0, y=0.0, z=0.0))
        >>> Pose((1, 2, 3, 0, 0, 0)) @ (1, 2, 3, 0, 0, 0)
        Pose(position=Vector3d(x=2.0, y=4.0, z=6.0), orientation=Vector3d(x=0.0, y=0.0, z=0.0))
        >>> def as_iterator(data):
        ...     for d in data:
        ...         yield d
        >>> Pose((1, 2, 3, 0, 0, 0)) @ as_iterator([1, 2, 3, 0, 0, 0])
        Pose(position=Vector3d(x=2.0, y=4.0, z=6.0), orientation=Vector3d(x=0.0, y=0.0, z=0.0))
        """
        if isinstance(other, Pose):
            transformed_matrix = np.dot(self.matrix, other.matrix)
            return self._matrix_to_pose(transformed_matrix)
        elif isinstance(other, Iterable):
            seq = tuple(other)
            return self.__matmul__(Pose(seq))

        else:
            raise ValueError(f"Cannot multiply Pose with {type(other)}")

    def transform(self, other) -> "Pose":
        return self @ other

    def _to_wb_pose(self) -> wb.models.Pose:
        """Convert to wandelbots_api_client Pose

        Examples:
        >>> Pose(position=Vector3d(x=10, y=20, z=30), orientation=Vector3d(x=1, y=2, z=3))._to_wb_pose()
        Pose(position=Vector3d(x=10, y=20, z=30), orientation=Vector3d(x=1, y=2, z=3), coordinate_system=None)
        """
        return wb.models.Pose(
            position=wb.models.Vector3d(**self.position.model_dump()),
            orientation=wb.models.Vector3d(**self.orientation.model_dump()),
        )

    def _to_wb_pose2(self) -> wb.models.Pose2:
        """Convert to wandelbots_api_client Pose

        Examples:
        >>> Pose(position=Vector3d(x=10, y=20, z=30), orientation=Vector3d(x=1, y=2, z=3))._to_wb_pose2()
        Pose2(position=[10, 20, 30], orientation=[1, 2, 3])
        """
        return wb.models.Pose2(
            position=list(self.position.to_tuple()), orientation=list(self.orientation.to_tuple())
        )

    @pydantic.model_serializer
    def serialize_model(self):
        """
        Examples:
        >>> Pose(position=Vector3d(x=10, y=20, z=30), orientation=Vector3d(x=1, y=2, z=3)).model_dump()
        {'position': [10, 20, 30], 'orientation': [1, 2, 3]}
        """
        return self._to_wb_pose2().model_dump()

    def _to_homogenous_transformation_matrix(self):
        """Converts the pose (position and rotation vector) to a 4x4 homogeneous transformation matrix."""
        rotation_vec = [self.orientation.x, self.orientation.y, self.orientation.z]
        rotation_matrix = Rotation.from_rotvec(rotation_vec).as_matrix()
        mat = np.eye(4)
        mat[:3, :3] = rotation_matrix
        mat[:3, 3] = [self.position.x, self.position.y, self.position.z]
        return mat

    def _matrix_to_pose(self, matrix: np.ndarray) -> "Pose":
        """Converts a homogeneous transformation matrix to a Pose."""
        rotation_matrix = matrix[:3, :3]
        position = matrix[:3, 3]
        rotation_vec = Rotation.from_matrix(rotation_matrix).as_rotvec()
        return Pose(
            (
                position[0],
                position[1],
                position[2],
                rotation_vec[0],
                rotation_vec[1],
                rotation_vec[2],
            )
        )

    def orientation_to_quaternion(self):
        values = np.asarray(self.orientation)
        half_angle = np.linalg.norm(values) / 2
        return np.concatenate([np.cos(half_angle)[None], values * np.sinc(half_angle / np.pi) / 2])

    @property
    def matrix(self) -> np.ndarray:
        """Returns the homogeneous transformation matrix."""
        return self._to_homogenous_transformation_matrix()
