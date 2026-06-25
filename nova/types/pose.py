from __future__ import annotations

from typing import Iterable, Sized

import numpy as np
import pydantic
from scipy.spatial.transform import Rotation

from nova import api
from nova.types.vector3d import Vector3d

_POSE_EQUALITY_PRECISION = 6


def _parse_args(*args):
    """Parse the arguments and return a dictionary that pydanctic can validate"""
    if args == (None,):
        return {
            "position": Vector3d(x=0.0, y=0.0, z=0.0),
            "orientation": Vector3d(x=0.0, y=0.0, z=0.0),
        }
    if len(args) == 1 and isinstance(args[0], api.models.Pose):
        pos = args[0].position
        ori = args[0].orientation
        if pos is None:
            pos = [0.0, 0.0, 0.0]
        if ori is None:
            ori = [0.0, 0.0, 0.0]
        return {
            "position": Vector3d(x=pos[0], y=pos[1], z=pos[2]),
            "orientation": Vector3d(x=ori[0], y=ori[1], z=ori[2]),
        }
    if len(args) == 1 and isinstance(args[0], tuple):
        args = args[0]
    if len(args) == 6:
        x1, y1, z1, x2, y2, z2 = args
        return {"position": Vector3d(x=x1, y=y1, z=z1), "orientation": Vector3d(x=x2, y=y2, z=z2)}
    if len(args) == 3:
        x1, y1, z1 = args
        return {"position": Vector3d(x=x1, y=y1, z=z1), "orientation": Vector3d(x=0, y=0, z=0)}
    raise ValueError("Invalid number of arguments for Pose")


class Pose(pydantic.BaseModel, Sized):
    """A pose (position and orientation)

    Example:
    >>> Pose(position=Vector3d(x=10, y=20, z=30), orientation=Vector3d(x=1, y=2, z=3))
    Pose(position=Vector3d(x=10, y=20, z=30), orientation=Vector3d(x=1, y=2, z=3), kinematic_configuration=None)
    """

    position: Vector3d
    orientation: Vector3d
    kinematic_configuration: api.models.KinematicConfiguration | None = None

    def __init__(self, *args, **kwargs):
        """Parse a tuple into a dict

        Examples:
        >>> Pose((1, 2, 3, 4, 5, 6))
        Pose(position=Vector3d(x=1, y=2, z=3), orientation=Vector3d(x=4, y=5, z=6), kinematic_configuration=None)
        >>> Pose((1, 2, 3))
        Pose(position=Vector3d(x=1, y=2, z=3), orientation=Vector3d(x=0, y=0, z=0), kinematic_configuration=None)
        >>> Pose(api.models.Pose(position=api.models.Vector3d([1, 2, 3]), orientation=api.models.Vector3d([4, 5, 6])))
        Pose(position=Vector3d(x=1.0, y=2.0, z=3.0), orientation=Vector3d(x=4.0, y=5.0, z=6.0), kinematic_configuration=None)
        >>> Pose(api.models.Pose(position=api.models.Vector3d([1, 2, 3]), orientation=api.models.RotationVector([4, 5, 6])))
        Pose(position=Vector3d(x=1.0, y=2.0, z=3.0), orientation=Vector3d(x=4.0, y=5.0, z=6.0), kinematic_configuration=None)
        >>> pose = Pose((1, 2, 3, 4, 5, 6))
        >>> new_pose = Pose.model_validate(pose.model_dump())
        >>> pose == new_pose
        True
        >>> Pose(api.models.Pose(position=None, orientation=None))
        Pose(position=Vector3d(x=0.0, y=0.0, z=0.0), orientation=Vector3d(x=0.0, y=0.0, z=0.0), kinematic_configuration=None)
        >>> Pose(api.models.Pose(position=api.models.Vector3d([1, 2, 3]), orientation=None))
        Pose(position=Vector3d(x=1.0, y=2.0, z=3.0), orientation=Vector3d(x=0.0, y=0.0, z=0.0), kinematic_configuration=None)
        >>> Pose(api.models.Pose(position=None, orientation=api.models.RotationVector([4, 5, 6])))
        Pose(position=Vector3d(x=0.0, y=0.0, z=0.0), orientation=Vector3d(x=4.0, y=5.0, z=6.0), kinematic_configuration=None)
        >>> Pose(None)
        Pose(position=Vector3d(x=0.0, y=0.0, z=0.0), orientation=Vector3d(x=0.0, y=0.0, z=0.0), kinematic_configuration=None)
        >>> kc = api.models.KinematicConfiguration(kinematic_branch=api.models.KinematicBranch(shoulder_branch='FRONT', elbow_branch='UP', wrist_branch='NO_FLIP'))
        >>> Pose((1, 2, 3, 4, 5, 6), kinematic_configuration=kc).kinematic_configuration == kc
        True
        >>> lr = api.models.LimitRange(lower_limit=-3.14, upper_limit=3.14)
        >>> ar = [api.models.AxisRange(axis=0, range=lr), api.models.AxisRange(axis=5, range=lr)]
        >>> kb = api.models.KinematicBranch(shoulder_branch='FRONT', elbow_branch='UP', wrist_branch='NO_FLIP')
        >>> kc2 = api.models.KinematicConfiguration(kinematic_branch=kb, axis_ranges=ar)
        >>> Pose((1, 2, 3, 4, 5, 6), kinematic_configuration=kc2).kinematic_configuration == kc2
        True
        """
        # >>> Pose(api.models.TcpOffset(name='Flange', pose=api.models.Pose(position=api.models.Vector3d([1, 2, 3]), orientation=api.models.Vector3d([4, 5, 6]))))
        # Pose(position=Vector3d(x=1, y=2, z=3), orientation=Vector3d(x=4, y=5, z=6))
        # Preserve kinematic_configuration from kwargs when positional args are parsed by _parse_args.
        # _parse_args only returns position/orientation, so we inject it back before validation.
        kinematic_configuration = kwargs.pop("kinematic_configuration", None)
        if args:
            values = _parse_args(*args)
            values["kinematic_configuration"] = kinematic_configuration
            super().__init__(**values)
        else:
            kwargs.setdefault("kinematic_configuration", kinematic_configuration)
            super().__init__(**kwargs)

    def __str__(self):
        return str(round(self).to_tuple())

    def __eq__(self, other):
        """Check equality of two poses.

        Note: Two poses are only equal if position, orientation AND kinematic_configuration
        all match.
        """
        if not isinstance(other, Pose):
            return NotImplemented

        first_val = tuple(round(val, _POSE_EQUALITY_PRECISION) for val in self.to_tuple())
        second_val = tuple(round(val, _POSE_EQUALITY_PRECISION) for val in other.to_tuple())
        return (
            first_val == second_val
            and self.kinematic_configuration == other.kinematic_configuration
        )

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

    def __iter__(self):
        """Iterate over the pose

        Examples:
        >>> p = Pose((1, 2, 3, 0, 0, 0))
        >>> list(p)
        [1, 2, 3, 0, 0, 0]
        >>> tuple(p)
        (1, 2, 3, 0, 0, 0)
        """
        return iter(self.to_tuple())

    def __invert__(self) -> Pose:
        """
        Return the inverse of this pose.
        In terms of 4x4 homogeneous matrices, this is T^-1 where T = R|p
                                                                     0|1
        i.e. T^-1 = R^T | -R^T p
                     0  |   1

        Returns:
            Pose: the inverse of the current pose

        Examples:
        >>> p = Pose((1, 2, 3, 0, 0, np.pi/2))  # rotate 90° about Z
        >>> inv_p = ~p
        >>> identity_approx = p @ inv_p
        >>> np.allclose(identity_approx.position.to_tuple(), (0, 0, 0), atol=1e-7)
        True
        """
        # Invert the homogeneous transformation matrix
        inv_matrix = np.linalg.inv(self.matrix)
        # Convert back to a Pose
        return self._matrix_to_pose(inv_matrix)

    def __getitem__(self, item):
        return self.to_tuple()[item]

    def to_tuple(self) -> tuple[float, float, float, float, float, float]:
        """Return the pose as a tuple

        Example:
        >>> Pose(position=Vector3d(x=10, y=20, z=30), orientation=Vector3d(x=1, y=2, z=3)).to_tuple()
        (10, 20, 30, 1, 2, 3)
        """
        return self.position.to_tuple() + self.orientation.to_tuple()  # ty: ignore[invalid-return-type]

    def to_api_model(self) -> api.models.Pose:
        """Convert to wandelbots_api_client Pose

        Note: kinematic_configuration is not included in the result since api.models.Pose
        does not support it. It is handled separately by motion actions (e.g. CartesianPTP).

        Examples:
        >>> Pose(position=Vector3d(x=10, y=20, z=30), orientation=Vector3d(x=1, y=2, z=3)).to_api_model()
        Pose(position=Vector3d(root=[10.0, 20.0, 30.0]), orientation=RotationVector(root=[1.0, 2.0, 3.0]))
        """
        return api.models.Pose(
            position=api.models.Vector3d([self.position.x, self.position.y, self.position.z]),
            orientation=api.models.RotationVector(
                [self.orientation.x, self.orientation.y, self.orientation.z]
            ),
        )

    def __matmul__(self, other):
        """
        Pose concatenation combines two poses into a single pose that represents the cumulative effect of both
        transformations applied sequentially.

        Note: kinematic_configuration is NOT propagated — the result always has
        kinematic_configuration=None.

        Args:
            other: can be a Pose, or an iterable with 6 elements

        Returns:
            Pose: the result of the concatenation

        Examples:
        >>> Pose((1, 2, 3, 0, 0, 0)) @ Pose((1, 2, 3, 0, 0, 0))
        Pose(position=Vector3d(x=2.0, y=4.0, z=6.0), orientation=Vector3d(x=0.0, y=0.0, z=0.0), kinematic_configuration=None)
        >>> Pose((1, 2, 3, 0, 0, 0)) @ [1, 2, 3, 0, 0, 0]
        Pose(position=Vector3d(x=2.0, y=4.0, z=6.0), orientation=Vector3d(x=0.0, y=0.0, z=0.0), kinematic_configuration=None)
        >>> Pose((1, 2, 3, 0, 0, 0)) @ (1, 2, 3, 0, 0, 0)
        Pose(position=Vector3d(x=2.0, y=4.0, z=6.0), orientation=Vector3d(x=0.0, y=0.0, z=0.0), kinematic_configuration=None)
        >>> def as_iterator(data):
        ...     for d in data:
        ...         yield d
        >>> Pose((1, 2, 3, 0, 0, 0)) @ as_iterator([1, 2, 3, 0, 0, 0])
        Pose(position=Vector3d(x=2.0, y=4.0, z=6.0), orientation=Vector3d(x=0.0, y=0.0, z=0.0), kinematic_configuration=None)
        >>> Pose((1, 2, 3, 0, 0, 0)) @ Vector3d.from_tuple((1, 2, 3))
        Pose(position=Vector3d(x=2.0, y=4.0, z=6.0), orientation=Vector3d(x=0.0, y=0.0, z=0.0), kinematic_configuration=None)
        """
        if isinstance(other, Pose):
            transformed_matrix = np.dot(self.matrix, other.matrix)
            return self._matrix_to_pose(transformed_matrix)
        if isinstance(other, Iterable):
            seq = tuple(other)
            return self.__matmul__(Pose(seq))

        raise ValueError(f"Cannot multiply Pose with {type(other)}")

    def __array__(self, dtype=None):
        """Convert Pose to a 6-element numpy array: [pos.x, pos.y, pos.z, ori.x, ori.y, ori.z].

        Examples:
        >>> p1 = Pose((1.0, 2.0, 3.0, 4.0, 5.0, 6.0))
        >>> p2 = Pose((1.001, 2.0, 3.0, 3.9995, 5.0, 6.0))
        >>> np.isclose(p1, p2, atol=1e-3)
        array([ True,  True,  True,  True,  True,  True])
        """
        # The `to_tuple()` method already returns (x, y, z, rx, ry, rz)
        return np.array(self.to_tuple(), dtype=dtype)

    def transform(self, other) -> Pose:
        return self @ other

    @pydantic.model_serializer
    def serialize_model(self):
        """
        Serializes the pose including kinematic_configuration if set.

        Examples:
        >>> Pose(position=Vector3d(x=10, y=20, z=30), orientation=Vector3d(x=1, y=2, z=3)).model_dump()
        {'position': [10.0, 20.0, 30.0], 'orientation': [1.0, 2.0, 3.0]}

        >>> from nova import api
        >>> kc = api.models.KinematicConfiguration(kinematic_branch=api.models.KinematicBranch(shoulder_branch='FRONT', elbow_branch='UP', wrist_branch='NO_FLIP'))
        >>> p = Pose((1, 2, 3, 4, 5, 6), kinematic_configuration=kc)
        >>> d = p.model_dump()
        >>> 'kinematic_configuration' in d
        True
        >>> Pose.model_validate(d) == p
        True
        """
        result = self.to_api_model().model_dump()
        if self.kinematic_configuration is not None:
            result["kinematic_configuration"] = self.kinematic_configuration.model_dump()
        return result

    @pydantic.model_validator(mode="before")
    @classmethod
    def model_validator(cls, data):
        """Transform the data that is passed into model validator to match what we return in the model_dump.

        Handles optional kinematic_configuration for roundtrip serialization.
        """
        if not isinstance(data, dict):
            raise ValueError("model_validator only accepts dicts")
        pos = data["position"]
        ori = data["orientation"]
        result: dict[str, object] = {
            "position": Vector3d(x=pos[0], y=pos[1], z=pos[2]),
            "orientation": Vector3d(x=ori[0], y=ori[1], z=ori[2]),
        }
        kc = data.get("kinematic_configuration")
        if kc is not None:
            result["kinematic_configuration"] = api.models.KinematicConfiguration.model_validate(kc)
        return result

    def _to_homogenous_transformation_matrix(self):
        """Converts the pose (position and rotation vector) to a 4x4 homogeneous transformation matrix."""
        rotation_vec = [self.orientation.x, self.orientation.y, self.orientation.z]
        rotation_matrix = Rotation.from_rotvec(rotation_vec).as_matrix()
        mat = np.eye(4)
        mat[:3, :3] = rotation_matrix
        mat[:3, 3] = [self.position.x, self.position.y, self.position.z]
        return mat

    def _matrix_to_pose(self, matrix: np.ndarray) -> Pose:
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

    @classmethod
    def from_euler(
        cls,
        position: Vector3d | tuple | list,
        euler_angles: tuple | list,
        convention: str = "xyz",
        degrees: bool = False,
    ) -> Pose:
        """
        Create a Pose from a position and Euler angles.

        Args:
            position: The position as a Vector3d, tuple, or list of 3 floats.
            euler_angles: The Euler angles (e.g., roll, pitch, yaw) as a tuple or list of 3 floats.
            convention: The Euler angle convention (e.g., 'xyz', 'zyx'). Defaults to 'xyz'.
            degrees: Whether the provided Euler angles are in degrees. Defaults to False (radians).

        Returns:
            A new Pose object.

        Example:
        >>> pose = Pose.from_euler(position=(1, 2, 3), euler_angles=(0, 0, 90), degrees=True)
        >>> np.allclose(pose.orientation.to_tuple(), (0, 0, np.pi/2))
        True
        """
        if not isinstance(position, Vector3d):
            position = Vector3d.from_tuple(tuple(position))

        # convert eulerangles to rotation vector
        rotation = Rotation.from_euler(convention, euler_angles, degrees=degrees)
        rotation_vector = rotation.as_rotvec()

        orientation = Vector3d.from_tuple(tuple(rotation_vector))

        return cls(position=position, orientation=orientation)

    def orientation_to_quaternion(self):
        values = np.asarray(self.orientation)
        half_angle = np.linalg.norm(values) / 2
        return np.concatenate([np.cos(half_angle)[None], values * np.sinc(half_angle / np.pi) / 2])

    @property
    def matrix(self) -> np.ndarray:
        """Returns the homogeneous transformation matrix."""
        return self._to_homogenous_transformation_matrix()
