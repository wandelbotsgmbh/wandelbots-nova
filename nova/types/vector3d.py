from __future__ import annotations

from typing import Any

import numpy as np
import pydantic
import wandelbots_api_client as wb


class Vector3d(pydantic.BaseModel):
    """A vector 3d class
    Examples:
    >>> Vector3d(x=10, y=20, z=30)
    Vector3d(x=10, y=20, z=30)
    """

    x: float | int
    y: float | int
    z: float | int

    def __eq__(self, other):
        if not isinstance(other, Vector3d):
            return NotImplemented
        return self.x == other.x and self.y == other.y and self.z == other.z

    def __neg__(self) -> Vector3d:
        return type(self)(x=-self.x, y=-self.y, z=-self.z)

    def __add__(self, other: Any) -> Vector3d:
        if isinstance(other, (float, int)):
            return type(self)(x=self.x + other, y=self.y + other, z=self.z + other)
        if isinstance(other, Vector3d):
            return type(self)(x=self.x + other.x, y=self.y + other.y, z=self.z + other.z)
        return NotImplemented

    def __radd__(self, other: Any) -> Vector3d:
        return self.__add__(other)

    def __sub__(self, other: Any) -> Vector3d:
        return self.__add__(-other)

    def __rsub__(self, other: Any) -> Vector3d:
        if isinstance(other, (float, int)):
            return type(self)(x=other - self.x, y=other - self.y, z=other - self.z)
        if isinstance(other, Vector3d):
            return type(self)(x=other.x - self.x, y=other.y - self.y, z=other.z - self.z)
        return NotImplemented

    def __mul__(self, other: Any) -> Vector3d:
        if isinstance(other, (float, int)):
            return type(self)(x=other * self.x, y=other * self.y, z=other * self.z)
        return NotImplemented

    def __rmul__(self, other: Any) -> Vector3d:
        return self * other

    def __truediv__(self, other: Any) -> Vector3d:
        if not isinstance(other, (float, int)):
            return NotImplemented
        return (1 / other) * self

    def __len__(self):
        return 3

    def __iter__(self):
        """Iterate over the vector

        Examples:
        >>> v = Vector3d(x=1, y=2, z=3)
        >>> list(v)
        [1, 2, 3]
        >>> tuple(v)
        (1, 2, 3)
        """
        return iter(self.to_tuple())

    def __getitem__(self, item):
        return self.to_tuple()[item]

    @classmethod
    def from_tuple(cls, value: tuple[float, float, float]) -> Vector3d:
        """Create a new Vector3d from tuple

        Examples:
        >>> Vector3d.from_tuple((10, 20, 30))
        Vector3d(x=10, y=20, z=30)
        >>> Vector3d.from_tuple((10.0, 20.5, 30.2))
        Vector3d(x=10.0, y=20.5, z=30.2)
        """
        return cls(x=value[0], y=value[1], z=value[2])

    def to_tuple(self) -> tuple[float, float, float]:
        """Return the vector as a tuple

        Examples:
        >>> Vector3d(x=10, y=20, z=30).to_tuple()
        (10, 20, 30)
        >>> Vector3d(x=10.0, y=20.5, z=30.2).to_tuple()
        (10.0, 20.5, 30.2)
        """
        return self.x, self.y, self.z

    def __array__(self, dtype=None):
        """Allows numpy to automatically convert Vector3d to a numeric array.

        Examples:
        >>> v1 = Vector3d(x=1.0, y=2.0, z=3.0)
        >>> v2 = Vector3d(x=1.001, y=2.0, z=3.0)
        >>> np.isclose(v1, v2, atol=1e-3)
        array([ True,  True,  True])
        """
        return np.array(self.to_tuple(), dtype=dtype)

    def to_quaternion(self):
        """Interpret the object as a rotation vector and convert it to a quaternion"""
        values = np.asarray(self)
        half_angle = np.linalg.norm(values) / 2
        return np.concatenate([np.cos(half_angle)[None], values * np.sinc(half_angle / np.pi) / 2])

    def to_api_vector3d(self) -> wb.models.Vector3d:
        return wb.models.Vector3d(x=self.x, y=self.y, z=self.z)

    @pydantic.model_serializer
    def serialize_model(self) -> wb.models.Vector3d:
        """
        Examples:
        >>> Vector3d.from_tuple((1, 2, 3)).model_dump()
        {'x': 1, 'y': 2, 'z': 3}
        """
        return self.to_api_vector3d()
