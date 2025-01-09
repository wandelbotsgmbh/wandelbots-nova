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

    def __neg__(self) -> "Vector3d":
        return type(self)(x=-self.x, y=-self.y, z=-self.z)

    def __add__(self, other: Any) -> "Vector3d":
        if isinstance(other, (float, int)):
            return type(self)(x=self.x + other, y=self.y + other, z=self.z + other)
        if isinstance(other, Vector3d):
            return type(self)(x=self.x + other.x, y=self.y + other.y, z=self.z + other.z)
        return NotImplemented

    def __radd__(self, other: Any) -> "Vector3d":
        return self.__add__(other)

    def __sub__(self, other: Any) -> "Vector3d":
        return self.__add__(-other)

    def __rsub__(self, other: Any) -> "Vector3d":
        if isinstance(other, (float, int)):
            return type(self)(x=other - self.x, y=other - self.y, z=other - self.z)
        if isinstance(other, Vector3d):
            return type(self)(x=other.x - self.x, y=other.y - self.y, z=other.z - self.z)
        return NotImplemented

    def __mul__(self, other: Any) -> "Vector3d":
        if isinstance(other, (float, int)):
            return type(self)(x=other * self.x, y=other * self.y, z=other * self.z)
        return NotImplemented

    def __rmul__(self, other: Any) -> "Vector3d":
        return self * other

    def __truediv__(self, other: Any) -> "Vector3d":
        if not isinstance(other, (float, int)):
            return NotImplemented
        return (1 / other) * self

    def __len__(self):
        return 3

    @classmethod
    def from_tuple(cls, value: tuple[float, float, float]) -> "Vector3d":
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

    def to_quaternion(self):
        """Interpret the object as a rotation vector and convert it to a quaternion"""
        values = np.asarray(self)
        half_angle = np.linalg.norm(values) / 2
        return np.concatenate([np.cos(half_angle)[None], values * np.sinc(half_angle / np.pi) / 2])

    @pydantic.model_serializer
    def serialize_model(self) -> wb.models.Vector3d:
        """
        Examples:
        >>> Vector3d.from_tuple((1, 2, 3)).model_dump()
        {'x': 1, 'y': 2, 'z': 3}
        """
        return wb.models.Vector3d(x=self.x, y=self.y, z=self.z)
