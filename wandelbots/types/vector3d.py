from wandelbots_api_client.models import Vector3d as Vector3dBase
from typing import Any


class Vector3d(Vector3dBase):
    """A vector class

    Example:
    >>> Vector3d(x=10, y=20, z=30)
    Vector3d(x=10.0, y=20.0, z=30.0)
    """

    x: float
    y: float
    z: float

    def __add__(self, other: Any):
        raise TypeError()

    def __radd__(self, other: Any):
        raise TypeError()

    def __mul__(self, other: Any) -> "Vector3d":
        if not isinstance(other, (float, int)):
            return NotImplemented
        return type(self)(x=other * self.x, y=other * self.y, z=other * self.z)

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

        Example:
        >>> Vector3d.from_tuple((10, 20, 30))
        Vector3d(x=10.0, y=20.0, z=30.0)
        """
        return cls(x=value[0], y=value[1], z=value[2])

    def to_tuple(self) -> tuple[float, float, float]:
        """Return the vector as a tuple

        Example:
        >>> Vector3d(x=10, y=20, z=30).to_tuple()
        (10.0, 20.0, 30.0)
        """
        return self.x, self.y, self.z
