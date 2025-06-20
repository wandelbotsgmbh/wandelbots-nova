from functools import singledispatch
from typing import Any, TypeVar

from nova.types import Pose, Vector3d
from wandelscript.metamodel import register_builtin_func

# TODO naming. updated, updating
# TODO make a more generic version of the function
# TODO what about the type safety of the val
T = TypeVar("T")


@register_builtin_func()
@singledispatch
def assoc(_: T, key: int, val: Any) -> T:
    """Update the value at the given key in a sequence.

    Args:
        _: The sequence.
        key: The key to update.
        val: The new value.

    Return:
        The updated sequence.

    Raises:
        NotImplementedError: If the sequence type is not supported.

    Examples:
    >>> assoc((1, 2, 3), 1, 4)
    (1, 4, 3)
    """
    raise NotImplementedError(f"assoc is not defined for {type(_)}.")


@assoc.register
def _(tup: tuple, key: int, val: Any) -> tuple:
    tmp = list(tup)
    tmp[key] = val
    return tuple(tmp)


@assoc.register
def _(vec: Vector3d, key: int, val: float) -> Vector3d:
    tmp = list(vec)
    tmp[key] = val
    return Vector3d(x=tmp[0], y=tmp[1], z=tmp[2])


@assoc.register
def _(pose: Pose, key: int, val: float) -> Pose:
    tmp = list(pose.to_tuple())
    tmp[key] = val
    return Pose(tuple(tmp))


# TODO: In the future we want to improve record manipulation. For example we could use frozen keyword like:
# frozen rec_frozen = { a: 1, b: 2 }
# rec = { a: 2, b: 3 }
# rec_frozen.a = 4 # throws error
# rec.a = 4 # works
@assoc.register
def _(record: dict, key: str, val: Any) -> dict:
    tmp = dict(record)
    tmp[key] = val
    return tmp
