"""Types and constructs for internal use."""

from __future__ import annotations

import uuid
from dataclasses import asdict, dataclass, field, is_dataclass
from functools import singledispatch
from typing import Any, Callable, Generic, TypeVar, Union

import numpy as np
import pydantic

from nova.types import Pose, Vector3d
from wandelscript.frames import FrameSystem
from wandelscript.utils.pose import pose_to_versor

# All types that can be used within Wandelscript
ElementType = Union[float, int, bool, str, bytes, Pose, Vector3d, tuple, list, dict]
BoundedElementType = TypeVar("BoundedElementType", bound=ElementType)


@dataclass(frozen=True)
class Frame:
    """A (physical) frame, i.e. coordinate system, in the space"""

    name: str
    system: FrameSystem = field(compare=False, hash=False)

    def __matmul__(self, other):
        assert False

    def __rmatmul__(self, other: Pose):
        assert isinstance(other, Pose)
        new_frame = Frame(uuid.uuid4().hex, self.system)
        self.system[self.name, new_frame.name] = pose_to_versor(other)
        return new_frame


@dataclass(frozen=True)
class Closure(Generic[BoundedElementType]):
    r"""A closure, i.e., an anonymous function together with a variable scope its implementation can refer to

    Example:
    >>> import asyncio
    >>> from wandelscript.metamodel import run_program
    >>> code = '''
    ... def foo():
    ...     def bar(u):
    ...         return 23
    ...     return bar
    ... b = foo()
    ... c = b(4)
    ... '''
    >>> context = asyncio.run(run_program(code))
    >>> context.store['c']
    23
    """

    store: Any
    body: Callable[..., ElementType]

    def __call__(self, store, *args) -> ElementType:
        return self.body(self.store, *args)

    def __matmul__(self, other):
        if isinstance(other, Pose):

            async def function(store, *args):  # pylint: disable=unused-argument
                pose = await self.body(self.store, *args)
                return pose @ other

            return Closure(self.store, function)
        if isinstance(other, Closure):

            async def function(store, *args):  # pylint: disable=unused-argument, function-redefined
                a = await self.body(self.store, *args)
                b = await other.body(other.store, *args)
                return a @ b

            return Closure(self.store, function)
        return NotImplemented

    def __rmatmul__(self, other):
        assert isinstance(other, Pose)

        async def function(store, *args):  # pylint: disable=unused-argument
            pose = await self.body(self.store, *args)
            return other @ pose

        return Closure(None, function)


@singledispatch
def as_builtin_type(data: Any) -> ElementType:
    if is_dataclass(data) and not isinstance(data, type):
        # dicts are Wandelscript records
        return asdict(data)

    raise TypeError(f"Datatype is not supported {type(data)}")


@as_builtin_type.register
def _(data: None):
    return None


@as_builtin_type.register
def _(data: bool):
    return data


@as_builtin_type.register
def _(data: int):
    return data


@as_builtin_type.register
def _(data: float):
    return data


@as_builtin_type.register
def _(data: np.float64):
    return float(data)


@as_builtin_type.register
def _(data: str):
    return data


@as_builtin_type.register
def _(data: Pose):
    return data


@as_builtin_type.register(tuple)
@as_builtin_type.register(list)
def _(data):
    return tuple(as_builtin_type(item) for item in data)


@as_builtin_type.register
def _(data: dict):
    return {key: as_builtin_type(value) for key, value in data.items()}


@as_builtin_type.register
def _(data: pydantic.BaseModel):
    return data.model_dump()


__all__ = ["Frame", "Closure", "as_builtin_type", "ElementType"]
