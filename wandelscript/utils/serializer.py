import json
from functools import singledispatch

import pydantic

from nova.types import Pose, Vector3d
from wandelscript.datatypes import ElementType

JsonType = float | int | bool | str | list | dict | None


class Element(pydantic.BaseModel):
    value: JsonType


class SerializedStore(pydantic.BaseModel):
    items: dict[str, JsonType]


@singledispatch
def decode(obj):
    """Decodes a plain dict to the matching pydantic model or elemental type.

    Args:
        obj: the obj to decode

    Returns: the pydantic model or elemental type

    Examples:
    >>> decode({ "id": "id1", "state": "stopped", "data": { "a": 1, "b": 2 } })
    {'id': 'id1', 'state': 'stopped', 'data': {'a': 1, 'b': 2}}
    >>> decode([{ "id": "id1", "state": "stopped"}, {"id": "id2", "state": "running"}])
    [{'id': 'id1', 'state': 'stopped'}, {'id': 'id2', 'state': 'running'}]
    >>> decode("Hello, World!")
    'Hello, World!'
    >>> decode(42)
    42
    >>> decode([1, 2, 3])
    [1, 2, 3]
    >>> decode(None)
    >>> decode({"position": [1, 2, 3], "orientation": [4, 5, 6]})
    Pose(position=Vector3d(x=1, y=2, z=3), orientation=Vector3d(x=4, y=5, z=6))
    >>> decode({"x": 1, "y": 2, "z": 3})
    Vector3d(x=1, y=2, z=3)
    """
    raise NotImplementedError(type(obj))


@decode.register
def _(obj: Element):
    """Decodes `Element` by decoding whatever it holds."""
    return decode(obj.value)


@decode.register
def _(obj: SerializedStore):
    """Decodes `SerializedStore` into a dict by decoding its items."""
    return {k: decode(v) for k, v in obj.items.items()}


@decode.register
def _(obj: dict):
    """
    If a dictionary has Pose-like or Vector3d-like fields, decode as that.
    Otherwise decode its children.
    """
    if "position" in obj and "orientation" in obj:
        return Pose(**obj)
    if {"x", "y", "z"} <= set(obj):
        return Vector3d(**obj)
    return {k: decode(v) for k, v in obj.items()}


@decode.register(list)
@decode.register(tuple)
def _(obj):
    """Decode lists/tuples by recursively decoding contents,
    preserving the original sequence type."""
    return type(obj)(map(decode, obj))


# Simple scalar registrations
@decode.register(int)
@decode.register(float)
@decode.register(str)
@decode.register(bool)
def _(obj):
    return obj


@decode.register(type(None))
def _(obj):
    return None


@singledispatch
def encode(obj):
    """Encodes an object to a pydantic model
    Args:
        obj: the object to encode
    Returns: the pydantic model
    Examples:
    >>> encode(Pose((1, 2, 3, 4, 5, 6)))
    {'position': [1, 2, 3], 'orientation': [4, 5, 6]}
    >>> encode(Vector3d(x=1, y=2, z=3))
    {'x': 1, 'y': 2, 'z': 3}
    >>> encode([1, 2, 3])
    [1, 2, 3]
    >>> encode(2.0)
    2.0
    """
    raise NotImplementedError(type(obj))


@encode.register
def _(obj: Pose):
    """Encodes Pose by turning it into a dict (via model_dump)."""
    return obj.model_dump()


@encode.register
def _(obj: Vector3d):
    """Encodes Vector3d by turning it into a dict (via model_dump)."""
    return obj.model_dump()


@encode.register(list)
@encode.register(tuple)
def _(obj):
    """Encodes lists/tuples by recursively encoding the contents,
    preserving the original sequence type."""
    return type(obj)(map(encode, obj))


@encode.register(dict)
def _(obj):
    """Encodes dictionaries by recursively encoding the values."""
    return {k: encode(v) for k, v in obj.items()}


@encode.register
def _(obj: SerializedStore):
    """Encodes a SerializedStore back into plain dict of JSON-compatible objects."""
    return {k: encode(v) for k, v in obj.items.items()}


@encode.register
def _(obj: Element):
    """Element is already a pydantic model, so pass through its fields or keep as-is."""
    return obj


# Simple scalars
@encode.register(int)
@encode.register(float)
@encode.register(str)
@encode.register(bool)
def _(obj):
    return obj


@encode.register(type(None))
def _(obj):
    return None


def dumps(data: ElementType) -> str:
    return Element(value=encode(data)).model_dump_json()


def loads(s: str) -> ElementType:
    data = json.loads(s)
    return decode(Element(**data))


def loads_store(content: str) -> ElementType:
    """
    Example:
    >>> json_content = '{"a": {"orientation": [1.0, 2.0, 3.0], "position": [4.0, 5.0, 6.0]}, "a_dict": {"a": 4}}'
    >>> loads_store(json_content)
    {'a': Pose(position=Vector3d(x=4.0, y=5.0, z=6.0), orientation=Vector3d(x=1.0, y=2.0, z=3.0)), 'a_dict': {'a': 4}}
    >>> json_content = '{"b": {"a": 3}, "c": [1, 2, 3]}'
    >>> loads_store(json_content)
    {'b': {'a': 3}, 'c': [1, 2, 3]}
    """
    data = json.loads(content)
    return decode(SerializedStore(items=data))


def is_encodable(obj):
    encodables = set(encode.registry.keys())
    encodables = [e for e in encodables if not isinstance(object, e)]
    encodables = [e for e in encodables if not isinstance(None, e)]
    return isinstance(obj, tuple(encodables))
