"""Gr00tMsgSerializer: the array formats GR00T policy servers answer with.

``npy`` (``{__ndarray_class__, as_npy}``) is what this client sends and what
N1.6 to the N1.7 release answer with; msgpack-numpy (``{nd, type, kind, shape,
data}``) is what Isaac-GR00T ``main`` answers with since 2026-05.
"""

from __future__ import annotations

import msgpack
import numpy as np
import pytest

from novapolicy.gr00t import Gr00tMsgSerializer


def _msgpack_numpy(value: np.ndarray | np.generic, *, bytes_keys: bool = True) -> dict:
    """``value`` the way msgpack-numpy packs it."""
    array = np.asarray(value)
    packed: dict = {
        "nd": isinstance(value, np.ndarray),
        "type": array.dtype.str,
        "data": array.tobytes(),
    }
    if packed["nd"]:
        packed |= {"kind": b"", "shape": list(array.shape)}
    return {key.encode(): item for key, item in packed.items()} if bytes_keys else packed


def _decode(payload: object) -> object:
    return Gr00tMsgSerializer.from_bytes(msgpack.packb(payload))


@pytest.mark.parametrize("bytes_keys", [True, False])
def test_serializer_decodes_msgpack_numpy_arrays_and_scalars(bytes_keys: bool) -> None:
    action = np.arange(18, dtype=np.float32).reshape(1, 3, 6)
    payload = msgpack.packb({
        "action": _msgpack_numpy(action, bytes_keys=bytes_keys),
        "scale": _msgpack_numpy(np.float32(2.5), bytes_keys=bytes_keys),
    })

    decoded = Gr00tMsgSerializer.from_bytes(payload)

    assert isinstance(decoded, dict)
    np.testing.assert_array_equal(decoded["action"], action)
    assert decoded["action"].flags.writeable
    assert decoded["scale"] == np.float32(2.5)


def test_serializer_still_round_trips_its_own_npy_arrays() -> None:
    action = np.ones((1, 16, 6), dtype=np.float32)

    decoded = Gr00tMsgSerializer.from_bytes(Gr00tMsgSerializer.to_bytes({"action": action}))

    assert isinstance(decoded, dict)
    np.testing.assert_array_equal(decoded["action"], action)


def test_serializer_reads_any_truthy_nd_as_an_array() -> None:
    action = np.arange(6, dtype=np.float32).reshape(1, 1, 6)
    packed = _msgpack_numpy(action) | {b"nd": 1}

    decoded = _decode({"action": packed})

    assert isinstance(decoded, dict)
    np.testing.assert_array_equal(decoded["action"], action)


def test_serializer_refuses_object_arrays() -> None:
    packed = _msgpack_numpy(np.zeros(1, dtype=np.float32)) | {b"kind": b"O"}

    with pytest.raises(TypeError, match="kind"):
        _decode({"action": packed})


def test_serializer_leaves_a_plain_dict_with_an_nd_key_alone() -> None:
    decoded = _decode({"info": {"nd": True, "steps": 3}})

    assert decoded == {"info": {"nd": True, "steps": 3}}


def test_serializer_refuses_record_arrays_by_their_kind() -> None:
    packed = _msgpack_numpy(np.zeros(1, dtype=np.float32)) | {
        b"kind": b"V",
        b"type": [["a", "<f4"]],
    }

    with pytest.raises(TypeError, match="kind"):
        _decode({"action": packed})


def test_serializer_rejects_a_scalar_without_exactly_one_value() -> None:
    packed = _msgpack_numpy(np.float32(1.0)) | {b"data": b""}

    with pytest.raises(TypeError, match="scalar"):
        _decode({"value": packed})
