"""GR00T ZeroMQ policy client.

This client speaks the GR00T REQ/REP msgpack protocol and adapts responses to the
``nova_policy.PolicyClient`` interface.
"""

from __future__ import annotations

import asyncio
import io
from collections.abc import Callable
from typing import TYPE_CHECKING, cast

import numpy as np

from nova_policy.types import ActionChunk, PolicyWaiting

if TYPE_CHECKING:
    from typing import Any

try:
    import msgpack as _msgpack
except ImportError:  # pragma: no cover - exercised when optional deps are absent
    _msgpack = None

try:
    import zmq as _zmq
except ImportError:  # pragma: no cover - exercised when optional deps are absent
    _zmq = None

_RESPONSE_PAIR_SIZE = 2

Gr00tDecodeResult = ActionChunk | dict[str, float] | dict[str, object] | None
Gr00tActionDecoder = Callable[[dict[str, object], dict[str, object]], Gr00tDecodeResult]


class Gr00tMsgSerializer:
    """Msgpack serializer compatible with GR00T's ndarray transport."""

    @staticmethod
    def to_bytes(data: object) -> bytes:
        """Serialize a Python value to msgpack bytes."""
        msgpack_module = _require_msgpack()
        return msgpack_module.packb(data, default=Gr00tMsgSerializer._encode_custom_classes)

    @staticmethod
    def from_bytes(data: bytes) -> object:
        """Deserialize msgpack bytes into Python values."""
        msgpack_module = _require_msgpack()
        return msgpack_module.unpackb(
            data,
            object_hook=Gr00tMsgSerializer._decode_custom_classes,
        )

    @staticmethod
    def _decode_custom_classes(obj: object) -> object:
        if not isinstance(obj, dict) or "__ndarray_class__" not in obj:
            return obj
        array_bytes = obj.get("as_npy")
        if not isinstance(array_bytes, bytes):
            msg = "Invalid ndarray payload in GR00T response"
            raise TypeError(msg)
        return np.load(io.BytesIO(array_bytes), allow_pickle=False)

    @staticmethod
    def _encode_custom_classes(obj: object) -> object:
        if not isinstance(obj, np.ndarray):
            return obj
        output = io.BytesIO()
        np.save(output, obj, allow_pickle=False)
        return {"__ndarray_class__": True, "as_npy": output.getvalue()}


class Gr00tPolicyClient:
    """Policy client for GR00T ZeroMQ inference servers.

    GR00T servers expose a single REQ/REP endpoint for the full embodiment, not
    one connection per robot. The caller is responsible for building a valid
    GR00T observation and decoding the returned GR00T action chunk.
    """

    def __init__(
        self,
        host: str,
        *,
        decode_action: Gr00tActionDecoder,
        port: int = 5555,
        timeout_ms: int = 15000,
        api_token: str | None = None,
    ) -> None:
        self._host = host
        self._port = port
        self._timeout_ms = timeout_ms
        self._api_token = api_token
        self._decode_action = decode_action
        self._context: object | None = None
        self._socket: object | None = None
        self._motion_group_ids: list[str] = []

    async def connect(self, motion_group_ids: list[str]) -> None:
        """Create the ZMQ REQ socket and remember the local motion groups."""
        self._motion_group_ids = list(motion_group_ids)
        await asyncio.to_thread(self._init_socket)

    async def get_actions(self, obs: dict[str, object]) -> ActionChunk | PolicyWaiting | dict[str, float]:
        """Send a GR00T observation and decode the returned action chunk."""
        response = await asyncio.to_thread(
            self._call_endpoint,
            "get_action",
            {"observation": obs, "options": None},
        )
        if not isinstance(response, (list, tuple)) or len(response) != _RESPONSE_PAIR_SIZE:
            msg = "GR00T get_action response must be a 2-tuple of (action, info)"
            raise TypeError(msg)

        action_raw = _require_dict(response[0], name="GR00T action")
        info_raw = _require_dict(response[1], name="GR00T info")
        decoded = self._decode_action(action_raw, info_raw)
        if decoded is None:
            return PolicyWaiting()
        if isinstance(decoded, ActionChunk):
            return decoded
        if isinstance(decoded, dict):
            if "joints" in decoded:
                return ActionChunk.from_dict(cast("dict[str, object]", decoded))
            return cast("dict[str, float]", decoded)
        msg = f"Unsupported decoded GR00T action type: {type(decoded)!r}"
        raise TypeError(msg)

    async def notify_stopped(self, reason: str) -> None:
        """No-op. GR00T's base protocol has no executor_stopped endpoint."""
        del reason

    async def close(self) -> None:
        """Close the socket and terminate the ZMQ context."""
        await asyncio.to_thread(self._close_blocking)

    async def ping(self) -> bool:
        """Check whether the GR00T server is reachable."""
        try:
            await asyncio.to_thread(self._call_endpoint, "ping")
        except TimeoutError:
            return False
        return True

    async def reset(self) -> dict[str, object]:
        """Reset remote policy state.

        GR00T's real ``Gr00tPolicy`` is stateless and returns an empty dict.
        Replay-style policies may use this endpoint for episode switching.
        """
        response = await asyncio.to_thread(self._call_endpoint, "reset", {"options": None})
        return _require_dict(response, name="GR00T reset response")

    async def get_modality_config(self) -> dict[str, object]:
        """Fetch raw modality config metadata from the server."""
        response = await asyncio.to_thread(self._call_endpoint, "get_modality_config")
        return _require_dict(response, name="GR00T get_modality_config response")

    def _close_blocking(self) -> None:
        socket = cast("Any | None", self._socket)
        context = cast("Any | None", self._context)
        if socket is not None:
            socket.close(linger=0)
            self._socket = None
        if context is not None:
            context.term()
            self._context = None

    def _init_socket(self) -> None:
        zmq_module = _require_zmq()
        socket = cast("Any | None", self._socket)
        if socket is not None:
            socket.close(linger=0)
        if self._context is None:
            self._context = zmq_module.Context()
        context = cast("Any", self._context)
        new_socket = context.socket(zmq_module.REQ)
        new_socket.setsockopt(zmq_module.RCVTIMEO, self._timeout_ms)
        new_socket.setsockopt(zmq_module.SNDTIMEO, self._timeout_ms)
        new_socket.connect(f"tcp://{self._host}:{self._port}")
        self._socket = new_socket

    def _call_endpoint(
        self,
        endpoint: str,
        data: dict[str, object] | None = None,
    ) -> object:
        zmq_module = _require_zmq()
        if self._socket is None:
            self._init_socket()
        socket = cast("Any | None", self._socket)
        if socket is None:
            msg = "Failed to initialize ZMQ socket"
            raise RuntimeError(msg)

        request: dict[str, object] = {"endpoint": endpoint}
        if data is not None:
            request["data"] = data
        if self._api_token is not None:
            request["api_token"] = self._api_token

        try:
            socket.send(Gr00tMsgSerializer.to_bytes(request))
            message = socket.recv()
        except zmq_module.error.Again as exc:
            self._init_socket()
            msg = f"Timed out calling GR00T endpoint '{endpoint}'"
            raise TimeoutError(msg) from exc

        response = Gr00tMsgSerializer.from_bytes(message)
        if isinstance(response, dict) and "error" in response:
            error_message = response["error"]
            msg = f"GR00T server error: {error_message}"
            raise RuntimeError(msg)
        return response


def _require_dict(response: object, *, name: str) -> dict[str, object]:
    if not isinstance(response, dict):
        msg = f"{name} must be a dict"
        raise TypeError(msg)
    return cast("dict[str, object]", response)


def _require_msgpack() -> object:
    if _msgpack is None:
        msg = "msgpack is required for Gr00tPolicyClient"
        raise ModuleNotFoundError(msg)
    return _msgpack


def _require_zmq() -> object:
    if _zmq is None:
        msg = "pyzmq is required for Gr00tPolicyClient"
        raise ModuleNotFoundError(msg)
    return _zmq
