"""GR00T ZMQ transport and msgpack serialization.

Handles the low-level ZMQ REQ/REP protocol and msgpack encoding/decoding
with numpy array support. This module is an implementation detail —
users interact with ``Gr00tPolicyClient``.
"""

from __future__ import annotations

import importlib
import io
from typing import TYPE_CHECKING, Any, cast

import numpy as np

if TYPE_CHECKING:
    from types import ModuleType

_msgpack: ModuleType | None
try:
    _msgpack = importlib.import_module("msgpack")
except ImportError:  # pragma: no cover
    _msgpack = None

_zmq: ModuleType | None
try:
    _zmq = importlib.import_module("zmq")
except ImportError:  # pragma: no cover
    _zmq = None


class Gr00tMsgSerializer:
    """Msgpack serializer compatible with GR00T's ndarray transport.

    Encodes numpy arrays as ``{__ndarray_class__: True, as_npy: <bytes>}``
    and decodes them back on the receiving side.
    """

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
        np.save(output, cast("Any", obj), allow_pickle=False)
        return {"__ndarray_class__": True, "as_npy": output.getvalue()}


class Gr00tZmqTransport:
    """Manages a ZMQ REQ socket for GR00T server communication.

    Handles connect, send/receive with timeout, and reconnection on timeout.
    """

    def __init__(
        self,
        host: str,
        port: int = 5555,
        timeout_ms: int = 15000,
        api_token: str | None = None,
    ) -> None:
        self._host = host
        self._port = port
        self._timeout_ms = timeout_ms
        self._api_token = api_token
        self._context: Any | None = None
        self._socket: Any | None = None

    def connect(self) -> None:
        """Create or reconnect the ZMQ REQ socket."""
        zmq_module = _require_zmq()
        if self._socket is not None:
            self._socket.close(linger=0)
        context = self._context
        if context is None:
            context = zmq_module.Context()
            self._context = context
        socket = context.socket(zmq_module.REQ)
        socket.setsockopt(zmq_module.RCVTIMEO, self._timeout_ms)
        socket.setsockopt(zmq_module.SNDTIMEO, self._timeout_ms)
        socket.connect(f"tcp://{self._host}:{self._port}")
        self._socket = socket

    def close(self) -> None:
        """Close the socket and terminate the ZMQ context."""
        if self._socket is not None:
            self._socket.close(linger=0)
            self._socket = None
        if self._context is not None:
            self._context.term()
            self._context = None

    def call(
        self,
        endpoint: str,
        data: dict[str, object] | None = None,
    ) -> object:
        """Send a request to the GR00T server and return the response.

        Args:
            endpoint: GR00T endpoint name (``"ping"``, ``"get_action"``, etc.).
            data: Optional request payload.

        Returns:
            Deserialized response (dict, list, or tuple).

        Raises:
            TimeoutError: If the server doesn't respond within ``timeout_ms``.
            RuntimeError: If the server returns an error.
        """
        zmq_module = _require_zmq()
        socket = self._socket
        if socket is None:
            self.connect()
            socket = self._socket
        if socket is None:
            raise RuntimeError("Failed to create GR00T ZMQ socket")

        request: dict[str, object] = {"endpoint": endpoint}
        if data is not None:
            request["data"] = data
        if self._api_token is not None:
            request["api_token"] = self._api_token

        try:
            socket.send(Gr00tMsgSerializer.to_bytes(request))
            message = socket.recv()
        except zmq_module.error.Again as exc:
            self.connect()  # Reconnect on timeout
            msg = f"Timed out calling GR00T endpoint '{endpoint}'"
            raise TimeoutError(msg) from exc

        response = Gr00tMsgSerializer.from_bytes(message)
        if isinstance(response, dict) and "error" in response:
            error = cast("dict[str, object]", response)["error"]
            msg = f"GR00T server error: {error}"
            raise RuntimeError(msg)
        return response


def require_dict(response: object, *, name: str) -> dict[str, object]:
    """Validate that a response is a dict."""
    if not isinstance(response, dict):
        msg = f"{name} must be a dict"
        raise TypeError(msg)
    return cast("dict[str, object]", response)


def _require_msgpack() -> ModuleType:
    if _msgpack is None:
        msg = "msgpack is required for Gr00tPolicyClient. Install with: wandelbots-nova[novapolicy-gr00t]"
        raise ModuleNotFoundError(msg)
    return _msgpack


def _require_zmq() -> ModuleType:
    if _zmq is None:
        msg = "pyzmq is required for Gr00tPolicyClient. Install with: wandelbots-nova[novapolicy-gr00t]"
        raise ModuleNotFoundError(msg)
    return _zmq
