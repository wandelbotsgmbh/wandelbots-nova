"""GR00T ZMQ transport and msgpack serialization.

Handles the low-level ZMQ REQ/REP protocol and msgpack encoding/decoding
with numpy array support. This module is an implementation detail —
users interact with ``Gr00tPolicyClient``.
"""

from __future__ import annotations

import io
from typing import Any, cast

import numpy as np

try:
    import msgpack as _msgpack
except ImportError:  # pragma: no cover
    _msgpack = None

try:
    import zmq as _zmq
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
        np.save(output, obj, allow_pickle=False)
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
        if self._context is None:
            self._context = zmq_module.Context()
        self._socket = self._context.socket(zmq_module.REQ)
        self._socket.setsockopt(zmq_module.RCVTIMEO, self._timeout_ms)
        self._socket.setsockopt(zmq_module.SNDTIMEO, self._timeout_ms)
        self._socket.connect(f"tcp://{self._host}:{self._port}")

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
        if self._socket is None:
            self.connect()

        request: dict[str, object] = {"endpoint": endpoint}
        if data is not None:
            request["data"] = data
        if self._api_token is not None:
            request["api_token"] = self._api_token

        try:
            self._socket.send(Gr00tMsgSerializer.to_bytes(request))
            message = self._socket.recv()
        except zmq_module.error.Again as exc:
            self.connect()  # Reconnect on timeout
            msg = f"Timed out calling GR00T endpoint '{endpoint}'"
            raise TimeoutError(msg) from exc

        response = Gr00tMsgSerializer.from_bytes(message)
        if isinstance(response, dict) and "error" in response:
            msg = f"GR00T server error: {response['error']}"
            raise RuntimeError(msg)
        return response


def require_dict(response: object, *, name: str) -> dict[str, object]:
    """Validate that a response is a dict."""
    if not isinstance(response, dict):
        msg = f"{name} must be a dict"
        raise TypeError(msg)
    return cast("dict[str, object]", response)


def _require_msgpack() -> object:
    if _msgpack is None:
        msg = "msgpack is required for Gr00tPolicyClient. Install with: wandelbots-nova[novapolicy-gr00t]"
        raise ModuleNotFoundError(msg)
    return _msgpack


def _require_zmq() -> object:
    if _zmq is None:
        msg = "pyzmq is required for Gr00tPolicyClient. Install with: wandelbots-nova[novapolicy-gr00t]"
        raise ModuleNotFoundError(msg)
    return _zmq
