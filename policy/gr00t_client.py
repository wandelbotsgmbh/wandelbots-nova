"""GR00T ZeroMQ policy client.

This client speaks the GR00T REQ/REP msgpack protocol and converts between
the executor's observation format and GR00T's fixed numpy-based format.

Observation building and action decoding are driven by the same ``FeatureMap``
that the executor uses — no redundant mapping needed.
"""

from __future__ import annotations

import asyncio
import io
import logging
from typing import TYPE_CHECKING, cast

import numpy as np

from policy.pose import pose_to_tcp
from policy.types import ActionChunk

logger = logging.getLogger(__name__)

if TYPE_CHECKING:
    from typing import Any

    from policy.feature_map import FeatureGroup, FeatureMap

try:
    import msgpack as _msgpack
except ImportError:  # pragma: no cover
    _msgpack = None

try:
    import zmq as _zmq
except ImportError:  # pragma: no cover
    _zmq = None

_RESPONSE_PAIR_SIZE = 2
_IMAGE_NDIM_SINGLE = 3  # (H, W, C)
_IMAGE_NDIM_TEMPORAL = 4  # (T, H, W, C)
_IMAGE_CHANNELS = 3
_ACTION_NDIM = 3


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

    Uses the same ``FeatureMap`` as the executor to build GR00T observations
    and decode GR00T actions — no separate embodiment config needed.

    Key names and DOF handling are configured on the ``FeatureGroup``:
    - ``joint_key``: key for joints (default: ``{name}_joint_position``)
    - ``tcp_key``: key for TCP pose (default: ``{name}_tcp``)
    - ``tcp_format``: pose format (``TcpFormat.ROT6D``, etc.)
    - ``ios``: dict of io feature names → hardware keys
    - ``model_dof``: pad/truncate joints to this DOF (0 = use actual)

    Parameters
    ----------
    host:
        Hostname of the GR00T ZMQ server.
    feature_map:
        The FeatureMap defining robot topology and GR00T key mappings.
    language:
        Language instruction sent with every observation.
    port:
        ZMQ port (default 5555).
    timeout_ms:
        ZMQ send/recv timeout in milliseconds.
    api_token:
        Optional API token for authenticated servers.
    dt_ms:
        Default step spacing if not in action info.
    """

    def __init__(
        self,
        host: str,
        *,
        feature_map: FeatureMap,
        language: str = "",
        port: int = 5555,
        timeout_ms: int = 15000,
        api_token: str | None = None,
        dt_ms: float = 33.0,
    ) -> None:
        self._host = host
        self._port = port
        self._timeout_ms = timeout_ms
        self._api_token = api_token
        self._feature_map = feature_map
        self._language = language
        self._dt_ms = dt_ms
        self._context: object | None = None
        self._socket: object | None = None
        self._motion_group_ids: list[str] = []
        self._actual_dof: dict[str, int] = {}
        self._dof_warned: set[str] = set()

    async def connect(self, motion_group_ids: list[str]) -> None:
        """Create the ZMQ REQ socket."""
        self._motion_group_ids = list(motion_group_ids)
        await asyncio.to_thread(self._init_socket)

    async def get_actions(self, obs: dict[str, Any]) -> ActionChunk:
        """Build GR00T observation from executor state, send, decode response.

        ``obs`` is the raw executor observation: ``{mg_id: RobotState, cam: ndarray}``.
        """
        groot_obs = self._build_observation(obs)
        response = await asyncio.to_thread(
            self._call_endpoint,
            "get_action",
            {"observation": groot_obs, "options": None},
        )
        if not isinstance(response, (list, tuple)) or len(response) != _RESPONSE_PAIR_SIZE:
            msg = "GR00T get_action response must be a 2-tuple of (action, info)"
            raise TypeError(msg)

        action_raw = _require_dict(response[0], name="GR00T action")
        info_raw = _require_dict(response[1], name="GR00T info")
        return self._decode_action(action_raw, info_raw)

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
        """Reset remote policy state."""
        response = await asyncio.to_thread(self._call_endpoint, "reset", {"options": None})
        return _require_dict(response, name="GR00T reset response")

    async def get_modality_config(self) -> dict[str, object]:
        """Fetch raw modality config metadata from the server."""
        response = await asyncio.to_thread(self._call_endpoint, "get_modality_config")
        return _require_dict(response, name="GR00T get_modality_config response")


    # ------------------------------------------------------------------
    # Observation building (FeatureMap → GR00T numpy format)
    # ------------------------------------------------------------------

    def _build_observation(self, obs: dict[str, Any]) -> dict[str, Any]:
        """Convert executor obs → GR00T format using FeatureMap groups."""
        state = self._build_state(obs)
        groot_obs: dict[str, Any] = {"state": state}

        video = self._build_video(obs)
        if video:
            groot_obs["video"] = video

        if self._language:
            groot_obs["language"] = {
                "annotation.language.language_instruction": [[self._language]],
            }

        return groot_obs

    def _build_state(self, obs: dict[str, Any]) -> dict[str, np.ndarray]:
        """Build GR00T state dict from robot observations."""
        state: dict[str, np.ndarray] = {}

        for group in self._feature_map.groups:
            robot_state = obs.get(group.motion_group.id)
            if robot_state is None:
                continue

            joints = self._extract_joints(robot_state)
            self._actual_dof[group.motion_group.id] = len(joints)
            if group.model_dof > len(joints):
                if group.motion_group.id not in self._dof_warned:
                    self._dof_warned.add(group.motion_group.id)
                    logger.warning(
                        "Model expects %d joints but %s has %d — padding with zeros",
                        group.model_dof, group.motion_group.id, len(joints),
                    )
                joints = [*joints, *([0.0] * (group.model_dof - len(joints)))]
            state[group.resolved_joint_key] = _to_state_array(joints)

            if group.tcp_format and hasattr(robot_state, "pose") and robot_state.pose is not None:
                eef_values = pose_to_tcp(robot_state.pose, group.tcp_format)
                state[group.resolved_tcp_key] = _to_state_array(eef_values)

            if group.ios:
                for io_name, hw_key in group.ios.items():
                    io_val = self._read_io(group, hw_key)
                    state[group.resolved_io_key(io_name)] = _to_state_array([io_val])

        return state

    @staticmethod
    def _build_video(obs: dict[str, Any]) -> dict[str, np.ndarray]:
        """Extract camera frames from observation into GR00T video format."""
        video: dict[str, np.ndarray] = {}
        for key, value in obs.items():
            if not isinstance(value, np.ndarray):
                continue
            if value.ndim == _IMAGE_NDIM_SINGLE and value.shape[2] == _IMAGE_CHANNELS:
                video[key] = value[np.newaxis, np.newaxis, :, :, :]
            elif value.ndim == _IMAGE_NDIM_TEMPORAL and value.shape[3] == _IMAGE_CHANNELS:
                video[key] = value[np.newaxis, :, :, :, :]
        return video

    # ------------------------------------------------------------------
    # Action decoding (GR00T numpy → ActionChunk using FeatureMap groups)
    # ------------------------------------------------------------------

    def _decode_action(self, action: dict[str, object], info: dict[str, object]) -> ActionChunk:
        """Convert GR00T action arrays → ActionChunk."""
        joints: dict[str, list[list[float]]] = {}
        ios: dict[str, dict[str, bool | int | float | str]] = {}

        for group in self._feature_map.groups:
            # Joint targets
            arr = action.get(group.resolved_joint_key)
            if isinstance(arr, np.ndarray) and arr.ndim == _ACTION_NDIM:
                joint_data = arr[0].astype(np.float32)
                # Truncate to actual robot DOF if we padded on input
                actual_dof = self._actual_dof.get(group.motion_group.id)
                if actual_dof and joint_data.shape[1] > actual_dof:
                    joint_data = joint_data[:, :actual_dof]
                joints[group.motion_group.id] = joint_data.tolist()

            # IO actions
            if group.ios:
                for io_name, hw_key in group.ios.items():
                    io_arr = action.get(group.resolved_io_key(io_name))
                    if isinstance(io_arr, np.ndarray) and io_arr.size > 0:
                        io_val = float(io_arr.flat[0])
                        ios.setdefault(group.motion_group.id, {})[hw_key] = bool(
                            io_val >= group.io_threshold
                        )

        dt_ms = float(info.get("dt_ms", self._dt_ms))
        return ActionChunk(joints=joints, ios=ios or None, dt_ms=dt_ms)

    # ------------------------------------------------------------------
    # Helpers
    # ------------------------------------------------------------------

    @staticmethod
    def _extract_joints(state: object) -> list[float]:
        """Extract joint values from a RobotState or dict."""
        if hasattr(state, "joints"):
            return list(state.joints)
        if isinstance(state, dict) and "joints" in state:
            return list(state["joints"])
        msg = f"Cannot extract joints from {type(state)}"
        raise TypeError(msg)

    def _read_io(self, group: FeatureGroup, hw_key: str) -> float:
        """Read an IO value from the stream cache via FeatureMap."""
        cache = self._feature_map._get_cache(group.motion_group.id)
        if cache is not None:
            raw = cache.values.get(hw_key)
            if raw is not None:
                return 100.0 if raw else 0.0
        return 0.0

    # ------------------------------------------------------------------
    # ZMQ transport
    # ------------------------------------------------------------------

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




def _to_state_array(values: list[float] | tuple[float, ...]) -> np.ndarray:
    """Convert values to GR00T state format: (B=1, T=1, D)."""
    return np.asarray(values, dtype=np.float32)[np.newaxis, np.newaxis, :]


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
