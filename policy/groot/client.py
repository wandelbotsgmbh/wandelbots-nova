"""GR00T ZeroMQ policy client.

Converts between the executor's observation format (RobotState dicts + numpy
images) and GR00T's fixed numpy-based format, using the ``FeatureMap`` for
key naming and DOF handling.

ZMQ transport and msgpack serialization live in ``_gr00t_transport.py``.
"""

from __future__ import annotations

import asyncio
import logging
from typing import TYPE_CHECKING

import numpy as np

from policy.groot.transport import Gr00tZmqTransport, require_dict
from policy.policy_client import PolicyClient
from policy.types import ActionChunk

logger = logging.getLogger(__name__)

if TYPE_CHECKING:
    from typing import Any

    from policy.feature_map import FeatureMap, GroupObservation

_RESPONSE_PAIR_SIZE = 2
_IMAGE_NDIM_SINGLE = 3  # (H, W, C)
_IMAGE_NDIM_TEMPORAL = 4  # (T, H, W, C)
_IMAGE_CHANNELS = 3
_ACTION_NDIM = 3


class Gr00tPolicyClient(PolicyClient):
    """Policy client for GR00T ZeroMQ inference servers.

    Uses the executor's ``FeatureMap`` (passed to ``get_actions()``) to build
    GR00T observations and decode GR00T actions.

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
        language: str = "",
        port: int = 5555,
        timeout_ms: int = 15000,
        api_token: str | None = None,
        dt_ms: float = 33.0,
    ) -> None:
        self._transport = Gr00tZmqTransport(
            host=host, port=port, timeout_ms=timeout_ms, api_token=api_token,
        )
        self._language = language
        self._dt_ms = dt_ms
        self._motion_group_ids: list[str] = []
        self._actual_dof: dict[str, int] = {}
        self._dof_warned: set[str] = set()

    async def connect(self, motion_group_ids: list[str]) -> None:
        """Create the ZMQ REQ socket."""
        self._motion_group_ids = list(motion_group_ids)
        await asyncio.to_thread(self._transport.connect)

    async def get_actions(
        self,
        states: dict[str, Any],
        feature_map: FeatureMap,
        images: dict[str, Any] | None = None,
        io_values: dict[str, object] | None = None,
    ) -> ActionChunk:
        """Build GR00T observation from robot states + images, send, decode response."""
        groot_obs = self._build_observation(feature_map, states, images, io_values)
        response = await asyncio.to_thread(
            self._transport.call,
            "get_action",
            {"observation": groot_obs, "options": None},
        )
        if not isinstance(response, (list, tuple)) or len(response) != _RESPONSE_PAIR_SIZE:
            msg = "GR00T get_action response must be a 2-tuple of (action, info)"
            raise TypeError(msg)

        action_raw = require_dict(response[0], name="GR00T action")
        info_raw = require_dict(response[1], name="GR00T info")
        return self._decode_action(feature_map, action_raw, info_raw)

    async def close(self) -> None:
        """Close the socket and terminate the ZMQ context."""
        await asyncio.to_thread(self._transport.close)

    async def ping(self) -> bool:
        """Check whether the GR00T server is reachable."""
        try:
            await asyncio.to_thread(self._transport.call, "ping")
        except TimeoutError:
            return False
        return True

    async def reset(self) -> dict[str, object]:
        """Reset remote policy state."""
        response = await asyncio.to_thread(
            self._transport.call, "reset", {"options": None},
        )
        return require_dict(response, name="GR00T reset response")

    async def get_modality_config(self) -> dict[str, object]:
        """Fetch raw modality config metadata from the server."""
        response = await asyncio.to_thread(self._transport.call, "get_modality_config")
        return require_dict(response, name="GR00T get_modality_config response")

    # ------------------------------------------------------------------
    # Observation building (uses FeatureMap.build_grouped_observation)
    # ------------------------------------------------------------------

    def _build_observation(
        self,
        feature_map: FeatureMap,
        states: dict[str, Any],
        images: dict[str, Any] | None = None,
        io_values: dict[str, object] | None = None,
    ) -> dict[str, Any]:
        """Convert executor obs → GR00T format using FeatureMap groups."""
        grouped = feature_map.build_grouped_observation(states, io_values)

        state = self._grouped_to_numpy(grouped)
        groot_obs: dict[str, Any] = {"state": state}

        if images:
            video = _build_video(images)
            if video:
                groot_obs["video"] = video

        if self._language:
            groot_obs["language"] = {
                "annotation.language.language_instruction": [[self._language]],
            }

        return groot_obs

    def _grouped_to_numpy(
        self, grouped: list[GroupObservation],
    ) -> dict[str, np.ndarray]:
        """Convert GroupObservation list to GR00T numpy state dict."""
        state: dict[str, np.ndarray] = {}

        for gobs in grouped:
            group = gobs.group
            joints = gobs.joints

            # Track actual DOF for action truncation
            self._actual_dof[group.motion_group.id] = len(joints)

            # Pad if model expects more joints
            if group.model_dof > len(joints):
                if group.motion_group.id not in self._dof_warned:
                    self._dof_warned.add(group.motion_group.id)
                    logger.warning(
                        "Model expects %d joints but %s has %d — padding with zeros",
                        group.model_dof, group.motion_group.id, len(joints),
                    )
                joints = [*joints, *([0.0] * (group.model_dof - len(joints)))]

            state[group.resolved_joint_key] = _to_state_array(joints)

            if gobs.tcp is not None:
                state[group.resolved_tcp_key] = _to_state_array(gobs.tcp)

            if gobs.ios is not None:
                for io_name, io_val in gobs.ios.items():
                    # GR00T typically uses 0/100 scale for gripper
                    state[group.resolved_io_key(io_name)] = _to_state_array(
                        [io_val * 100.0 if io_val <= 1.0 else io_val]
                    )

        return state

    # ------------------------------------------------------------------
    # Action decoding (GR00T numpy → ActionChunk using FeatureMap groups)
    # ------------------------------------------------------------------

    def _decode_action(
        self,
        feature_map: FeatureMap,
        action: dict[str, object],
        info: dict[str, object],
    ) -> ActionChunk:
        """Convert GR00T action arrays → ActionChunk."""
        joints: dict[str, list[list[float]]] = {}
        ios: dict[str, dict[str, bool | int | float | str]] = {}

        for group in feature_map.groups:
            arr = action.get(group.resolved_joint_key)
            if isinstance(arr, np.ndarray) and arr.ndim == _ACTION_NDIM:
                joint_data = arr[0].astype(np.float32)
                actual_dof = self._actual_dof.get(group.motion_group.id)
                if actual_dof and joint_data.shape[1] > actual_dof:
                    joint_data = joint_data[:, :actual_dof]
                joints[group.motion_group.id] = joint_data.tolist()

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


# ---------------------------------------------------------------------------
# Shared helpers
# ---------------------------------------------------------------------------


def _to_state_array(values: list[float] | tuple[float, ...]) -> np.ndarray:
    """Convert values to GR00T state format: (B=1, T=1, D)."""
    return np.asarray(values, dtype=np.float32)[np.newaxis, np.newaxis, :]


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
