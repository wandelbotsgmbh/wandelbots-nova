"""GR00T ZeroMQ policy client.

Converts between the executor's observation format (RobotState dicts + numpy
images) and GR00T's fixed numpy-based format, using the ``PolicySchema`` for
key naming and DOF handling.

ZMQ transport and msgpack serialization live in ``transport.py``.
"""

from __future__ import annotations

import asyncio
import logging
from typing import TYPE_CHECKING

import numpy as np

from policy.gr00t.transport import Gr00tZmqTransport, require_dict
from policy.policy_client import PolicyClient
from policy.types import ActionChunk

logger = logging.getLogger(__name__)

if TYPE_CHECKING:
    from typing import Any

    from policy.schema import GroupedObservation, PolicySchema

_RESPONSE_PAIR_SIZE = 2
_IMAGE_NDIM_SINGLE = 3  # (H, W, C)
_IMAGE_NDIM_TEMPORAL = 4  # (T, H, W, C)
_IMAGE_CHANNELS = 3
_ACTION_NDIM = 3


class Gr00tPolicyClient(PolicyClient):
    """Policy client for GR00T ZeroMQ inference servers.

    Uses the executor's ``PolicySchema`` (passed to ``get_actions()``) to build
    GR00T observations and decode GR00T actions.

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
    model_dof:
        If set, pad/truncate joint arrays to this DOF. 0 = use actual.
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
        model_dof: int = 0,
    ) -> None:
        self._transport = Gr00tZmqTransport(
            host=host, port=port, timeout_ms=timeout_ms, api_token=api_token,
        )
        self._language = language
        self._dt_ms = dt_ms
        self._model_dof = model_dof
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
        schema: PolicySchema,
        images: dict[str, Any] | None = None,
        io_values: dict[str, object] | None = None,
    ) -> ActionChunk:
        """Build GR00T observation from robot states + images, send, decode response."""
        grouped = schema.build_grouped_observation(states, io_values)
        groot_obs = self._build_groot_obs(grouped, images)

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
        return self._decode_action(grouped, action_raw, info_raw)

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
    # Observation building
    # ------------------------------------------------------------------

    def _build_groot_obs(
        self,
        grouped: list[GroupedObservation],
        images: dict[str, Any] | None = None,
    ) -> dict[str, Any]:
        """Convert grouped observations → GR00T format."""
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
        self, grouped: list[GroupedObservation],
    ) -> dict[str, np.ndarray]:
        """Convert GroupedObservation list to GR00T numpy state dict."""
        state: dict[str, np.ndarray] = {}

        for gobs in grouped:
            joints = gobs.joints

            # Track actual DOF for action truncation
            self._actual_dof[gobs.motion_group_id] = len(joints)

            # Pad if model expects more joints
            if self._model_dof > len(joints):
                if gobs.motion_group_id not in self._dof_warned:
                    self._dof_warned.add(gobs.motion_group_id)
                    logger.warning(
                        "Model expects %d joints but %s has %d — padding with zeros",
                        self._model_dof, gobs.motion_group_id, len(joints),
                    )
                joints = [*joints, *([0.0] * (self._model_dof - len(joints)))]

            state[gobs.key] = _to_state_array(joints)

            if gobs.tcp is not None and gobs.tcp_key:
                state[gobs.tcp_key] = _to_state_array(gobs.tcp)

            if gobs.ios is not None:
                for io_name, io_val in gobs.ios.items():
                    state[io_name] = _to_state_array(
                        [io_val * 100.0 if io_val <= 1.0 else io_val]
                    )

        return state

    # ------------------------------------------------------------------
    # Action decoding
    # ------------------------------------------------------------------

    def _decode_action(
        self,
        grouped: list[GroupedObservation],
        action: dict[str, object],
        info: dict[str, object],
    ) -> ActionChunk:
        """Convert GR00T action arrays → ActionChunk."""
        joints: dict[str, list[list[float]]] = {}

        for gobs in grouped:
            arr = action.get(gobs.key)
            if isinstance(arr, np.ndarray) and arr.ndim == _ACTION_NDIM:
                joint_data = arr[0].astype(np.float32)
                actual_dof = self._actual_dof.get(gobs.motion_group_id)
                if actual_dof and joint_data.shape[1] > actual_dof:
                    joint_data = joint_data[:, :actual_dof]
                joints[gobs.motion_group_id] = joint_data.tolist()

        dt_ms = float(info.get("dt_ms", self._dt_ms))
        return ActionChunk(joints=joints, dt_ms=dt_ms)


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
