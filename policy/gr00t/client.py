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

from policy.gr00t.eef import TcpFormat, pose_to_eef
from policy.gr00t.transport import Gr00tZmqTransport, require_dict
from policy.policy_client import PolicyClient
from policy.types import ActionChunk

logger = logging.getLogger(__name__)

if TYPE_CHECKING:
    from typing import Any

    from nova.types import RobotState
    from policy.schema import PolicySchema

_RESPONSE_PAIR_SIZE = 2
_IMAGE_NDIM_SINGLE = 3  # (H, W, C)
_IMAGE_NDIM_TEMPORAL = 4  # (T, H, W, C)
_IMAGE_CHANNELS = 3
_ACTION_NDIM = 3


class Gr00tPolicyClient(PolicyClient):
    """Policy client for NVIDIA GR00T N1.7 ZeroMQ inference servers.

    Receives the same raw ``(states, schema, images, io_values)`` as every
    other policy client. Internally converts to GR00T's numpy format using
    the schema's joint/TCP/IO mappings, sends over ZMQ, and decodes the
    response back to an ``ActionChunk``.

    This implementation targets GR00T protocol version
    :data:`~policy.gr00t.GROOT_PROTOCOL_VERSION`.

    Parameters
    ----------
    host:
        Hostname of the GR00T ZMQ server.
    port:
        ZMQ port (default 5555).
    timeout_ms:
        ZMQ send/recv timeout in milliseconds.
    api_token:
        Optional API token for authenticated servers.
    dt_ms:
        Default step spacing if not in action info.
    tcp_format:
        TCP pose representation format sent to the server.
    """

    def __init__(
        self,
        host: str,
        *,
        port: int = 5555,
        timeout_ms: int = 15000,
        api_token: str | None = None,
        dt_ms: float = 33.0,
        tcp_format: TcpFormat = TcpFormat.ROT6D,
    ) -> None:
        self._transport = Gr00tZmqTransport(
            host=host, port=port, timeout_ms=timeout_ms, api_token=api_token,
        )
        self._dt_ms = dt_ms
        self._tcp_format = tcp_format
        self._motion_group_ids: list[str] = []
        self._actual_dof: dict[str, int] = {}

    async def connect(self, motion_group_ids: list[str]) -> None:
        """Create the ZMQ REQ socket."""
        self._motion_group_ids = list(motion_group_ids)
        await asyncio.to_thread(self._transport.connect)

    async def validate_schema(self, schema: PolicySchema) -> None:
        """Check that the schema satisfies the GR00T server's expected modalities.

        Queries ``get_modality_config()`` from the server and verifies that all
        required state, video, and language keys are present in the schema.
        Raises ``ValueError`` with a clear message listing any missing keys.
        """
        config = await self.get_modality_config()
        errors: list[str] = []

        # Check state keys (joint_positions, etc.)
        server_state_keys = _extract_modality_keys(config, "state")
        schema_state_keys = {m.key for m in schema.joint_mappings}
        schema_state_keys |= {m.key for m in schema.tcp_mappings}
        missing_state = server_state_keys - schema_state_keys
        if missing_state:
            errors.append(f"Missing state observations: {sorted(missing_state)}")

        # Check video keys (camera images)
        server_video_keys = _extract_modality_keys(config, "video")
        schema_image_keys = set(schema.image_sources.keys())
        missing_video = server_video_keys - schema_image_keys
        if missing_video:
            errors.append(f"Missing image observations: {sorted(missing_video)}")

        # Check language keys
        server_lang_keys = _extract_modality_keys(config, "language")
        if server_lang_keys:
            has_language = bool(schema.constants.get("language"))
            if not has_language:
                errors.append(
                    "Server expects language instruction but schema has no "
                    'Observation.constant("language", ...)'
                )

        if errors:
            msg = (
                "Schema does not satisfy GR00T server requirements:\n"
                + "\n".join(f"  - {e}" for e in errors)
            )
            raise ValueError(msg)

    async def get_actions(
        self,
        states: dict[str, RobotState],
        schema: PolicySchema,
        images: dict[str, Any] | None = None,
        io_values: dict[str, object] | None = None,
    ) -> ActionChunk:
        """Build GR00T observation from raw states + images, send, decode response."""
        groot_obs = self._build_groot_obs(states, schema, images, io_values)

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
        return self._decode_action(schema, action_raw, info_raw)

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
    # Observation building — reads raw states via schema mappings
    # ------------------------------------------------------------------

    def _build_groot_obs(
        self,
        states: dict[str, RobotState],
        schema: PolicySchema,
        images: dict[str, Any] | None,
        io_values: dict[str, object] | None,
    ) -> dict[str, Any]:
        """Convert raw robot states → GR00T observation format."""
        state_dict = self._build_state_dict(states, schema, io_values)
        groot_obs: dict[str, Any] = {"state": state_dict}

        if images:
            video = _build_video(images)
            if video:
                groot_obs["video"] = video

        language = schema.constants.get("language", "")
        if language:
            groot_obs["language"] = {
                "annotation.language.language_instruction": [[language]],
            }

        return groot_obs

    def _build_state_dict(
        self,
        states: dict[str, RobotState],
        schema: PolicySchema,
        io_values: dict[str, object] | None,
    ) -> dict[str, np.ndarray]:
        """Build the GR00T state dict from robot states via schema mappings."""
        state_dict: dict[str, np.ndarray] = {}

        for m in schema.joint_mappings:
            concat_joints: list[float] = []
            for mg in m.sources:
                s = states.get(mg.id)
                if s is None:
                    continue
                joints = list(s.joints)
                self._actual_dof[mg.id] = len(joints)
                concat_joints.extend(joints)
            if concat_joints:
                state_dict[m.key] = _to_state_array(concat_joints)

        for tm in schema.tcp_mappings:
            s = states.get(tm.source.id)
            if s is not None and hasattr(s, "pose") and s.pose is not None:
                state_dict[tm.key] = _to_state_array(
                    pose_to_eef(s.pose, self._tcp_format)
                )

        if io_values:
            for iom in schema.obs_io_mappings:
                raw = io_values.get(iom.io)
                val = iom.mapping.to_policy(raw) if raw is not None else 0.0
                state_dict[iom.key] = _to_state_array([val])

        return state_dict

    # ------------------------------------------------------------------
    # Action decoding
    # ------------------------------------------------------------------

    def _decode_action(
        self,
        schema: PolicySchema,
        action: dict[str, object],
        info: dict[str, object],
    ) -> ActionChunk:
        """Convert GR00T action arrays → ActionChunk."""
        joints: dict[str, list[list[float]]] = {}
        ios: dict[str, dict[str, bool | int | float | str]] = {}

        for key, mgs in schema.joint_action_keys:
            arr = action.get(key)
            if isinstance(arr, np.ndarray) and arr.ndim == _ACTION_NDIM:
                for mg in mgs:
                    joint_data = arr[0].astype(np.float32)
                    actual_dof = self._actual_dof.get(mg.id)
                    if actual_dof and joint_data.shape[1] > actual_dof:
                        joint_data = joint_data[:, :actual_dof]
                    joints[mg.id] = joint_data.tolist()

        # Decode IO actions
        for key, mg, hw_key, mapping in schema.io_action_keys:
            arr = action.get(key)
            if isinstance(arr, np.ndarray):
                # GR00T IO arrays are (B, T, 1) — take the last timestep
                val = float(arr.flat[-1])
                ios.setdefault(mg.id, {})[hw_key] = mapping.to_hardware(val)
            elif key in action:
                val = float(action[key])  # type: ignore[arg-type]
                ios.setdefault(mg.id, {})[hw_key] = mapping.to_hardware(val)

        # Decode TCP actions
        tcp_targets: dict[str, list[list[float]]] = {}
        for key, mg in schema.tcp_action_keys:
            arr = action.get(key)
            if isinstance(arr, np.ndarray) and arr.ndim == _ACTION_NDIM:
                tcp_targets[mg.id] = arr[0].astype(np.float32).tolist()

        dt_ms = float(info.get("dt_ms", self._dt_ms))
        return ActionChunk(joints=joints, tcp=tcp_targets, ios=ios or None, dt_ms=dt_ms)


# ---------------------------------------------------------------------------
# Helpers
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


def _extract_modality_keys(config: dict[str, object], modality: str) -> set[str]:
    """Extract ``modality_keys`` from a GR00T ``get_modality_config`` response."""
    entry = config.get(modality)
    if not isinstance(entry, dict):
        return set()
    as_json = entry.get("as_json")
    if not isinstance(as_json, dict):
        return set()
    keys = as_json.get("modality_keys")
    if not isinstance(keys, list):
        return set()
    return {str(k) for k in keys}
