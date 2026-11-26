"""GR00T ZeroMQ policy client; the ZMQ transport and msgpack codec are in ``transport.py``."""

from __future__ import annotations

import asyncio
import json
import logging
from pathlib import Path
from typing import TYPE_CHECKING, cast

import numpy as np

from novapolicy.mapping import training_range
from novapolicy.providers.base import PolicyClient, resolve_control_freq
from novapolicy.providers.gr00t.eef import TcpFormat, pose_to_eef
from novapolicy.providers.gr00t.transport import Gr00tZmqTransport, require_dict

logger = logging.getLogger(__name__)

if TYPE_CHECKING:
    from typing import Any

    from nova.types import RobotState
    from novapolicy.mapping.schema import PolicySchema
    from novapolicy.types import ActionChunk

_RESPONSE_PAIR_SIZE = 2
_DEFAULT_LANGUAGE_KEY = "annotation.language.language_instruction"
_IMAGE_NDIM_SINGLE = 3  # (H, W, C)
_IMAGE_NDIM_TEMPORAL = 4  # (T, H, W, C)
_IMAGE_CHANNELS = 3
_ACTION_NDIM = 3
_PROTOCOL_HINT = (
    "This client speaks the GR00T N1.6+ policy server protocol "
    "(gr00t/policy/server_client.py); N1 and N1.5 servers (gr00t/eval/service.py) "
    "take a flat observation and answer with the action alone."
)


class Gr00tPolicyClient(PolicyClient):
    """Policy client for NVIDIA GR00T N1.7 ZeroMQ inference servers.

    Targets GR00T protocol version :data:`~novapolicy.providers.gr00t.GROOT_PROTOCOL_VERSION`.

    Args:
        host: Hostname of the GR00T ZMQ server.
        port: ZMQ port.
        timeout_ms: ZMQ send/recv timeout in milliseconds.
        api_token: Optional API token for authenticated servers.
        control_freq: Rate of the policy's action steps in Hz (the dataset's ``fps``), default
            30; a ``dt_ms`` in the server's action info wins.
        tcp_format: TCP format sent unless the ``Observation.tcp`` declares an
            ``orientation_type``.
        statistics: Path to the checkpoint's ``statistics.json``; the first observation is
            checked against it (``training_range``).
        embodiment: Embodiment tag in ``statistics.json``; default: the one whose state keys
            are exactly the server's.
    """

    def __init__(
        self,
        host: str,
        *,
        port: int = 5555,
        timeout_ms: int = 15000,
        api_token: str | None = None,
        control_freq: float | None = None,
        tcp_format: TcpFormat = TcpFormat.ROT6D,
        statistics: str | Path | None = None,
        embodiment: str | None = None,
    ) -> None:
        self._transport = Gr00tZmqTransport(
            host=host,
            port=port,
            timeout_ms=timeout_ms,
            api_token=api_token,
        )
        self._dt_ms = 1000.0 / resolve_control_freq(control_freq, owner="Gr00tPolicyClient")
        self._tcp_format = tcp_format
        # The checkpoint's statistics.json, to hold the first observation against
        self._statistics: dict[str, Any] = (
            json.loads(Path(statistics).read_text(encoding="utf-8")) if statistics else {}
        )
        self._embodiment = embodiment
        self._state_ranges: dict[str, list[tuple[float, float]]] = {}
        self._range_checked = False
        self._actual_dof: dict[str, int] = {}
        # What the server takes, learned in validate_schema (the executor calls it before
        # the run): the state keys it expects and the key its language instruction goes in.
        self._server_state_keys: set[str] | None = None
        self._language_key = _DEFAULT_LANGUAGE_KEY

    async def connect(self, motion_group_ids: list[str]) -> None:  # ruff: ignore[unused-method-argument]
        """Create the ZMQ REQ socket; called once per episode by the executor."""
        self._range_checked = False
        await asyncio.to_thread(self._transport.connect)

    async def validate_schema(self, schema: PolicySchema) -> None:
        """Check that the schema has every state, video and language key the server expects.

        Raises:
            ValueError: listing the missing keys.
        """
        config = await self.get_modality_config()
        errors: list[str] = []

        server_state_keys = _extract_modality_keys(config, "state")
        schema_state_keys = {m.key for m in schema.joint_mappings}
        schema_state_keys |= {m.key for m in schema.tcp_mappings}
        schema_state_keys |= {m.key for m in schema.obs_io_mappings}
        schema_state_keys |= {m.key for m in schema.joint_velocity_mappings}
        schema_state_keys |= {m.key for m in schema.pose_mappings}
        schema_state_keys |= {k for o in schema.computed_observations for k in o.keys}
        errors.extend(
            f"Observation.tcp({tm.key!r}) has a coordinate_system but no orientation_type: "
            "GR00T's tcp_format applies only to world poses, so declare orientation_type"
            for tm in schema.tcp_mappings
            if tm.coordinate_system and tm.orientation_type is None
        )
        self._server_state_keys = server_state_keys or None
        if self._statistics:
            self._state_ranges = self._ranges_for(server_state_keys)
        missing_state = server_state_keys - schema_state_keys
        if missing_state:
            errors.append(f"Missing state observations: {sorted(missing_state)}")

        server_video_keys = _extract_modality_keys(config, "video")
        schema_image_keys = set(schema.image_sources.keys())
        missing_video = server_video_keys - schema_image_keys
        if missing_video:
            errors.append(f"Missing image observations: {sorted(missing_video)}")

        server_lang_keys = _extract_modality_keys(config, "language")
        if len(server_lang_keys) == 1:
            self._language_key = next(iter(server_lang_keys))
        if server_lang_keys:
            has_language = bool(schema.constants.get("language"))
            if not has_language:
                errors.append(
                    "Server expects language instruction but schema has no "
                    'Observation.constant("language", ...)'
                )

        if errors:
            msg = "Schema does not satisfy GR00T server requirements:\n" + "\n".join(
                f"  - {e}" for e in errors
            )
            raise ValueError(msg)

    def _ranges_for(self, server_state_keys: set[str]) -> dict[str, list[tuple[float, float]]]:
        """Per state key, the trained ``(min, max)`` of each dimension."""
        if self._embodiment is not None:
            if self._embodiment not in self._statistics:
                msg = f"statistics.json has no embodiment {self._embodiment!r} ({sorted(self._statistics)})"
                raise ValueError(msg)
            name = self._embodiment
        else:
            matches = [
                tag
                for tag, entry in self._statistics.items()
                if isinstance(entry, dict) and set(entry.get("state", {})) == server_state_keys
            ]
            if len(matches) != 1:
                msg = (
                    f"statistics.json: {len(matches)} embodiments have exactly the server's "
                    f"state keys {sorted(server_state_keys)}; pass embodiment= ({sorted(self._statistics)})"
                )
                raise ValueError(msg)
            name = matches[0]
        ranges: dict[str, list[tuple[float, float]]] = {}
        for key, stats in self._statistics[name].get("state", {}).items():
            lows, highs = stats.get("min") or [], stats.get("max") or []
            ranges[key] = [(float(lo), float(hi)) for lo, hi in zip(lows, highs, strict=False)]
        return ranges

    async def get_actions(
        self,
        states: dict[str, RobotState],
        schema: PolicySchema,
        images: dict[str, Any] | None = None,
        io_values: dict[str, object] | None = None,
    ) -> ActionChunk:
        """Build GR00T observation from raw states + images, send, decode response."""
        groot_obs = await self._build_groot_obs(states, schema, images, io_values)
        if self._state_ranges and not self._range_checked:
            self._range_checked = True
            state = {k: np.ravel(v).tolist() for k, v in groot_obs["state"].items()}
            training_range.check_vectors(state, self._state_ranges)

        response = await asyncio.to_thread(
            self._transport.call,
            "get_action",
            {"observation": groot_obs, "options": None},
        )

        if not isinstance(response, (list, tuple)) or len(response) != _RESPONSE_PAIR_SIZE:
            msg = (
                "GR00T get_action response must be a 2-tuple of (action, info), "
                f"got {type(response).__name__}. {_PROTOCOL_HINT}"
            )
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

    async def get_modality_config(self) -> dict[str, object]:
        """Fetch raw modality config metadata from the server."""
        response = await asyncio.to_thread(self._transport.call, "get_modality_config")
        return require_dict(response, name="GR00T get_modality_config response")

    async def get_server_info(self) -> dict[str, object]:
        """A summary of the server's configuration.

        Returns:
            ``state_keys``, ``action_keys``, ``video_keys``, ``language_keys``,
            ``action_horizon`` (steps per chunk) and ``action_configs`` (per action key).
        """
        config = await self.get_modality_config()

        state_keys = sorted(_extract_modality_keys(config, "state"))
        action_keys = sorted(_extract_modality_keys(config, "action"))
        video_keys = sorted(_extract_modality_keys(config, "video"))
        language_keys = sorted(_extract_modality_keys(config, "language"))

        action_horizon = 0
        action_configs: list[dict[str, str]] = []
        action_metadata = _modality_metadata(config, "action")
        deltas = action_metadata.get("delta_indices", [])
        if isinstance(deltas, list):
            action_horizon = len(deltas)
        configs = action_metadata.get("action_configs", [])
        if isinstance(configs, list):
            action_configs = [
                {str(key): str(value) for key, value in config.items()}
                for config in configs
                if isinstance(config, dict)
            ]

        return {
            "state_keys": state_keys,
            "action_keys": action_keys,
            "video_keys": video_keys,
            "language_keys": language_keys,
            "action_horizon": action_horizon,
            "action_configs": action_configs,
        }

    # ------------------------------------------------------------------
    # Observation building — reads raw states via schema mappings
    # ------------------------------------------------------------------

    async def _build_groot_obs(
        self,
        states: dict[str, RobotState],
        schema: PolicySchema,
        images: dict[str, Any] | None,
        io_values: dict[str, object] | None,
    ) -> dict[str, Any]:
        """The robot state as GR00T's (1, 1, D) arrays; a plain TCP in metres, ``tcp_format``."""
        groups = schema.observation_groups(await schema.build_observation(states, io_values))
        for tm in schema.tcp_mappings:
            pose = getattr(states.get(tm.source.id), "pose", None)
            if pose is not None and tm.orientation_type is None and not tm.coordinate_system:
                groups[tm.key] = pose_to_eef(pose, self._tcp_format)
        self._actual_dof.update({mg_id: len(s.joints) for mg_id, s in states.items()})
        if self._server_state_keys is not None:
            # An observation the model does not take (a TCP pose that only pins the TCP
            # for relative actions, say) stays out of the request.
            groups = {k: v for k, v in groups.items() if k in self._server_state_keys}
        groot_obs: dict[str, Any] = {"state": {k: _to_state_array(v) for k, v in groups.items()}}

        if images:
            video = _build_video(images)
            if video:
                groot_obs["video"] = video

        language = schema.constants.get("language", "")
        if language:
            groot_obs["language"] = {self._language_key: [[language]]}

        return groot_obs

    # ------------------------------------------------------------------
    # Action decoding
    # ------------------------------------------------------------------

    def _decode_action(
        self,
        schema: PolicySchema,
        action: dict[str, object],
        info: dict[str, object],
    ) -> ActionChunk:
        """Convert GR00T action arrays (``(batch, horizon, dof)``) to an ActionChunk."""
        outputs: dict[str, object] = {}
        for key in [*schema.motion_action_keys, *schema.computed_action_keys]:
            arr = _action_array(action, key)
            if arr is not None:
                outputs[key] = arr[0]
        outputs.update({k: action[k] for k, *_ in schema.io_action_keys if k in action})
        reported = info.get("dt_ms")
        return schema.decode_actions(
            outputs,
            dt_ms=float(cast("float", reported)) if reported is not None else self._dt_ms,
            dof=self._actual_dof,
            source="GR00T",
        )


def _action_array(
    action: dict[str, object], key: str
) -> np.ndarray[Any, np.dtype[np.float32]] | None:
    """The ``(batch, horizon, dof)`` array under ``key``, or ``None`` if the key is absent.

    A present key that holds no such array raises (ADR 0014).
    """
    if key not in action:
        return None
    arr = action[key]
    if isinstance(arr, np.ndarray) and arr.ndim == _ACTION_NDIM - 1:
        arr = arr[None]  # (horizon, dof) without the batch axis
    if isinstance(arr, np.ndarray) and arr.ndim == _ACTION_NDIM:
        return cast("np.ndarray[Any, np.dtype[np.float32]]", arr)
    got = f"{type(arr).__name__} {getattr(arr, 'shape', '')}".strip()
    msg = (
        f"GR00T action '{key}' must be an array of shape ([batch,] horizon, dof), got {got}. "
        f"Response keys: {sorted(action)}. If the server was updated, it may use an array "
        "serialization this client does not decode."
    )
    raise TypeError(msg)


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


def _modality_metadata(config: dict[str, object], modality: str) -> dict[str, object]:
    """Return one modality's required ``as_json`` metadata payload."""
    entry = config.get(modality)
    if entry is None:
        return {}
    entry_dict = require_dict(entry, name=f"GR00T {modality} modality")
    return require_dict(
        entry_dict.get("as_json"),
        name=f"GR00T {modality} modality as_json",
    )


def _extract_modality_keys(config: dict[str, object], modality: str) -> set[str]:
    """Extract ``modality_keys`` from a GR00T modality payload."""
    keys = _modality_metadata(config, modality).get("modality_keys")
    if not isinstance(keys, list):
        return set()
    return {str(key) for key in keys}
