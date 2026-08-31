"""LeRobot asynchronous-inference policy client."""

from __future__ import annotations

import asyncio
import logging
from typing import TYPE_CHECKING, Any, cast

from lerobot.async_inference.helpers import RemotePolicyConfig

from novapolicy.action_queue import AsyncQueueAggregation
from novapolicy.lerobot.action_queue import LeRobotAsyncActionQueue
from novapolicy.lerobot.config import try_load_execution_settings
from novapolicy.lerobot.schema import LeRobotSchema, check_observation_range
from novapolicy.lerobot.transport import LeRobotGrpcTransport
from novapolicy.policy_client import PolicyClient

if TYPE_CHECKING:
    from pathlib import Path

    from nova.types import RobotState
    from novapolicy.lerobot.config import LeRobotExecutionSettings
    from novapolicy.lerobot.schema import FlatActionLayout
    from novapolicy.schema import PolicySchema
    from novapolicy.types import ActionChunk

logger = logging.getLogger(__name__)

_DEFAULT_ASYNC_QUEUE_REFILL_THRESHOLD = 0.75


class LeRobotPolicyClient(PolicyClient):
    """Adapt NOVA observations and actions to LeRobot's trusted gRPC protocol.

    The client supports sequential inference and LeRobot's asynchronous action
    queue. Observation/action ordering is derived from ``PolicySchema``. Flat
    actions contain joint targets, then TCP targets, then optional IO values.

    Settings the checkpoint already declares are read from it rather than asked
    for: ``policy_type``, ``actions_per_chunk`` and ``n_action_steps``. Pass one
    explicitly only to override it — an override that contradicts the checkpoint
    is reported rather than silently accepted. ``fps`` cannot be derived: frame
    rate is a property of the dataset, not the policy. Camera frame size is not
    derived either; a mismatch is reported and left to the camera stream.

    Derivation needs a checkpoint the *client* can read. When
    ``pretrained_name_or_path`` is a server-local absolute path, pass
    ``config_path`` pointing at a client-local copy of the checkpoint directory
    or its ``config.json``.

    Args:
        server_address: LeRobot server address in ``host:port`` form.
        pretrained_name_or_path: Model path or Hugging Face model id passed to
            the inference server.
        config_path: Client-readable checkpoint directory or ``config.json`` to
            derive settings from. Defaults to ``pretrained_name_or_path``.
        actions_per_chunk: Override for the actions requested per inference.
            Defaults to the checkpoint's ``chunk_size``.
        n_action_steps: Override for the actions executed per chunk. Defaults to
            the checkpoint's ``n_action_steps``; the executor reads it from here.
        policy_type: Override for the LeRobot policy type, such as ``"act"``.
        fps: Dataset/control frequency used for action timing.
        playback_speed: Physical playback speed relative to the dataset rate.
        device: Torch device used by the inference server.
        timeout_s: Deadline for individual gRPC calls.
        use_async_queue: Use timestamp-aligned asynchronous inference.
        async_queue_aggregation: How predictions for an existing future
            timestep are aggregated. Defaults to ``AVERAGE``: on both cells
            measured so far (UR3 plug task, UR10e pick-and-place) it showed
            lower peak path curvature than LeRobot's own weighted average.
        async_queue_refill_threshold: Remaining queue fraction that starts an
            asynchronous refill.
        async_queue_smoothing: Smooth only the replaceable lookahead after
            aggregation while retaining the active prefix exactly. On by
            default: it removes the waypoint-scale kinks aggregation seams
            leave, at the cost of rounding intended corners by about two
            waypoints - pass ``False`` for tasks that need sharp contact
            transitions.

    Note:
        LeRobot's protocol uses pickle and must only be used on trusted networks.
    """

    def __init__(
        self,
        server_address: str,
        pretrained_name_or_path: str,
        *,
        config_path: str | Path | None = None,
        actions_per_chunk: int | None = None,
        n_action_steps: int | None = None,
        policy_type: str | None = None,
        fps: float = 15.0,
        playback_speed: float = 1.0,
        device: str = "cpu",
        timeout_s: float = 15.0,
        use_async_queue: bool = False,
        async_queue_aggregation: AsyncQueueAggregation = AsyncQueueAggregation.AVERAGE,
        async_queue_refill_threshold: float = _DEFAULT_ASYNC_QUEUE_REFILL_THRESHOLD,
        async_queue_smoothing: bool = True,
    ) -> None:
        if fps <= 0:
            raise ValueError(f"fps must be positive, got {fps}")
        if playback_speed <= 0:
            raise ValueError(f"playback_speed must be positive, got {playback_speed}")
        if actions_per_chunk is not None and actions_per_chunk <= 0:
            raise ValueError(f"actions_per_chunk must be positive, got {actions_per_chunk}")
        if n_action_steps is not None and n_action_steps <= 0:
            raise ValueError(f"n_action_steps must be positive, got {n_action_steps}")
        if not 0 < async_queue_refill_threshold <= 1:
            msg = (
                "async_queue_refill_threshold must be in (0, 1], "
                f"got {async_queue_refill_threshold}"
            )
            raise ValueError(msg)

        self._pretrained_name_or_path = pretrained_name_or_path
        self._config_path = config_path if config_path is not None else pretrained_name_or_path
        self._explicit_policy_type = policy_type
        self._explicit_actions_per_chunk = actions_per_chunk
        self._explicit_n_action_steps = n_action_steps
        self._settings: LeRobotExecutionSettings | None = None
        self._settings_resolved = False
        self._policy_type = policy_type or ""
        self._actions_per_chunk = actions_per_chunk or 0
        self._n_action_steps = n_action_steps
        self._device = device
        self._dt_ms = 1000.0 / (fps * playback_speed)
        self._transport = LeRobotGrpcTransport(server_address, timeout_s=timeout_s)
        self._lerobot_schema = LeRobotSchema(dt_ms=self._dt_ms)
        self._async_queue = (
            LeRobotAsyncActionQueue(
                self._transport,
                self._lerobot_schema,
                aggregation=async_queue_aggregation,
                refill_threshold=async_queue_refill_threshold,
                smoothing=async_queue_smoothing,
            )
            if use_async_queue
            else None
        )
        self._setup_sent = False
        self._timestep = 0
        self._range_checked = False

    @property
    def dt_ms(self) -> float:
        """Physical action timestep in milliseconds."""
        return self._dt_ms

    @property
    def requires_first_waypoint_bridge(self) -> bool:
        """Whether continuous execution needs one measured-state bridge."""
        return self._async_queue is not None

    def synchronize_action_timestep(self, timestep: int) -> None:
        """Drop queue actions whose NOVA execution timestamps have elapsed."""
        if self._async_queue is not None:
            self._async_queue.synchronize(timestep)

    @property
    def n_action_steps(self) -> int | None:
        """Actions to execute per chunk, from the checkpoint or an override."""
        return self._n_action_steps

    async def connect(self, motion_group_ids: list[str]) -> None:  # ruff: ignore[unused-method-argument]
        """Read the checkpoint, open the gRPC channel, and reset episode state."""
        await asyncio.to_thread(self._resolve_settings)
        await asyncio.to_thread(self._transport.connect)
        self._setup_sent = False
        self._range_checked = False
        self._timestep = 0
        if self._async_queue is not None:
            self._async_queue.reset()

    async def validate_schema(self, schema: PolicySchema) -> None:
        """Validate schema constraints known without remote model metadata."""
        self._lerobot_schema.validate_schema(schema)

    def _resolve_settings(self) -> None:
        """Derive execution settings from the checkpoint, reconciling overrides.

        Blocking: reads the checkpoint config, which may download from the Hub.
        Called from a worker thread, and only once — ``connect`` may run again
        on a reconnect, and re-reading a Hub checkpoint each time would be
        wasted work.
        """
        if self._settings_resolved:
            return

        settings = try_load_execution_settings(self._config_path)
        if settings is None:
            self._settings = None
            missing = [
                name
                for name, value in (
                    ("policy_type", self._explicit_policy_type),
                    ("actions_per_chunk", self._explicit_actions_per_chunk),
                )
                if value is None
            ]
            if missing:
                msg = (
                    "LeRobotPolicyClient cannot read the checkpoint at "
                    f"{self._config_path!r}, so {' and '.join(missing)} cannot be derived. "
                    f"Pass {' and '.join(missing)} explicitly, or set config_path to a "
                    "client-readable checkpoint directory."
                )
                raise ValueError(msg)
            logger.warning(
                "LeRobot checkpoint %r is not readable by the client: using explicit "
                "settings and skipping validation against the checkpoint.",
                self._config_path,
            )
            self._settings_resolved = True
            return

        # Reconcile before storing, so a contradicting override leaves the
        # client unresolved and raises again rather than running on half-applied
        # settings.
        policy_type = self._resolved_policy_type(settings)
        actions_per_chunk = self._resolved_actions_per_chunk(settings)
        n_action_steps = self._resolved_n_action_steps(settings)

        self._settings = settings
        self._policy_type = policy_type
        self._actions_per_chunk = actions_per_chunk
        self._n_action_steps = n_action_steps
        self._settings_resolved = True

    def _resolved_policy_type(self, settings: LeRobotExecutionSettings) -> str:
        explicit = self._explicit_policy_type
        if explicit is None:
            return settings.policy_type
        if explicit != settings.policy_type:
            msg = (
                f"policy_type={explicit!r} contradicts the checkpoint, which is "
                f"{settings.policy_type!r}. Remove the argument to use the checkpoint's type."
            )
            raise ValueError(msg)
        return explicit

    def _resolved_actions_per_chunk(self, settings: LeRobotExecutionSettings) -> int:
        """Actions to request per inference: the checkpoint's chunk, or an override."""
        explicit = self._explicit_actions_per_chunk
        chunk_size = settings.chunk_size

        if chunk_size is None:
            if explicit is None:
                msg = (
                    f"The {settings.policy_type!r} checkpoint declares no action chunk, "
                    "so actions_per_chunk cannot be derived. Pass it explicitly."
                )
                raise ValueError(msg)
            return explicit

        if explicit is None:
            return chunk_size
        if explicit > chunk_size:
            msg = (
                f"actions_per_chunk={explicit} exceeds the checkpoint's action chunk of "
                f"{chunk_size}; the policy cannot predict that many actions."
            )
            raise ValueError(msg)
        if explicit < chunk_size:
            logger.warning(
                "actions_per_chunk=%d is below the checkpoint's action chunk of %d; "
                "requesting a shortened chunk.",
                explicit,
                chunk_size,
            )
        return explicit

    def _resolved_n_action_steps(self, settings: LeRobotExecutionSettings) -> int | None:
        """Actions to execute per chunk: the checkpoint's horizon, or an override."""
        explicit = self._explicit_n_action_steps
        chunk_size = settings.chunk_size

        if explicit is None:
            return settings.n_action_steps
        if chunk_size is not None and explicit > chunk_size:
            msg = (
                f"n_action_steps={explicit} exceeds the checkpoint's action chunk of "
                f"{chunk_size}; there are not that many actions in a chunk."
            )
            raise ValueError(msg)
        if settings.n_action_steps is not None and explicit != settings.n_action_steps:
            logger.warning(
                "n_action_steps=%d differs from the checkpoint's %d; executing a "
                "different horizon than the checkpoint intends.",
                explicit,
                settings.n_action_steps,
            )
        return explicit

    async def prepare(
        self,
        states: dict[str, RobotState],
        schema: PolicySchema,
        images: dict[str, Any] | None = None,
        io_values: dict[str, object] | None = None,  # ruff: ignore[unused-method-argument]
    ) -> None:
        """Send policy setup before the executor timeout starts."""
        if not self._transport.connected:
            await self.connect([])
        state_names, layout = self._schema_layout(states, schema)
        await asyncio.to_thread(self._ensure_policy_setup, schema, state_names, images, layout)

    async def get_actions(
        self,
        states: dict[str, RobotState],
        schema: PolicySchema,
        images: dict[str, Any] | None = None,
        io_values: dict[str, object] | None = None,
    ) -> ActionChunk:
        """Send one observation and return a decoded action chunk."""
        if not self._transport.connected:
            await self.connect([])

        observation = await self._lerobot_schema.build_observation(
            states, schema, images, io_values
        )
        state_names, layout = self._schema_layout(states, schema)
        self._check_observation_range(observation, state_names)
        await asyncio.to_thread(self._ensure_policy_setup, schema, state_names, images, layout)

        if self._async_queue is not None:
            return await self._async_queue.get_actions(observation, layout)

        actions = await asyncio.to_thread(
            self._transport.infer,
            observation,
            timestep=self._timestep,
            must_go=True,
        )
        self._timestep += len(actions)
        return self._lerobot_schema.decode_timed_actions(actions, layout)

    async def close(self) -> None:
        """Cancel pending inference and close the gRPC channel."""
        if self._async_queue is not None:
            await self._async_queue.close()
        await asyncio.to_thread(self._transport.close)
        self._setup_sent = False

    def _schema_layout(
        self,
        states: dict[str, RobotState],
        schema: PolicySchema,
    ) -> tuple[list[str], FlatActionLayout]:
        state_names = self._lerobot_schema.state_names(states, schema)
        layout = self._lerobot_schema.action_layout(states, schema)
        return state_names, layout

    def _check_observation_range(
        self,
        observation: dict[str, Any],
        state_names: list[str],
    ) -> None:
        """Hold the first live observation against the checkpoint's statistics.

        Once per run: the point is to catch a units or ordering mistake before
        the robot moves, not to police every tick.
        """
        if self._range_checked or self._settings is None:
            return
        self._range_checked = True
        check_observation_range(self._settings, observation, state_names)

    def _ensure_policy_setup(
        self,
        schema: PolicySchema,
        state_names: list[str],
        images: dict[str, Any] | None,
        layout: FlatActionLayout,
    ) -> None:
        if self._setup_sent:
            return
        if self._settings is not None:
            self._lerobot_schema.assert_matches(self._settings, schema, state_names, images, layout)
        self._transport.configure_policy(
            RemotePolicyConfig(
                policy_type=self._policy_type,
                pretrained_name_or_path=self._pretrained_name_or_path,
                lerobot_features=cast(
                    "dict[str, Any]",
                    self._lerobot_schema.features(schema, state_names, images),
                ),
                actions_per_chunk=self._actions_per_chunk,
                device=self._device,
            )
        )
        self._setup_sent = True


__all__ = ["AsyncQueueAggregation", "LeRobotPolicyClient"]
