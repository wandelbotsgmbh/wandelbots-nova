"""LeRobot asynchronous-inference policy client."""

from __future__ import annotations

import asyncio
from typing import TYPE_CHECKING, Any, cast

from lerobot.async_inference.helpers import RemotePolicyConfig

from novapolicy.action_queue import AsyncQueueAggregation
from novapolicy.lerobot.action_queue import LeRobotAsyncActionQueue
from novapolicy.lerobot.codec import LeRobotCodec
from novapolicy.lerobot.transport import LeRobotGrpcTransport
from novapolicy.policy_client import PolicyClient

if TYPE_CHECKING:
    from lerobot.configs.types import PolicyFeature

    from nova.types import RobotState
    from novapolicy.lerobot.codec import FlatActionLayout
    from novapolicy.schema import PolicySchema
    from novapolicy.types import ActionChunk

_DEFAULT_ASYNC_QUEUE_REFILL_THRESHOLD = 0.75


class LeRobotPolicyClient(PolicyClient):
    """Adapt NOVA observations and actions to LeRobot's trusted gRPC protocol.

    The client supports sequential inference and LeRobot's asynchronous action
    queue. Observation/action ordering is derived from ``PolicySchema``. Flat
    actions contain joint targets followed by optional IO values.

    Args:
        server_address: LeRobot server address in ``host:port`` form.
        pretrained_name_or_path: Model path or Hugging Face model id passed to
            the inference server.
        actions_per_chunk: Number of policy actions requested per inference.
        policy_type: LeRobot policy type, such as ``"act"``.
        fps: Dataset/control frequency used for action timing.
        playback_speed: Physical playback speed relative to the dataset rate.
        device: Torch device used by the inference server.
        timeout_s: Deadline for individual gRPC calls.
        use_async_queue: Use timestamp-aligned asynchronous inference.
        async_queue_aggregation: How predictions for an existing future
            timestep are aggregated.
        async_queue_refill_threshold: Remaining queue fraction that starts an
            asynchronous refill.
        async_queue_smoothing: Smooth only the replaceable lookahead after
            aggregation while retaining the active prefix exactly.

    Note:
        LeRobot's protocol uses pickle and must only be used on trusted networks.
    """

    def __init__(
        self,
        server_address: str,
        pretrained_name_or_path: str,
        *,
        actions_per_chunk: int,
        policy_type: str = "act",
        fps: float = 15.0,
        playback_speed: float = 1.0,
        device: str = "cpu",
        timeout_s: float = 15.0,
        use_async_queue: bool = False,
        async_queue_aggregation: AsyncQueueAggregation = AsyncQueueAggregation.WEIGHTED_AVERAGE,
        async_queue_refill_threshold: float = _DEFAULT_ASYNC_QUEUE_REFILL_THRESHOLD,
        async_queue_smoothing: bool = False,
    ) -> None:
        if fps <= 0:
            raise ValueError(f"fps must be positive, got {fps}")
        if playback_speed <= 0:
            raise ValueError(f"playback_speed must be positive, got {playback_speed}")
        if actions_per_chunk <= 0:
            raise ValueError(f"actions_per_chunk must be positive, got {actions_per_chunk}")
        if not 0 < async_queue_refill_threshold <= 1:
            msg = (
                "async_queue_refill_threshold must be in (0, 1], "
                f"got {async_queue_refill_threshold}"
            )
            raise ValueError(msg)

        self._pretrained_name_or_path = pretrained_name_or_path
        self._policy_type = policy_type
        self._actions_per_chunk = actions_per_chunk
        self._device = device
        self._dt_ms = 1000.0 / (fps * playback_speed)
        self._transport = LeRobotGrpcTransport(server_address, timeout_s=timeout_s)
        self._codec = LeRobotCodec(dt_ms=self._dt_ms)
        self._async_queue = (
            LeRobotAsyncActionQueue(
                self._transport,
                self._codec,
                aggregation=async_queue_aggregation,
                refill_threshold=async_queue_refill_threshold,
                smoothing=async_queue_smoothing,
            )
            if use_async_queue
            else None
        )
        self._setup_sent = False
        self._timestep = 0

    @property
    def dt_ms(self) -> float:
        """Physical action timestep in milliseconds."""
        return self._dt_ms

    @property
    def trajectory_trace(self) -> dict[str, object]:
        """Raw pre-aggregation predictions when debug tracing is enabled."""
        if self._async_queue is None:
            return {"raw_action_chunks": []}
        return self._async_queue.trajectory_trace

    def enable_trajectory_trace(self) -> None:
        """Enable raw asynchronous prediction tracing for the next episode."""
        if self._async_queue is not None:
            self._async_queue.enable_trajectory_trace()

    @property
    def requires_first_waypoint_bridge(self) -> bool:
        """Whether continuous execution needs one measured-state bridge."""
        return self._async_queue is not None

    def synchronize_action_timestep(self, timestep: int) -> None:
        """Drop queue actions whose NOVA execution timestamps have elapsed."""
        if self._async_queue is not None:
            self._async_queue.synchronize(timestep)

    async def connect(self, motion_group_ids: list[str]) -> None:  # noqa: ARG002
        """Open the gRPC channel and reset episode state."""
        await asyncio.to_thread(self._transport.connect)
        self._setup_sent = False
        self._timestep = 0
        if self._async_queue is not None:
            self._async_queue.reset()

    async def validate_schema(self, schema: PolicySchema) -> None:
        """Validate schema constraints known without remote model metadata."""
        self._codec.validate_schema(schema)

    async def prepare(
        self,
        states: dict[str, RobotState],
        schema: PolicySchema,
        images: dict[str, Any] | None = None,
        io_values: dict[str, object] | None = None,  # noqa: ARG002
    ) -> None:
        """Send policy setup before the executor timeout starts."""
        if not self._transport.connected:
            await self.connect([])
        state_names, _layout = self._schema_layout(states, schema)
        await asyncio.to_thread(self._ensure_policy_setup, schema, state_names, images)

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

        observation = await self._codec.build_observation(states, schema, images, io_values)
        state_names, layout = self._schema_layout(states, schema)
        await asyncio.to_thread(self._ensure_policy_setup, schema, state_names, images)

        if self._async_queue is not None:
            return await self._async_queue.get_actions(observation, layout)

        actions = await asyncio.to_thread(
            self._transport.infer,
            observation,
            timestep=self._timestep,
            must_go=True,
        )
        self._timestep += len(actions)
        return self._codec.decode_timed_actions(actions, layout)

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
        state_names = self._codec.state_names(states, schema)
        layout = self._codec.action_layout(states, schema)
        return state_names, layout

    def _ensure_policy_setup(
        self,
        schema: PolicySchema,
        state_names: list[str],
        images: dict[str, Any] | None,
    ) -> None:
        if self._setup_sent:
            return
        self._transport.configure_policy(
            RemotePolicyConfig(
                policy_type=self._policy_type,
                pretrained_name_or_path=self._pretrained_name_or_path,
                lerobot_features=cast(
                    "dict[str, PolicyFeature]",
                    self._codec.features(schema, state_names, images),
                ),
                actions_per_chunk=self._actions_per_chunk,
                device=self._device,
            )
        )
        self._setup_sent = True


__all__ = ["AsyncQueueAggregation", "LeRobotPolicyClient"]
