"""LeRobot asynchronous-inference queue orchestration."""

from __future__ import annotations

import asyncio
import contextlib
import logging
from typing import TYPE_CHECKING, Any

from novapolicy.action_queue import TimestampedActionQueue
from novapolicy.chunking import smooth_action_chunk
from novapolicy.debug import RawActionChunkTrace
from novapolicy.lerobot.schema import FlatActionLayout

if TYPE_CHECKING:
    from novapolicy.action_queue import AsyncQueueAggregation
    from novapolicy.lerobot.schema import LeRobotSchema
    from novapolicy.lerobot.transport import LeRobotGrpcTransport
    from novapolicy.types import ActionChunk

logger = logging.getLogger(__name__)

_ASYNC_FROZEN_QUEUE_STEPS = 3


class LeRobotAsyncActionQueue:
    """Run asynchronous refills and publish controller-ready lookaheads."""

    def __init__(
        self,
        transport: LeRobotGrpcTransport,
        schema: LeRobotSchema,
        *,
        aggregation: AsyncQueueAggregation,
        refill_threshold: float,
        smoothing: bool,
    ) -> None:
        self._transport = transport
        self._schema = schema
        self._queue = TimestampedActionQueue(
            aggregation=aggregation,
            frozen_steps=_ASYNC_FROZEN_QUEUE_STEPS,
        )
        self._refill_threshold = refill_threshold
        self._smoothing = smoothing
        self._pending_request: asyncio.Task[list[Any]] | None = None
        self._trace: RawActionChunkTrace | None = None

    @property
    def pending_request(self) -> asyncio.Task[list[Any]] | None:
        return self._pending_request

    @property
    def action_queue(self) -> TimestampedActionQueue:
        return self._queue

    @property
    def trajectory_trace(self) -> dict[str, object]:
        return self._trace.data if self._trace is not None else {"raw_action_chunks": []}

    def enable_trajectory_trace(self) -> None:
        self._trace = RawActionChunkTrace()

    def reset(self) -> None:
        self._queue.clear()
        self._pending_request = None
        if self._trace is not None:
            self._trace.clear()

    async def close(self) -> None:
        pending = self._pending_request
        self._pending_request = None
        if pending is not None:
            if not pending.done():
                pending.cancel()
            with contextlib.suppress(asyncio.CancelledError):
                await pending
        self.reset()

    def synchronize(self, timestep: int) -> None:
        skipped = self._queue.synchronize(timestep)
        if skipped:
            logger.info(
                "Synchronized LeRobot queue to NOVA timestep %d (skipped %d actions)",
                timestep,
                skipped,
            )

    async def get_actions(
        self,
        observation: dict[str, Any],
        layout: FlatActionLayout,
    ) -> ActionChunk:
        """Consume one action and return a newly published lookahead when available."""
        queue_updated = await self._merge_completed_request()

        while self._queue.empty:
            if self._pending_request is None:
                self._start_request(observation)
            queue_updated = await self._merge_completed_request(wait=True) or queue_updated

        timestep, current_action = self._queue.consume()

        if self._queue.refill_needed(self._refill_threshold):
            if self._pending_request is None:
                self._start_request(observation)
            else:
                await asyncio.to_thread(
                    self._transport.send_observation,
                    observation,
                    timestep=max(self._queue.latest_timestep, 0),
                    must_go=False,
                )

        preview = self._queue.preview((timestep, current_action))
        if queue_updated:
            predecessor = self._queue.predecessor(timestep)
            action_timestep = timestep
            retained_prefix_steps = 0
            if predecessor is not None:
                preview.insert(0, (timestep - 1, predecessor))
                action_timestep -= 1
                retained_prefix_steps = 1 + _ASYNC_FROZEN_QUEUE_STEPS

            action_chunk = self._schema.decode_arrays(
                [action for _timestep, action in preview],
                layout,
                action_timestep=action_timestep,
                io_action_array=current_action,
            )
            if self._smoothing:
                action_chunk = smooth_action_chunk(
                    action_chunk,
                    retained_prefix_steps=retained_prefix_steps,
                )
                preview = [
                    (
                        entry_timestep,
                        self._schema.replace_joint_values(
                            action,
                            action_chunk,
                            layout,
                            step=index,
                        ),
                    )
                    for index, (entry_timestep, action) in enumerate(preview)
                ]
                action_chunk = self._schema.decode_arrays(
                    [action for _timestep, action in preview],
                    layout,
                    action_timestep=action_timestep,
                    io_action_array=current_action,
                )
            self._queue.publish(preview)
            return action_chunk

        return self._schema.decode_arrays(
            [current_action],
            FlatActionLayout(joints=[], ios=layout.ios),
            action_timestep=timestep,
        )

    def _start_request(self, observation: dict[str, Any]) -> None:
        self._pending_request = asyncio.create_task(
            asyncio.to_thread(
                self._transport.infer,
                observation,
                timestep=max(self._queue.latest_timestep, 0),
                must_go=True,
                allow_empty=True,
            ),
            name="lerobot-async-action-refill",
        )

    async def _merge_completed_request(self, *, wait: bool = False) -> bool:
        task = self._pending_request
        if task is None or (not wait and not task.done()):
            return False
        self._pending_request = None
        actions = await task
        if not actions:
            return False
        decoded = [
            (
                int(timed_action.get_timestep()),
                self._schema.action_to_array(timed_action.get_action()),
            )
            for timed_action in actions
        ]
        if self._trace is not None:
            self._trace.record(decoded)
        return self._queue.merge(decoded)
