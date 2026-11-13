"""Continuous execution for policies that answer each observation with a fresh chunk.

A policy that predicts a whole chunk per observation (GR00T, a callback, ...) has no
notion of the chunk NOVA is executing. Replacing that lookahead with each new chunk
makes the arm jump at every seam. :class:`QueuedPolicyClient` wraps such a client and
keeps one plan on the executor's timestep timeline, as the LeRobot async queue does:

- each chunk is resolved to absolute NOVA targets (:func:`~novapolicy.resolve_actions`),
- the steps that executed while it was computed, plus a margin, stay exactly as published,
- later overlapping steps are blended old/new (0.3/0.7),
- the plan is smoothed with the frozen head retained, and handed to the executor with its
  first absolute timestep.

Use it with ``ContinuousExecution()`` and ``PolicyExecutor(..., n_action_steps=0)``.
"""

from __future__ import annotations

import math
import time
from typing import TYPE_CHECKING, Any

import numpy as np

from novapolicy.action_queue import TimestampedActionQueue
from novapolicy.chunking import smooth_action_chunk, unwrap_tcp_rotations
from novapolicy.executor import resolve_actions
from novapolicy.policy_client import PolicyClient
from novapolicy.types import ActionChunk

if TYPE_CHECKING:
    from numpy.typing import NDArray

    from nova.types import RobotState
    from novapolicy.schema import PolicySchema

_TCP_DIM = 6


class QueuedPolicyClient(PolicyClient):
    """Merge a chunk-per-observation policy's chunks into one continuous plan.

    Args:
        policy: The wrapped client.
        margin_steps: Steps kept frozen beyond those that executed during inference.
        smoothing_passes: ``[1, 2, 1] / 4`` passes over the merged plan; 0 disables.
    """

    def __init__(
        self, policy: PolicyClient, *, margin_steps: int = 2, smoothing_passes: int = 2
    ) -> None:
        self._policy = policy
        self._margin_steps = margin_steps
        self._smoothing_passes = smoothing_passes
        self._queue = TimestampedActionQueue()
        self._replaceable = 0
        self._layout: list[tuple[str, str, int]] = []  # (kind, group id, width)

    @property
    def policy(self) -> PolicyClient:
        return self._policy

    @property
    def requires_first_waypoint_bridge(self) -> bool:
        return True  # chunks carry absolute timesteps; only the first one is bridged

    def synchronize_action_timestep(self, timestep: int) -> None:
        self._replaceable = timestep

    async def connect(self, motion_group_ids: list[str]) -> None:
        self._queue.clear()
        self._replaceable = 0
        self._layout = []
        await self._policy.connect(motion_group_ids)

    async def validate_schema(self, schema: PolicySchema) -> None:
        await self._policy.validate_schema(schema)

    async def prepare(
        self,
        states: dict[str, RobotState],
        schema: PolicySchema,
        images: dict[str, NDArray[Any]] | None = None,
        io_values: dict[str, object] | None = None,
    ) -> None:
        await self._policy.prepare(states, schema, images, io_values)

    async def close(self) -> None:
        await self._policy.close()

    async def get_actions(
        self,
        states: dict[str, RobotState],
        schema: PolicySchema,
        images: dict[str, NDArray[Any]] | None = None,
        io_values: dict[str, object] | None = None,
    ) -> ActionChunk:
        t0 = time.monotonic()
        raw = await self._policy.get_actions(states, schema, images, io_values)
        latency_ms = (time.monotonic() - t0) * 1000.0
        chunk = resolve_actions(schema, raw, states)
        if chunk.dt_ms <= 0:
            msg = "QueuedPolicyClient needs the wrapped policy's chunks to carry dt_ms"
            raise ValueError(msg)
        first = self._replaceable
        self._queue.synchronize(first)
        predecessor = self._queue.predecessor(first)
        steps = self._flatten(chunk, predecessor)
        frozen = math.ceil(latency_ms / chunk.dt_ms) + self._margin_steps
        self._queue.merge(((first + i, step) for i, step in enumerate(steps)), frozen_steps=frozen)
        plan = self._queue.actions
        retained = 0  # nothing published yet: nothing to keep exactly
        if predecessor is not None:
            plan.insert(0, (first - 1, predecessor))
            retained = 1 + min(frozen, len(plan) - 1)
        merged = self._unflatten([v for _ts, v in plan], chunk, plan[0][0])
        if self._smoothing_passes > 0:
            merged = smooth_action_chunk(
                merged, passes=self._smoothing_passes, retained_prefix_steps=retained
            ).model_copy(update={"resolved": True})  # the filter rebuilds the chunk
        self._queue.publish(zip([ts for ts, _v in plan], self._flatten(merged, None), strict=True))
        return merged

    def _flatten(
        self, chunk: ActionChunk, predecessor: NDArray[np.float32] | None
    ) -> list[NDArray[np.float32]]:
        """One vector per step: every joint group, then every TCP group (unwrapped)."""
        if not self._layout:
            self._layout = [("j", g, len(s[0])) for g, s in sorted(chunk.joints.items()) if s]
            self._layout += [("t", g, _TCP_DIM) for g, s in sorted(chunk.tcp.items()) if s]
        columns: list[list[list[float]]] = []
        offset = 0
        for kind, group, width in self._layout:
            steps = (chunk.joints if kind == "j" else chunk.tcp)[group]
            if kind == "t" and predecessor is not None:
                ref = predecessor[offset : offset + width].tolist()
                steps = unwrap_tcp_rotations(steps, ref[3:6]) if steps else steps
            columns.append(steps)
            offset += width
        n = min(len(c) for c in columns)
        return [
            np.asarray([v for c in columns for v in c[i]], dtype=np.float64).astype(np.float32)
            for i in range(n)
        ]

    def _unflatten(
        self, vectors: list[NDArray[np.float32]], template: ActionChunk, first: int
    ) -> ActionChunk:
        joints: dict[str, list[list[float]]] = {}
        tcp: dict[str, list[list[float]]] = {}
        offset = 0
        for kind, group, width in self._layout:
            target = joints if kind == "j" else tcp
            target[group] = [v[offset : offset + width].astype(float).tolist() for v in vectors]
            offset += width
        return ActionChunk(
            joints=joints,
            tcp=tcp,
            ios=template.ios,
            dt_ms=template.dt_ms,
            action_timestep=first,
            resolved=True,
        )
