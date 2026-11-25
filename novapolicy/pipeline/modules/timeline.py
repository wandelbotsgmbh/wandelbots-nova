"""``TimestepTimeline``: keep a timestep queue's chunks on one timeline (contract T2)."""

from __future__ import annotations

from dataclasses import dataclass, field, replace
from typing import TYPE_CHECKING, ClassVar

from novapolicy.transport.clock import ServerMs

if TYPE_CHECKING:
    from novapolicy.pipeline.base import ModuleContext
    from novapolicy.transport.action_chunks import PendingChunk


@dataclass
class TimestepTimeline:
    """Chunk module: keep a timestep queue's chunks on one timeline (contract T2).

    The first chunk is placed like any other; once sent, the time its step ``t0`` is due is the
    origin, and a later chunk starting at timestep ``t`` is placed at
    ``origin + (t - t0) * dt``. Chunks without timesteps pass. Put it before the ``Bridge``.
    """

    stage: ClassVar[str] = "timeline"  # its place among the chunk modules (CHUNK_STAGES)

    _origin: tuple[int, int] | None = field(default=None, init=False, repr=False)

    def on_chunk(self, chunk: PendingChunk, context: ModuleContext) -> PendingChunk:
        if chunk.action_timestep < 0:
            return chunk
        if self._origin is None and context.scheduled_start is not None:
            self._origin = context.scheduled_start
        if self._origin is None:
            return chunk  # the first chunk: once sent, it sets the origin
        timestep, origin_ms = self._origin
        first_ms = int(origin_ms + (chunk.action_timestep - timestep) * chunk.dt_ms)
        return replace(
            chunk,
            first_timestamp_ms=ServerMs(first_ms),
            timestamp_offset_steps=0,
            lead=False,
            observed_at_ms=None,
        )
