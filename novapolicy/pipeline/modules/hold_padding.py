"""``HoldPadding``: hold a chunk's last target to a minimum horizon (contract T6)."""

from __future__ import annotations

from dataclasses import dataclass, replace
import math
from typing import TYPE_CHECKING, ClassVar

if TYPE_CHECKING:
    from novapolicy.pipeline.base import ModuleContext
    from novapolicy.transport.action_chunks import PendingChunk


@dataclass(frozen=True, slots=True)
class HoldPadding:
    """Chunk module: hold a chunk's last target until it spans ``min_horizon_ms``, and at
    least ``after_ms`` past its last step.

    Pads only chunks queued with ``extend_buffer=True`` (contract T6). ``after_ms`` lets an
    arm still moving at the last step settle on it: NOVA brakes after the last waypoint and
    does not return to it (ADR 0010). Leave it out to use
    ``ActionChunkConfig.delay_until_braking_ms``; put it after modules that set their own horizon.
    """

    stage: ClassVar[str] = "horizon"  # its place among the chunk modules (CHUNK_STAGES)

    min_horizon_ms: float = 500.0
    after_ms: float = 0.0

    def __post_init__(self) -> None:
        for name in ("min_horizon_ms", "after_ms"):
            value = getattr(self, name)
            if not math.isfinite(value) or value < 0:
                raise ValueError(f"HoldPadding needs a non-negative finite {name}")

    def on_chunk(
        self,
        chunk: PendingChunk,
        context: ModuleContext,  # ruff: ignore[unused-method-argument] (the protocol's)
    ) -> PendingChunk:
        if not chunk.extend_buffer or chunk.dt_ms <= 0:
            return chunk
        min_steps = max(
            math.ceil(self.min_horizon_ms / chunk.dt_ms),
            len(chunk.steps) + math.ceil(self.after_ms / chunk.dt_ms),
        )
        if len(chunk.steps) >= min_steps:
            return chunk
        held = [list(chunk.steps[-1]) for _ in range(min_steps - len(chunk.steps))]
        return replace(chunk, steps=[*chunk.steps, *held])
