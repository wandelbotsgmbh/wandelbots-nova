"""The built-in send-pipeline modules, and the default list.

An explicit ``pipeline=[...]`` is exactly what runs; ``None`` (the default on
:class:`~novapolicy.PolicyExecutor`, the joggers and the session) runs
:func:`default_pipeline`. To keep the defaults and add a module, extend that list::

    pipeline=[LiveBuffer(window_ms=500), *default_pipeline()]

See :mod:`novapolicy.jogging.pipeline` for the order and the contract of each module kind.
"""

from __future__ import annotations

from dataclasses import dataclass, replace
import math
from typing import TYPE_CHECKING

from novapolicy.jogging.pipeline import Timestamps

if TYPE_CHECKING:
    from novapolicy.jogging.pipeline import ModuleContext
    from novapolicy.jogging.waypoints import PendingChunk


@dataclass(frozen=True, slots=True)
class HoldPadding:
    """Chunk module: extend a short chunk by holding its last target up to ``min_horizon_ms``.

    Gives the controller a horizon it can brake within, so a chunk that ends early does not
    end in an abrupt stop. Only chunks queued with ``extend_buffer=True`` are padded: the
    executor's chunks except one-step chunks placed ahead (the next step replaces them before
    they are due, and a hold would make the controller brake towards every step), and the
    jogger's chunks; live targets are not. Padding ends a chunk in a hold, which leaves
    ``WaypointConfig.delay_until_braking_ms`` no velocity to keep: leave this module out to use
    that instead. Put it after the modules that set their own horizon, such as ``LiveBuffer``.
    """

    min_horizon_ms: float = 500.0

    def __post_init__(self) -> None:
        if not math.isfinite(self.min_horizon_ms) or self.min_horizon_ms < 0:
            raise ValueError("HoldPadding needs a non-negative finite min_horizon_ms")

    def on_chunk(
        self,
        chunk: PendingChunk,
        context: ModuleContext,  # ruff: ignore[unused-method-argument] (the protocol's)
    ) -> PendingChunk:
        if not chunk.extend_buffer or self.min_horizon_ms <= 0 or chunk.dt_ms <= 0:
            return chunk
        min_steps = max(1, math.ceil(self.min_horizon_ms / chunk.dt_ms))
        if len(chunk.steps) >= min_steps:
            return chunk
        held = [list(chunk.steps[-1]) for _ in range(min_steps - len(chunk.steps))]
        return replace(chunk, steps=[*chunk.steps, *held])


def default_pipeline() -> list[object]:
    """The modules that run when no ``pipeline`` is given, in order."""
    return [HoldPadding(), Timestamps()]
