""":func:`default_pipeline`: the modules that run when no pipeline is given (contracts T13,
ADR 0010).

An explicit ``pipeline=[...]`` is exactly what runs; ``None`` runs :func:`default_pipeline`.
"""

from __future__ import annotations

from novapolicy.pipeline.base import Timestamps
from novapolicy.pipeline.modules.bridge import Bridge
from novapolicy.pipeline.modules.drop_late import DropLate
from novapolicy.pipeline.modules.endpoint_ramp import EndpointRamp
from novapolicy.pipeline.modules.hold_padding import HoldPadding
from novapolicy.pipeline.modules.joint_clamp import JointClamp
from novapolicy.pipeline.modules.smoothing import Smoothing
from novapolicy.pipeline.modules.timeline import TimestepTimeline
from novapolicy.types import SequentialExecution


def timestep_queue_modules() -> list[object]:
    """A timestep queue's modules: ``Smoothing``, ``TimestepTimeline``, anchored ``Bridge``."""
    return [Smoothing(), TimestepTimeline(), Bridge(always_anchor=True)]


SEQUENTIAL_HOLD_MS = 300.0
"""How long sequential execution holds each chunk's final target after its last step."""


def default_pipeline(
    *, execution: object | None = None, policy: object | None = None
) -> list[object]:
    """The modules that run when no ``pipeline`` is given, in order (contracts T13).

    Without ``execution``: the joggers' list; with the executor's execution mode and policy:
    the executor's for that mode.
    """
    if execution is None:
        return [HoldPadding(), Timestamps(), DropLate()]
    if isinstance(execution, SequentialExecution):
        front: list[object] = [Bridge(), EndpointRamp()]
        hold = HoldPadding(after_ms=SEQUENTIAL_HOLD_MS)
    else:
        front = list(getattr(policy, "default_modules", []))
        hold = HoldPadding()
    return [*front, JointClamp(), hold, Timestamps(), DropLate()]
