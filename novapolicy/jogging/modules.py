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

# The module, not its names: chunking imports this package, so it may still be loading here.
from novapolicy import chunking
from novapolicy.jogging.pipeline import Timestamps
from novapolicy.joint_limits import DEFAULT_MARGIN_RAD
from novapolicy.types import SequentialExecution

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


@dataclass(frozen=True, slots=True)
class JointClamp:
    """Chunk module: hold every joint target inside the robot's position limits.

    NOVA pauses the motion on an out-of-range waypoint, so a target a policy only slightly
    overshoots would end the run. The session reads the limits from the motion group's
    description when it starts, ``margin_rad`` inside each bound (see
    :mod:`novapolicy.joint_limits`), and each joint's first clip is logged once. TCP waypoints
    pass unchanged: their limit is the workspace, not a per-axis bound. A robot that reports no
    limits is sent unclamped, with a warning.
    """

    margin_rad: float = DEFAULT_MARGIN_RAD

    def on_chunk(  # ruff: ignore[no-self-use] (the protocol's method)
        self, chunk: PendingChunk, context: ModuleContext
    ) -> PendingChunk:
        limits = context.joint_limits
        if limits is None:
            return chunk
        clamped = [limits.clamp(step, context.motion_group_id) for step in chunk.steps]
        return chunk if clamped == chunk.steps else replace(chunk, steps=clamped)


@dataclass(frozen=True, slots=True)
class EndpointRamp:
    """Chunk module: ease into and out of each chunk by subdividing its first and last interval.

    The first interval gets a quadratic ease-in and the last a quadratic ease-out (a
    smoothstep when they are the same), each split into ``interpolation_steps`` intervals of
    the chunk's own spacing, so the added points give the robot real time to accelerate and
    brake. Every original step is kept; IO due at a step stays with it. Meant for sequential
    execution, where each chunk starts and ends at rest; in the executor's default pipeline
    only there.
    """

    interpolation_steps: int = 3
    """Intervals replacing each endpoint interval. Must be at least two."""

    def __post_init__(self) -> None:
        if self.interpolation_steps < 2:  # ruff: ignore[magic-value-comparison] (one interval is no ramp)
            raise ValueError("interpolation_steps must be at least 2")

    def on_chunk(self, chunk: PendingChunk, context: ModuleContext) -> PendingChunk:
        steps, indices = chunking.ramp_steps(
            chunk.steps,
            interpolation_steps=self.interpolation_steps,
            tcp=context.mode == "cartesian",
        )
        if len(steps) == len(chunk.steps):
            return chunk
        when_due = chunk.when_due
        if when_due is not None:
            when_due = (indices[min(when_due[0], len(indices) - 1)], when_due[1])
        return replace(chunk, steps=steps, caller_step_count=len(steps), when_due=when_due)


def default_pipeline(*, execution: object | None = None) -> list[object]:
    """The modules that run when no ``pipeline`` is given, in order.

    Without ``execution``: the joggers' list. With the ``PolicyExecutor``'s execution mode: the
    executor's, which also clamps joint targets to the robot's limits and, in sequential
    execution, eases each chunk in and out.
    """
    if execution is None:
        return [HoldPadding(), Timestamps()]
    ramp = [EndpointRamp()] if isinstance(execution, SequentialExecution) else []
    return [*ramp, JointClamp(), HoldPadding(), Timestamps()]
