"""The built-in send-pipeline modules and :func:`default_pipeline` (contracts T13, ADR 0010).

An explicit ``pipeline=[...]`` is exactly what runs; ``None`` runs :func:`default_pipeline`.
"""

from __future__ import annotations

from dataclasses import dataclass, field, replace
import math
from typing import TYPE_CHECKING

# The module, not its names: chunking imports this package, so it may still be loading here.
from novapolicy import chunking
from novapolicy.jogging.clock import ServerMs
from novapolicy.jogging.pipeline import Timestamps
from novapolicy.joint_limits import DEFAULT_MARGIN_RAD
from novapolicy.types import SequentialExecution

if TYPE_CHECKING:
    from novapolicy.jogging.pipeline import ModuleContext, TimedWaypoint
    from novapolicy.jogging.waypoints import PendingChunk


@dataclass(frozen=True, slots=True)
class HoldPadding:
    """Chunk module: extend a short chunk by holding its last target up to ``min_horizon_ms``.

    Pads only chunks queued with ``extend_buffer=True`` (contract T6). Leave it out to use
    ``WaypointConfig.delay_until_braking_ms``; put it after modules that set their own horizon.
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
    """Chunk module: hold every joint target ``margin_rad`` inside the robot's limits (M7).

    TCP waypoints pass unchanged; a robot that reports no limits is sent unclamped, with a
    warning.
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

    Each endpoint interval becomes ``interpolation_steps`` intervals of the chunk's spacing,
    quadratic ease-in / ease-out (a smoothstep when they coincide). Every original step and its
    IO is kept. For sequential execution, where chunks start and end at rest.
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
        return replace(
            chunk,
            steps=steps,
            caller_step_count=len(steps),
            when_due=when_due,
            start_step=indices[min(chunk.start_step, len(indices) - 1)],
        )


@dataclass
class Smoothing:
    """Chunk module: smooth a plan's motion with ``passes`` of the filter ``[1, 2, 1] / 4``.

    Steps already sent (``retained_prefix_steps``) keep the values sent for their timesteps.
    Joint and TCP values are filtered per component; IO, timing and computed-action values are
    untouched; chunks under three steps pass unchanged. Put it before the ``Bridge``.
    """

    passes: int = 2
    _sent: dict[int, list[float]] = field(default_factory=dict, init=False, repr=False)
    """The values last sent, by policy timestep."""

    def __post_init__(self) -> None:
        if self.passes < 0:
            raise ValueError("Smoothing needs a non-negative number of passes")

    def on_chunk(
        self,
        chunk: PendingChunk,
        context: ModuleContext,  # ruff: ignore[unused-method-argument] (the protocol's)
    ) -> PendingChunk:
        if self.passes == 0:
            return chunk
        retained = min(chunk.retained_prefix_steps, len(chunk.steps))
        first = chunk.action_timestep
        steps = [list(step) for step in chunk.steps]
        if first >= 0:
            for i in range(retained):
                sent = self._sent.get(first + i)
                if sent is not None and len(sent) == len(steps[i]):
                    steps[i] = list(sent)
        smoothed = chunking.smooth_steps(steps, passes=self.passes, retained_prefix_steps=retained)
        if first >= 0:
            self._sent = {first + i: step for i, step in enumerate(smoothed)}
        return replace(chunk, steps=smoothed)


@dataclass(frozen=True, slots=True)
class Bridge:
    """Chunk module: bridge from the measured position to the policy's first step.

    A hold on the measured position, then steps at the chunk's spacing, subdivided as far as its
    largest step needs; ``always_anchor`` adds the hold even without subdivision. Chunks with
    ``first_timestamp_ms`` or already at the robot pass unchanged. In continuous execution a
    bridged chunk is dated at the send (T2). IO stays with the policy's first step; TCP
    rotations are unwrapped against the measured pose.
    """

    always_anchor: bool = False

    def on_chunk(self, chunk: PendingChunk, context: ModuleContext) -> PendingChunk:
        if chunk.first_timestamp_ms is not None or context.position is None:
            return chunk
        tcp = context.mode == "cartesian"
        steps = (
            chunking.unwrap_tcp_rotations(chunk.steps, context.position[3:6])
            if tcp
            else chunk.steps
        )
        bridge = chunking.bridge_steps(
            context.position, steps, tcp=tcp, always_anchor=self.always_anchor
        )
        if not bridge:
            return chunk
        shift = len(bridge) - 1
        when_due = chunk.when_due
        if when_due is not None:
            when_due = (when_due[0] + shift, when_due[1])
        bridged = replace(
            chunk,
            steps=[*bridge, *steps[1:]],
            caller_step_count=len(bridge) + len(steps) - 1,
            when_due=when_due,
            start_step=chunk.start_step + shift,
        )
        if chunk.lead and chunk.observed_at_ms is not None:
            # Dated by its observation (continuous execution): the bridge starts now instead.
            bridged = replace(bridged, lead=False, observed_at_ms=None, timestamp_offset_steps=0)
        return bridged


@dataclass
class TimestepTimeline:
    """Chunk module: keep a timestep queue's chunks on one timeline (contract T2).

    The first chunk is placed like any other; once sent, the time its step ``t0`` is due is the
    origin, and a later chunk starting at timestep ``t`` is placed at
    ``origin + (t - t0) * dt``. Chunks without timesteps pass. Put it before the ``Bridge``.
    """

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


MIN_LEAD_MS = 20.0
"""Smallest lead, in ms, a waypoint may still be sent with (contract T3)."""


@dataclass(frozen=True, slots=True)
class DropLate:
    """Filter: drop the waypoints due within ``min_lead_ms`` of now; the rest keep their times.

    Contract T3, ADR 0004. A chunk stamped at the send passes whole; when every waypoint is late,
    nothing is sent.
    """

    min_lead_ms: float = MIN_LEAD_MS

    def keep(
        self, waypoints: list[TimedWaypoint], chunk: PendingChunk, context: ModuleContext
    ) -> list[TimedWaypoint]:
        if not chunk.lead and chunk.first_timestamp_ms is None:
            return waypoints
        earliest_ms = context.now_ms + self.min_lead_ms
        return [(t, v) for t, v in waypoints if t >= earliest_ms]


def timestep_queue_modules() -> list[object]:
    """A timestep queue's modules: ``Smoothing``, ``TimestepTimeline``, anchored ``Bridge``."""
    return [Smoothing(), TimestepTimeline(), Bridge(always_anchor=True)]


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
    else:
        front = list(getattr(policy, "default_modules", []))
    return [*front, JointClamp(), HoldPadding(), Timestamps(), DropLate()]
