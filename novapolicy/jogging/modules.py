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
    from novapolicy.jogging.pipeline import ModuleContext, TimedWaypoint
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
        return replace(
            chunk,
            steps=steps,
            caller_step_count=len(steps),
            when_due=when_due,
            start_step=indices[min(chunk.start_step, len(indices) - 1)],
        )


@dataclass(frozen=True, slots=True)
class Bridge:
    """Chunk module: move from where the robot is to the policy's first step at the chunk's
    own spacing, instead of jumping there in one step.

    The bridge starts with a hold on the measured position, so sending the request does not use
    up the first interval, and ends exactly on the policy's first step; it is subdivided as far
    as the chunk's largest step needs (contract T4). With ``always_anchor`` the hold is
    prepended even when no subdivision is needed, which gives a continuously refreshed queue one
    full step before its first target. A chunk on a timeline of its own (``first_timestamp_ms``)
    is not bridged, and neither is one the robot is already at. A bridged chunk starts where
    the robot is now, so in continuous execution it is dated at the send rather than by its
    observation. IO stays with the policy's first step. TCP rotations are unwrapped against the
    measured pose first, so no interval crosses a sign flip.
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


MIN_LEAD_MS = 20.0
"""Smallest lead a waypoint may still be sent with. A waypoint closer than this to the robot's
session time is unreachable, and commanding it makes the server jump to catch up."""


@dataclass(frozen=True, slots=True)
class DropLate:
    """Filter: drop the waypoints due within ``min_lead_ms`` of now; the rest keep their times.

    A chunk is stamped when it is queued but sent a little later, and the robot is already past
    a waypoint whose moment has gone: sending it makes the server jump back to it (contract T3,
    ADR 0004). A chunk stamped at the send itself (no ``first_timestamp_ms`` and not placed
    ahead) cannot be late and passes whole. When every waypoint is late, nothing is sent and the
    robot keeps following the previous chunk. Leave this module out only to see late waypoints
    on the wire.
    """

    min_lead_ms: float = MIN_LEAD_MS

    def keep(
        self, waypoints: list[TimedWaypoint], chunk: PendingChunk, context: ModuleContext
    ) -> list[TimedWaypoint]:
        if not chunk.lead and chunk.first_timestamp_ms is None:
            return waypoints
        earliest_ms = context.now_ms + self.min_lead_ms
        return [(t, v) for t, v in waypoints if t >= earliest_ms]


def default_pipeline(
    *, execution: object | None = None, policy: object | None = None
) -> list[object]:
    """The modules that run when no ``pipeline`` is given, in order.

    Without ``execution``: the joggers' list. With the ``PolicyExecutor``'s execution mode and
    policy: the executor's, which also clamps joint targets to the robot's limits and bridges
    from where the robot is to the policy's first step; in sequential execution it bridges
    every chunk and eases each in and out, in continuous execution it bridges only for a policy
    that asks for it (a timestep queue, :attr:`PolicyClient.requires_first_waypoint_bridge`),
    with a measured-state hold in front.
    """
    if execution is None:
        return [HoldPadding(), Timestamps(), DropLate()]
    if isinstance(execution, SequentialExecution):
        front: list[object] = [Bridge(), EndpointRamp()]
    elif getattr(policy, "requires_first_waypoint_bridge", False):
        front = [Bridge(always_anchor=True)]
    else:
        front = []
    return [*front, JointClamp(), HoldPadding(), Timestamps(), DropLate()]
