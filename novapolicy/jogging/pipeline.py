"""The send pipeline: opt-in modules between the policy's chunk and NOVA.

Everything that reaches NOVA passes one ordered list, per motion group::

    common chunk  ->  [chunk modules]  ->  Timestamps()  ->  [waypoint modules]  ->  NOVA sender

* **Common chunk.** What a policy, a jogger target or a replay produced, already mapped to NOVA
  (joints in rad or TCP poses in mm and rad, one motion group): a
  :class:`~novapolicy.jogging.waypoints.PendingChunk`.
* **Chunk modules** (``on_chunk``) run when the chunk is queued, on every chunk, before the session
  keeps only the newest one. They may change steps and timing, e.g. :class:`LiveBuffer` turns a
  live target into a horizon on its own timeline.
* **Timestamps()** is the placement step (contracts T1 to T3): each step gets its time on the
  session clock. It is in every pipeline; list it to say where, or it goes after the last chunk
  module.
* **Waypoint modules** (``on_waypoints``) run at the send, on every request, after the timestamps:
  they get ``(timestamp, values)`` and return the values, so they cannot touch timing. A re-sent
  waypoint reaches them with its raw value again, so they must give a timestamp the same result.
  e.g. :class:`EaseIn`.
* **The NOVA sender** is always last: trimming, the rolling window, IO at the policy's first step,
  the request (T3, T5, T7).

Contract for every module: none runs unless passed (``PolicyExecutor``, ``jog_joints`` and
``jog_tcp`` take ``pipeline=[...]``); each motion group runs its own copy, so arms never share
state; a chunk module that adds or removes steps keeps ``when_due`` pointing at the step its
action belongs to.
"""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import TYPE_CHECKING, Protocol, runtime_checkable

if TYPE_CHECKING:
    from collections.abc import Sequence

    from novapolicy.jogging.clock import ServerMs
    from novapolicy.jogging.waypoints import PendingChunk

TimedWaypoint = tuple["ServerMs", list[float]]
"""A waypoint about to be sent: its time on the session clock and its values."""


@dataclass(frozen=True, slots=True)
class ModuleContext:
    """What a module may read besides its input."""

    now_ms: ServerMs
    """The session clock now."""
    position: list[float] | None
    """The robot's latest measured position, in the waypoints' form; ``None`` before the first
    state arrived."""
    step_ms: float
    """The session's grid for steps without their own spacing (``single_step_dt_ms``)."""


@runtime_checkable
class ChunkModule(Protocol):
    """Before the timestamps: a chunk in, a chunk out."""

    def on_chunk(self, chunk: PendingChunk, context: ModuleContext) -> PendingChunk:
        """The chunk to pass on."""
        ...


@runtime_checkable
class WaypointModule(Protocol):
    """After the timestamps: the waypoints' new values, one per waypoint, in order."""

    def on_waypoints(
        self, waypoints: list[TimedWaypoint], context: ModuleContext
    ) -> list[list[float]]:
        """The new values."""
        ...


@dataclass(frozen=True, slots=True)
class Timestamps:
    """The placement step, as an item of the list (contracts T1 to T3)."""


def split_pipeline(
    pipeline: Sequence[object],
) -> tuple[list[ChunkModule], list[WaypointModule]]:
    """The chunk modules and the waypoint modules of ``pipeline``, in order.

    Raises ``ValueError`` when a chunk module comes after the timestamps or a waypoint module
    before them, when ``Timestamps()`` appears twice, or for an item that is neither.
    """
    chunk_modules: list[ChunkModule] = []
    waypoint_modules: list[WaypointModule] = []
    stamped = False
    for item in pipeline:
        if isinstance(item, Timestamps):
            if stamped:
                raise ValueError("Timestamps() appears twice in the pipeline")
            stamped = True
        elif isinstance(item, ChunkModule):
            if stamped or waypoint_modules:
                msg = f"{type(item).__name__} works on chunks; put it before Timestamps()"
                raise ValueError(msg)
            chunk_modules.append(item)
        elif isinstance(item, WaypointModule):
            waypoint_modules.append(item)
            stamped = True  # the timestamps sit before the first waypoint module
        else:
            msg = f"{type(item).__name__} is not a pipeline module (on_chunk or on_waypoints)"
            raise TypeError(msg)
    return chunk_modules, waypoint_modules


@dataclass
class EaseIn:
    """Waypoint module: blend the commanded values in from where the robot stood, over the first
    ``duration_s``.

    The blend is a smootherstep (``6x^5 - 15x^4 + 10x^3``) of the time since the first waypoint
    it saw, so velocity and acceleration start and end at zero; a straight ramp steps the velocity
    when it completes, and a smoothstep the acceleration. Each waypoint is blended by its own
    timestamp, so overlapping chunks agree on every moment. TCP poses blend their rotation vector
    component by component, which is fine for the small rotations at a start.
    """

    duration_s: float
    _start: tuple[int, list[float]] | None = field(default=None, init=False, repr=False)

    def __post_init__(self) -> None:
        if self.duration_s <= 0:
            raise ValueError("EaseIn needs a positive duration_s")

    def on_waypoints(
        self, waypoints: list[TimedWaypoint], context: ModuleContext
    ) -> list[list[float]]:
        values = [list(v) for _, v in waypoints]
        if not waypoints:
            return values
        if self._start is None:
            if context.position is None:
                return values  # nothing measured yet to blend from
            self._start = (waypoints[0][0], list(context.position))
        t0_ms, start = self._start
        blended: list[list[float]] = []
        for (t_ms, _), value in zip(waypoints, values, strict=True):
            x = min(1.0, max(0.0, (t_ms - t0_ms) / (1000.0 * self.duration_s)))
            if x >= 1.0:
                blended.append(value)
                continue
            e = x**3 * (x * (x * 6.0 - 15.0) + 10.0)
            blended.append([s + e * (v - s) for s, v in zip(start, value, strict=True)])
        return blended
