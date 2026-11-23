"""The send pipeline: the module kinds between the policy's chunk and NOVA (contracts T13).

``Timestamps()`` goes after the last chunk module unless listed. A waypoint module sees a
re-sent waypoint's raw value again and must give a timestamp the same result.
"""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import TYPE_CHECKING, NamedTuple, Protocol, runtime_checkable

if TYPE_CHECKING:
    from collections.abc import Sequence

    from novapolicy.jogging.clock import ServerMs
    from novapolicy.jogging.waypoints import PendingChunk
    from novapolicy.joint_limits import JointLimits
    from novapolicy.types import JoggingMode

TimedWaypoint = tuple["ServerMs", list[float]]
"""A waypoint about to be sent: its time on the session clock and its values."""


@dataclass(frozen=True, slots=True)
class ModuleContext:
    """What a module may read besides its input."""

    now_ms: ServerMs
    """The session clock now."""
    position: list[float] | None
    """The robot's latest measured position, in the waypoints' form; ``None`` before one."""
    step_ms: float
    """The session's grid for steps without their own spacing (``single_step_dt_ms``)."""
    motion_group_id: str = ""
    """The motion group the session drives."""
    mode: JoggingMode = "joint"
    """What the waypoints are: joint positions, or TCP poses (``"cartesian"``)."""
    joint_limits: JointLimits | None = None
    """The robot's joint limits, read at start for a ``JointClamp``; else ``None``."""
    scheduled_start: tuple[int, ServerMs] | None = None
    """``(timestep, due)`` of the policy's first step of the latest chunk sent with timesteps."""


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


@runtime_checkable
class WaypointFilter(Protocol):
    """After the timestamps, before the waypoint modules: which waypoints go out."""

    def keep(
        self, waypoints: list[TimedWaypoint], chunk: PendingChunk, context: ModuleContext
    ) -> list[TimedWaypoint]:
        """The waypoints to send: some of ``waypoints``, in order, unchanged."""
        ...


@dataclass(frozen=True, slots=True)
class Timestamps:
    """The placement step, as an item of the list (contracts T1 to T3)."""


class SplitPipeline(NamedTuple):
    """A pipeline sorted by kind, each in the order given."""

    chunk_modules: list[ChunkModule]
    filters: list[WaypointFilter]
    waypoint_modules: list[WaypointModule]


def split_pipeline(pipeline: Sequence[object]) -> SplitPipeline:
    """The modules of ``pipeline`` by kind, in order.

    Raises:
        ValueError: kinds out of order, or ``Timestamps()`` twice.
        TypeError: an item of no kind.
    """
    split = SplitPipeline([], [], [])
    stamped = False
    for item in pipeline:
        if isinstance(item, Timestamps):
            if stamped:
                raise ValueError("Timestamps() appears twice in the pipeline")
            stamped = True
        elif isinstance(item, ChunkModule):
            if stamped:
                msg = f"{type(item).__name__} works on chunks; put it before Timestamps()"
                raise ValueError(msg)
            split.chunk_modules.append(item)
        elif isinstance(item, WaypointFilter):
            if split.waypoint_modules:
                msg = f"{type(item).__name__} drops waypoints; put it before the waypoint modules"
                raise ValueError(msg)
            split.filters.append(item)
            stamped = True  # the timestamps sit before the first filter
        elif isinstance(item, WaypointModule):
            split.waypoint_modules.append(item)
            stamped = True  # the timestamps sit before the first waypoint module
        else:
            msg = f"{type(item).__name__} is not a pipeline module (on_chunk, keep or on_waypoints)"
            raise TypeError(msg)
    return split


@dataclass
class EaseIn:
    """Waypoint module: blend from where the robot stood over the first ``duration_s``.

    A smootherstep of each waypoint's own timestamp since the first one (docs/jogging.md); TCP
    rotation vectors blend per component.
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
