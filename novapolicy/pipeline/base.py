"""The send pipeline: the module kinds between the policy's chunk and NOVA (contracts T13).

``Timestamps()`` goes after the last chunk module unless listed. A waypoint module sees a
re-sent waypoint's raw value again and must give a timestamp the same result.
"""

from __future__ import annotations

from dataclasses import dataclass, field
import math
from typing import TYPE_CHECKING, NamedTuple, Protocol, runtime_checkable

from novapolicy.pipeline.modules.drop_late import MIN_LEAD_MS
from novapolicy.pipeline.placement import resample_on_grid

if TYPE_CHECKING:
    from collections.abc import Sequence

    from novapolicy.mapping.joint_limits import JointLimits
    from novapolicy.transport.clock import ServerMs
    from novapolicy.transport.waypoints import PendingChunk
    from novapolicy.types import JoggingMode

CHUNK_STAGES = ("input", "smooth", "timeline", "anchor", "shape", "horizon")
"""The order of the built-in chunk modules, by their ``stage``: LiveBuffer or ServoHorizon, Smoothing,
TimestepTimeline, Bridge, EndpointRamp, HoldPadding. A module without a ``stage`` (JointClamp, your
own) may stand anywhere among them."""

_STAGE_ORDER = (
    "LiveBuffer or ServoHorizon, Smoothing, TimestepTimeline, Bridge, EndpointRamp, HoldPadding"
)

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


@dataclass
class Timestamps:
    """The placement step, as an item of the list (contracts T1 to T3).

    ``grid_ms`` puts one-step joint waypoints on the multiples of ``grid_ms`` of the session clock,
    resampled from the policy's path, instead of at the policy's own times. NOVA's follower steers
    toward the next waypoint with a gain of 1 / (time left), from a state one 8 ms cycle old: a
    waypoint due just after a cycle makes that gain spike and the arm pulse around the policy's
    pace. A multiple of the 8 ms cycle (32 for a 30 Hz policy) keeps a cycle of time left
    (ADR 0021). Off by default.
    """

    grid_ms: float | None = None
    _path: list[tuple[float, list[float]]] = field(default_factory=list, repr=False, compare=False)

    def __post_init__(self) -> None:
        if self.grid_ms is not None and not (math.isfinite(self.grid_ms) and self.grid_ms > 0):
            raise ValueError("Timestamps needs a positive finite grid_ms")

    def on_grid(
        self,
        window: list[tuple[int, list[float]]],
        now_ms: float,
        current: list[float] | None,
    ) -> list[tuple[int, list[float]]]:
        """The lead window of one-step joint targets (``window``: due time and values) resampled
        onto the grid, from ``now_ms`` plus the minimum lead; empty when the grid is off or has no
        point yet. The path continues from the targets before the window, or starts at the
        ``current`` joints at ``now_ms``. Pass an empty window to forget the path."""
        grid = self.grid_ms
        if grid is None or not window:
            self._path = []
            return []
        path = [(t, v) for t, v in self._path if t < window[0][0]]
        while len(path) > 1 and path[1][0] <= now_ms:  # keep one point at or before "now"
            path.pop(0)
        if not path and current is not None:
            path = [(float(now_ms), list(current))]
        self._path = path + [(float(t), list(v)) for t, v in window if not path or t > path[-1][0]]
        return resample_on_grid(self._path, now_ms + MIN_LEAD_MS, grid)


class SplitPipeline(NamedTuple):
    """A pipeline sorted by kind, each in the order given."""

    chunk_modules: list[ChunkModule]
    filters: list[WaypointFilter]
    waypoint_modules: list[WaypointModule]
    timestamps: Timestamps


def _check_order(pipeline: Sequence[object]) -> None:
    """Refuse a built-in module twice, or chunk modules out of their stages' order."""
    builtins: set[type] = set()
    last: tuple[int, str] | None = None
    for item in pipeline:
        name = type(item).__name__
        if type(item).__module__.startswith("novapolicy.") and not isinstance(item, Timestamps):
            if type(item) in builtins:
                raise ValueError(f"{name} appears twice in the pipeline")
            builtins.add(type(item))
        stage = getattr(item, "stage", None)
        if stage is None:
            continue
        rank = CHUNK_STAGES.index(stage)
        if last is not None and rank < last[0]:
            raise ValueError(f"{name} must come before {last[1]}: chunk modules run {_STAGE_ORDER}")
        last = (rank, name)


def split_pipeline(pipeline: Sequence[object]) -> SplitPipeline:
    """The modules of ``pipeline`` by kind, in order.

    Raises:
        ValueError: kinds out of order, chunk modules out of their stages' order
            (:data:`CHUNK_STAGES`), a built-in module twice, or ``Timestamps()`` twice.
        TypeError: an item of no kind.
    """
    _check_order(pipeline)
    split = SplitPipeline([], [], [], Timestamps())
    stamped = False
    for item in pipeline:
        if isinstance(item, Timestamps):
            if stamped:
                raise ValueError("Timestamps() appears twice in the pipeline")
            split = split._replace(timestamps=item)
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
