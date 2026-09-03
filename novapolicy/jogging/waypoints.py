"""Waypoint request construction for action chunk streaming.

Pure helpers that turn raw action steps + timing into NOVA API request models
(``ActionChunkRequest``, carrying joint- or pose-flavoured waypoints), plus the
small pending-chunk record. No session state lives here — the session passes in
its :class:`~novapolicy.jogging.clock.JoggingTimeClock` and mode.
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import TYPE_CHECKING

from nova import api

if TYPE_CHECKING:
    from novapolicy.jogging.clock import JoggingTimeClock
    from novapolicy.types import JoggingMode


@dataclass(slots=True, frozen=True)
class PendingChunk:
    """A queued action chunk awaiting send on the next jogging-loop iteration.

    Raw steps/timing are stored here; the request is built at yield time so
    timestamps are computed as late as possible (see :func:`make_waypoints_request`).
    """

    steps: list[list[float]]
    dt_ms: float
    first_timestamp_ms: int | None = None
    timestamp_offset_steps: int = 0
    server_dt_ms: float | None = None
    action_timestep: int = -1
    sequence: int = 0
    caller_step_count: int = 0
    """How many leading entries of ``steps`` the caller actually asked for.

    ``steps`` may have been padded to fill ``min_chunk_horizon_ms`` by repeating its
    final target. That padding is a braking horizon for the server, not motion
    the caller requested, so anything waiting for the commanded motion to finish
    has to stop at this many steps rather than at the end of ``steps``.

    ``0`` means "the whole list", for records built without padding.
    """

    @property
    def last_caller_step(self) -> int:
        """Index of the last step the caller asked for."""
        count = self.caller_step_count or len(self.steps)
        return max(0, min(count, len(self.steps)) - 1)


def step_spacing_ms(effective_dt_ms: float, server_dt_ms: float | None) -> float:
    """Spacing between consecutive waypoints, in server milliseconds.

    An explicit ``server_dt_ms`` overrides the caller's own step interval;
    otherwise the caller's interval is used as-is. Server jogger milliseconds are
    real milliseconds, so no clock-rate scaling is applied.
    """
    return server_dt_ms if server_dt_ms is not None else effective_dt_ms


def anchor_timestamp_ms(
    first_timestamp_ms: int, timestamp_offset_steps: int, step_dt_ms: float
) -> int:
    """Where step zero lands, given a base timestamp and a whole-step offset.

    Shared with the session's trimming pass, which has to resolve the same
    absolute anchor before it can tell which waypoints are already in the past.
    Deliberately unclamped: the ``max(0, ...)`` floor belongs to request
    building, and applying it here would change which waypoints trimming
    considers reachable.
    """
    return first_timestamp_ms + int(timestamp_offset_steps * step_dt_ms)


def make_waypoints_request(
    clock: JoggingTimeClock,
    mode: JoggingMode,
    *,
    steps: list[list[float]],
    effective_dt_ms: float,
    first_timestamp_ms: int | None = None,
    timestamp_offset_steps: int = 0,
    server_dt_ms: float | None = None,
) -> api.models.ActionChunkRequest:
    """Build the ActionChunkRequest for a chunk at stream-yield time.

    Every waypoint carries an absolute server-time timestamp laid out as
    ``base + i*dt``. The only decision is where ``base`` (step 0) sits:

    * ``first_timestamp_ms`` set: the exact raw NOVA jogger-session timestamp
      for step zero on an existing controller timeline.
    * ``first_timestamp_ms`` omitted: step zero is relative to server "now",
      resolved here so it cannot go stale while the chunk waits in the queue.
      "Now" is extrapolated from acknowledged server progress rather than
      wall-clock, so it tracks the robot's own timer between state samples.

    Spacing is ``server_dt_ms`` when given, otherwise ``effective_dt_ms``
    unchanged: server jogger milliseconds are real milliseconds, so no
    clock-rate scaling is applied.

    ``timestamp_offset_steps`` shifts the selected timestamp by whole ``dt``
    steps: ``+1`` places step zero one interval in the future; a negative value
    backdates an overlapping seam; ``0`` uses the timestamp exactly.
    """
    step_dt_ms = step_spacing_ms(effective_dt_ms, server_dt_ms)
    base_ms = (
        clock.estimated_server_timestamp_ms if first_timestamp_ms is None else first_timestamp_ms
    )
    base_ms = max(0, anchor_timestamp_ms(base_ms, timestamp_offset_steps, step_dt_ms))
    timestamps = [base_ms + int(i * step_dt_ms) for i in range(len(steps))]

    coordinates = _pose_coordinates(steps) if mode == "cartesian" else _joint_coordinates(steps)

    return api.models.ActionChunkRequest(
        waypoints=[
            api.models.Waypoint(timestamp=ts, waypoint=coordinate)
            for ts, coordinate in zip(timestamps, coordinates, strict=True)
        ],
    )


def _joint_coordinates(steps: list[list[float]]) -> list[api.models.WaypointCoordinates]:
    """Wrap joint steps [rad] as ``JOINTS`` waypoint coordinates."""
    return [api.models.JointWaypoint(joints=step) for step in steps]


def _pose_coordinates(steps: list[list[float]]) -> list[api.models.WaypointCoordinates]:
    """Wrap TCP pose steps as ``POSE`` waypoint coordinates.

    Each step is [x, y, z, rx, ry, rz] where position is in mm and
    orientation is a rotation vector in radians.
    """
    return [
        api.models.PoseWaypoint(
            pose=api.models.Pose(position=tuple(step[:3]), orientation=tuple(step[3:6]))
        )
        for step in steps
    ]
