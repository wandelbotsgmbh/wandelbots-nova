"""Waypoint request construction for action chunk streaming.

Pure helpers that turn raw action steps + timing into NOVA API request models
(``ActionChunkRequest``, carrying joint- or pose-flavoured waypoints), plus the
small pending-chunk record. No session state lives here — the session passes in
its :class:`~novapolicy.jogging.clock.JoggingTimeClock` and mode.
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import TYPE_CHECKING, Annotated

from pydantic import Field

from nova import api

if TYPE_CHECKING:
    from collections.abc import Awaitable, Callable

    from novapolicy.jogging.clock import ServerMs
    from novapolicy.types import JoggingMode


class ActionChunkRequest(api.models.ActionChunkRequest):
    """``ActionChunkRequest`` with ``delay_until_braking`` [ms].

    The pinned API client (26.7.0) predates the field, while NOVA 26.8.0-112
    accepts it. Drop this once the client is 26.8.0 or newer.
    """

    delay_until_braking: Annotated[int, Field(ge=0)] | None = None


@dataclass(slots=True, frozen=True)
class PendingChunk:
    """A queued action chunk awaiting send on the next jogging-loop iteration.

    Raw steps/timing are stored here; the request is built at yield time so
    timestamps are computed as late as possible, at the send.
    """

    steps: list[list[float]]
    dt_ms: float
    first_timestamp_ms: ServerMs | None = None
    timestamp_offset_steps: int = 0
    action_timestep: int = -1
    sequence: int = 0
    caller_step_count: int = 0
    """How many leading entries of ``steps`` the caller actually asked for.

    ``steps`` may have been padded (``HoldPadding``) by repeating its final target. That padding is a braking horizon for the server, not motion
    the caller requested, so anything waiting for the commanded motion to finish
    has to stop at this many steps rather than at the end of ``steps``.

    ``0`` means "the whole list", for records built without padding.
    """
    final: bool = False
    """The last target of a run: sent with ``delay_until_braking`` 0 so the robot
    stops on it instead of keeping its velocity past it."""
    lead: bool = False
    """Placed by the lead rule: step ``k`` due ``max_expected_latency_ms + k * dt`` after
    ``observed_at_ms``, or after the send when that is ``None``."""
    observed_at_ms: ServerMs | None = None
    """When the state the chunk was computed from was measured, on the session clock; for a live
    target, when it was produced (the jogger's loop time), which a ``LiveBuffer`` times it by."""
    extend_buffer: bool = True
    """Let ``HoldPadding`` pad the chunk with its last target (contract T6). A chunk module that
    sets its own horizon (``LiveBuffer``) turns it off."""
    when_due: tuple[int, Callable[[], Awaitable[None]]] | None = None
    """``(step, action)``: run ``action`` when the caller's ``step`` is due, see
    :meth:`~novapolicy.jogging.waypoint_session.WaypointJoggingSession.update_chunk`."""
    retained_prefix_steps: int = 0
    """Leading steps already sent in an earlier chunk, which ``Smoothing`` keeps exact."""
    start_step: int = 0
    """Index of the caller's first step in ``steps``: a chunk module that puts steps in front of
    it (``Bridge``) or moves it (``EndpointRamp``) keeps this on it."""

    @property
    def last_caller_step(self) -> int:
        """Index of the last step the caller asked for."""
        count = self.caller_step_count or len(self.steps)
        return max(0, min(count, len(self.steps)) - 1)


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


def make_timed_waypoints_request(
    mode: JoggingMode,
    timed_steps: list[tuple[int, list[float]]],
    *,
    delay_until_braking_ms: int | None = None,
) -> api.models.ActionChunkRequest:
    """Build an ActionChunkRequest from waypoints that carry their own timestamps."""
    steps = [step for _timestamp, step in timed_steps]
    coordinates = _pose_coordinates(steps) if mode == "cartesian" else _joint_coordinates(steps)
    return ActionChunkRequest(
        waypoints=[
            api.models.Waypoint(timestamp=timestamp, waypoint=coordinate)
            for (timestamp, _step), coordinate in zip(timed_steps, coordinates, strict=True)
        ],
        delay_until_braking=delay_until_braking_ms,
    )


def _joint_coordinates(
    steps: list[list[float]],
) -> list[api.models.WaypointCoordinates]:
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
