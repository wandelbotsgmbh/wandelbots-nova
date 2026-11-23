"""Pure helpers that build NOVA ``ActionChunkRequest`` models, and the ``PendingChunk`` record."""

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
    """``ActionChunkRequest`` with ``delay_until_braking`` [ms]; drop with API client >= 26.8.0."""

    delay_until_braking: Annotated[int, Field(ge=0)] | None = None


@dataclass(slots=True, frozen=True)
class PendingChunk:
    """A queued action chunk; its request and timestamps are built at the send."""

    steps: list[list[float]]
    dt_ms: float
    first_timestamp_ms: ServerMs | None = None
    timestamp_offset_steps: int = 0
    action_timestep: int = -1
    """The policy timestep of the caller's first step (``steps[start_step]``); ``-1`` without."""
    sequence: int = 0
    caller_step_count: int = 0
    """How many leading entries of ``steps`` the caller asked for, before ``HoldPadding``;
    ``0`` means all."""
    final: bool = False
    """The last target of a run: sent with ``delay_until_braking`` 0 so the robot stops on it."""
    lead: bool = False
    """Placed by the lead rule: step ``k`` due ``max_expected_latency_ms + k * dt`` after
    ``observed_at_ms``, or after the send when that is ``None``."""
    observed_at_ms: ServerMs | None = None
    """When the chunk's state was measured (a live target: when it was produced), session clock."""
    extend_buffer: bool = True
    """Let ``HoldPadding`` pad the chunk (contract T6); ``LiveBuffer`` turns it off."""
    when_due: tuple[int, Callable[[], Awaitable[None]]] | None = None
    """``(step, action)``: run ``action`` when the caller's ``step`` is due, see
    :meth:`~novapolicy.jogging.waypoint_session.WaypointJoggingSession.update_chunk`."""
    retained_prefix_steps: int = 0
    """Leading steps already sent in an earlier chunk, which ``Smoothing`` keeps exact."""
    start_step: int = 0
    """Index of the caller's first step in ``steps``, kept on it by chunk modules."""

    @property
    def last_caller_step(self) -> int:
        """Index of the last step the caller asked for."""
        count = self.caller_step_count or len(self.steps)
        return max(0, min(count, len(self.steps)) - 1)


def anchor_timestamp_ms(
    first_timestamp_ms: int, timestamp_offset_steps: int, step_dt_ms: float
) -> int:
    """Where step zero lands, given a base timestamp and a whole-step offset.

    Not clamped at 0: trimming has to see the anchor before request building's ``max(0, ...)``.
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
    """Wrap TCP pose steps [x, y, z, rx, ry, rz] (mm, rotation vector) as ``POSE`` coordinates."""
    return [
        api.models.PoseWaypoint(
            pose=api.models.Pose(position=tuple(step[:3]), orientation=tuple(step[3:6]))
        )
        for step in steps
    ]
