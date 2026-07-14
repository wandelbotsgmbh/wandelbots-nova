"""Waypoint request construction for jogging.

Pure helpers that turn raw action steps + timing into NOVA API request models
(``JointWaypointsRequest`` / ``PoseWaypointsRequest``), plus the small
pending-chunk record. No session state lives here — the session passes in its
:class:`~novapolicy.jogging.clock.JoggingTimeClock` and mode.
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import TYPE_CHECKING

from nova import api

if TYPE_CHECKING:
    from novapolicy.jogging.clock import JoggingTimeClock
    from novapolicy.types import JoggingMode

NOW = -1
"""Anchor sentinel: resolve the chunk's anchor to "now", at yield time."""


@dataclass(slots=True, frozen=True)
class PendingChunk:
    """A queued action chunk awaiting send on the next jogging-loop iteration.

    Raw steps/timing are stored here; the request is built at yield time so
    timestamps are computed as late as possible (see :func:`make_waypoints_request`).
    """

    steps: list[list[float]]
    dt_ms: float
    anchor_ms: int = NOW
    anchor_offset_steps: int = 0
    server_anchor_ms: int | None = None
    server_dt_ms: float | None = None
    action_timestep: int = -1
    sequence: int = 0


def make_waypoints_request(
    clock: JoggingTimeClock,
    mode: JoggingMode,
    *,
    steps: list[list[float]],
    effective_dt_ms: float,
    anchor_ms: int = NOW,
    anchor_offset_steps: int = 0,
    server_anchor_ms: int | None = None,
    server_dt_ms: float | None = None,
) -> object:
    """Build a JointWaypointsRequest or PoseWaypointsRequest at stream-yield time.

    Every waypoint carries an absolute server-time timestamp laid out as
    ``base + i*dt``. The only decision is where ``base`` (step 0) sits:

    * ``server_anchor_ms`` set: an exact raw NOVA jogger-session timestamp,
      used for queue replacements that must preserve an existing server timeline.
    * ``server_dt_ms`` set: exact spacing in that raw controller timeline. This
      bypasses client-wall clock-rate scaling for controller-timed policy queues.
    * ``anchor_ms == NOW`` (default): ``base`` is "now", read *here* at yield
      time so it cannot go stale while the chunk waits in the queue. "Now" is
      acknowledged server progress (capped), not wall-clock, so a stalled link
      freezes the anchor instead of racing ahead of the robot.
    * ``anchor_ms >= 0``: an explicit absolute anchor (replay / scheduled
      segments), used verbatim.

    ``anchor_offset_steps`` then shifts that anchor by whole ``dt`` steps:
    ``+1`` places step 0 one dt into the future (live single targets, so the
    server has time to reach it); a negative value backdates the anchor so an
    already-passed step lands at "now" (RTC seam stitching); ``0`` anchors
    exactly. All timestamps are scaled to server-time by the clock's speed
    ratio.
    """
    scaled_dt_ms = server_dt_ms if server_dt_ms is not None else clock.scale_dt(effective_dt_ms)
    if server_anchor_ms is not None:
        base_ms = server_anchor_ms + int(anchor_offset_steps * scaled_dt_ms)
    elif anchor_ms == NOW:
        # Schedule directly in the server clock domain. Converting through a
        # client-relative session origin assumes both clocks started together;
        # that assumption makes later chunks drift into the server's past.
        base_ms = clock.estimated_server_timestamp_ms + int(anchor_offset_steps * scaled_dt_ms)
    else:
        base_real_ms = max(0.0, anchor_ms + anchor_offset_steps * effective_dt_ms)
        base_ms = clock.scale_timestamp(int(base_real_ms))
    base_ms = max(0, base_ms)
    timestamps = [base_ms + int(i * scaled_dt_ms) for i in range(len(steps))]

    if mode == "cartesian":
        return _build_pose_request(timestamps, steps)
    return _build_joint_request(timestamps, steps)


def _build_joint_request(timestamps: list[int], steps: list[list[float]]) -> object:
    """Build a JointWaypointsRequest from timestamps and joint steps.

    The request uses the array-of-structs layout: a single ``waypoints``
    list where each ``JointWaypoint`` bundles its timestamp with its joints.
    """
    return api.models.JointWaypointsRequest(
        waypoints=[
            api.models.JointWaypoint(timestamp=ts, joints=api.models.Joints(root=step))
            for ts, step in zip(timestamps, steps, strict=True)
        ],
    )


def _build_pose_request(timestamps: list[int], steps: list[list[float]]) -> object:
    """Build a PoseWaypointsRequest from timestamps and TCP pose steps.

    Each step is [x, y, z, rx, ry, rz] where position is in mm and
    orientation is a rotation vector in radians.
    """
    from wandelbots_api_client.v2_pydantic.models.models import (  # noqa: PLC0415
        Pose as ApiPose,
        RotationVector,
        Vector3d,
    )

    waypoints = []
    for ts, step in zip(timestamps, steps, strict=True):
        # step = [x, y, z, rx, ry, rz]
        pos = Vector3d(root=list(step[:3]))
        orient = RotationVector(root=list(step[3:6]))
        waypoints.append(
            api.models.PoseWaypoint(timestamp=ts, pose=ApiPose(position=pos, orientation=orient))
        )

    return api.models.PoseWaypointsRequest(waypoints=waypoints)
