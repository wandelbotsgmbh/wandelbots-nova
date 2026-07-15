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


def make_waypoints_request(
    clock: JoggingTimeClock,
    mode: JoggingMode,
    *,
    steps: list[list[float]],
    effective_dt_ms: float,
    first_timestamp_ms: int | None = None,
    timestamp_offset_steps: int = 0,
    server_dt_ms: float | None = None,
) -> api.models.JointWaypointsRequest | api.models.PoseWaypointsRequest:
    """Build a JointWaypointsRequest or PoseWaypointsRequest at stream-yield time.

    Every waypoint carries an absolute server-time timestamp laid out as
    ``base + i*dt``. The only decision is where ``base`` (step 0) sits:

    * ``first_timestamp_ms`` set: the exact raw NOVA jogger-session timestamp
      for step zero on an existing controller timeline.
    * ``first_timestamp_ms`` omitted: step zero is relative to server "now",
      resolved here so it cannot go stale while the chunk waits in the queue.
      "Now" is acknowledged server progress (capped), not wall-clock, so a
      stalled link freezes the timestamp instead of racing ahead of the robot.
    * ``server_dt_ms`` set: exact spacing in the raw controller timeline. This
      bypasses client-wall clock-rate scaling for controller-timed policy queues.

    ``timestamp_offset_steps`` shifts the selected timestamp by whole ``dt``
    steps: ``+1`` places step zero one interval in the future; a negative value
    backdates an overlapping seam; ``0`` uses the timestamp exactly.
    """
    scaled_dt_ms = server_dt_ms if server_dt_ms is not None else clock.scale_dt(effective_dt_ms)
    base_ms = (
        clock.estimated_server_timestamp_ms if first_timestamp_ms is None else first_timestamp_ms
    )
    base_ms += int(timestamp_offset_steps * scaled_dt_ms)
    base_ms = max(0, base_ms)
    timestamps = [base_ms + int(i * scaled_dt_ms) for i in range(len(steps))]

    if mode == "cartesian":
        return _build_pose_request(timestamps, steps)
    return _build_joint_request(timestamps, steps)


def _build_joint_request(
    timestamps: list[int], steps: list[list[float]]
) -> api.models.JointWaypointsRequest:
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


def _build_pose_request(
    timestamps: list[int], steps: list[list[float]]
) -> api.models.PoseWaypointsRequest:
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
