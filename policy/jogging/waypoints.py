"""Waypoint request construction for jogging.

Pure helpers that turn raw action steps + timing into NOVA API request models
(``JointWaypointsRequest`` / ``PoseWaypointsRequest``), plus the SDK capability
checks and the small pending-chunk record. No session state lives here — the
session passes in its :class:`~policy.jogging.clock.JoggingTimeClock` and mode.
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import TYPE_CHECKING

from nova import api

if TYPE_CHECKING:
    from policy.jogging.clock import JoggingTimeClock
    from policy.types import JoggingMode


@dataclass(slots=True, frozen=True)
class PendingChunk:
    """A queued action chunk awaiting send on the next jogging-loop iteration.

    Raw steps/timing are stored here; the request is built at yield time so
    timestamps are computed as late as possible (see :func:`make_waypoints_request`).
    """

    steps: list[list[float]]
    dt_ms: float
    first_timestamp_ms: int
    overlapping: bool = False
    backdate_ms: int = 0


def make_waypoints_request(
    clock: JoggingTimeClock,
    mode: JoggingMode,
    *,
    steps: list[list[float]],
    effective_dt_ms: float,
    first_timestamp_ms: int,
    overlapping: bool = False,
    backdate_ms: int = 0,
) -> object:
    """Build a JointWaypointsRequest or PoseWaypointsRequest at stream-yield time.

    Scales the policy's real-time timestamps to server-time using the
    auto-computed speed ratio from ``clock``. Three placement modes:

    * ``first_timestamp_ms >= 0`` (explicit absolute, e.g. replay): timestamps
      are ``[anchor, anchor + dt, ...]`` with ``anchor = scale(first_timestamp_ms)``.
    * ``first_timestamp_ms == -1`` and ``overlapping`` (RTC): the "now" anchor is
      read *here*, at yield time, so it cannot go stale while the chunk waits in
      the queue. Anchored at ``now - backdate_ms`` with the layout starting at
      the anchor, so the step matching the robot's current position lands at
      "now".
    * ``first_timestamp_ms == -1`` and not ``overlapping`` (sequential):
      timestamps start one ``dt`` into the future from "now", also read at yield
      time.

    In cartesian mode, steps are [x, y, z, rx, ry, rz] and are sent as a
    PoseWaypointsRequest. In joint mode, steps are joint radians sent as a
    JointWaypointsRequest.
    """
    # Scale timestamps by auto-computed speed ratio so the server receives
    # timestamps aligned. The policy sends in "real time"; we convert to
    # "server time".
    scaled_dt_ms = clock.scale_dt(effective_dt_ms)

    if first_timestamp_ms >= 0:
        base_ms = clock.scale_timestamp(first_timestamp_ms)
        timestamps = [base_ms + int(i * scaled_dt_ms) for i in range(len(steps))]
    elif overlapping:
        # RTC: anchor in the past so step `backdate` lands at "now". Resolve
        # "now" here, at yield, not in the executor — the queue delay between
        # update_chunk and this yield would otherwise stale the seam.
        anchor_ms = max(0, clock.client_elapsed_ms - backdate_ms)
        base_ms = clock.scale_timestamp(anchor_ms)
        timestamps = [base_ms + int(i * scaled_dt_ms) for i in range(len(steps))]
    else:
        server_now_ms = clock.scale_timestamp(clock.client_elapsed_ms)
        timestamps = [server_now_ms + int((i + 1) * scaled_dt_ms) for i in range(len(steps))]

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
