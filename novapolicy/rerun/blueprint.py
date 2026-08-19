"""Rerun blueprint layout for policy execution visualization."""

from __future__ import annotations

from typing import TYPE_CHECKING, Any

if TYPE_CHECKING:
    from rerun import RecordingStream

import rerun as rr
import rerun.blueprint as rrb

# Every plotted series is stamped on ``policy_time``. Left unset, a view falls
# back to whichever timeline the viewer has selected — in practice the built-in
# wall-clock ``log_time``, which records when a value was written rather than
# when it was measured. Pinning the range to ``policy_time`` keeps the plots on
# the timeline the data actually belongs to.
_POLICY_TIMELINE = "policy_time"


def _whole_policy_timeline() -> rrb.VisibleTimeRange:
    return rrb.VisibleTimeRange(
        _POLICY_TIMELINE,
        start=rrb.TimeRangeBoundary.infinite(),
        end=rrb.TimeRangeBoundary.infinite(),
    )


def send_blueprint(
    motion_group_ids: list[str],
    camera_names: list[str],
    *,
    recording: RecordingStream | None = None,
) -> None:
    """Send a Rerun blueprint with 3D view, camera panels, joints, and text logs."""

    # Motion-group ids can contain characters that Rerun escapes in entity paths
    # (e.g. "@" in "0@ur10e" is stored as "0\@ur10e"). Blueprint content filters
    # must use the same escaped form or they silently match nothing.
    escaped_ids = [rr.escape_entity_path_part(mg_id) for mg_id in motion_group_ids]

    # 3D view contents: robot meshes + policy overlays
    contents_3d = ["coordinate_system_world/**", "motion/**", "collision_scenes/**"]
    for mg_id in escaped_ids:
        contents_3d.extend([f"{mg_id}/**", f"policy/{mg_id}/**"])

    views: list[Any] = [
        rrb.Spatial3DView(contents=contents_3d, name="3D View", background=[20, 22, 35]),
    ]

    camera_views = [
        rrb.Spatial2DView(contents=[f"policy/cameras/{n}"], name=n) for n in camera_names
    ]
    joint_views = [
        rrb.TimeSeriesView(
            contents=[
                f"policy/{mg_id}/joints/**",
                f"policy/{mg_id}/joint_target/**",
            ],
            name=f"Joints target/actual {mg_id}",
            time_ranges=_whole_policy_timeline(),
        )
        for mg_id in escaped_ids
    ]
    tcp_tracking_views = []
    for mg_id in escaped_ids:
        tcp_tracking_views.extend([
            rrb.TimeSeriesView(
                contents=[
                    f"policy/{mg_id}/tcp_target/position/**",
                    f"policy/{mg_id}/tcp_target/orientation/**",
                    f"policy/{mg_id}/tcp_actual/position/**",
                    f"policy/{mg_id}/tcp_actual/orientation/**",
                ],
                name=f"TCP target/actual {mg_id}",
                time_ranges=_whole_policy_timeline(),
            ),
            # Position and orientation error get a view each. A time-series
            # view shares one Y axis across everything in it, and these are in
            # different units at wildly different magnitudes: position error runs
            # to tens of mm while orientation error is a few thousandths of a
            # radian. Plotted together, the orientation series are flattened onto
            # zero by the position scale and read as "no error" whatever their
            # actual value.
            #
            # The ``_norm_`` entities are siblings of the ``position``/
            # ``orientation`` groups rather than children, so they have to be
            # listed explicitly — a bare ``position/**`` would silently drop the
            # norm, which is the one series that shows a constant lag as constant.
            rrb.TimeSeriesView(
                contents=[
                    f"policy/{mg_id}/tcp_error/position/**",
                    f"policy/{mg_id}/tcp_error/position_norm_mm",
                ],
                name=f"TCP position error {mg_id} [mm]",
                time_ranges=_whole_policy_timeline(),
            ),
            rrb.TimeSeriesView(
                contents=[
                    f"policy/{mg_id}/tcp_error/orientation/**",
                    f"policy/{mg_id}/tcp_error/orientation_norm_rad",
                ],
                name=f"TCP orientation error {mg_id} [rad]",
                time_ranges=_whole_policy_timeline(),
            ),
        ])
    text_views = [
        rrb.TextLogView(
            contents=["policy/action_chunks", "policy/status"],
            name="Action Chunks",
        ),
    ]

    right_panels: list[Any] = []
    if camera_views:
        right_panels.append(rrb.Grid(*camera_views))
    if joint_views:
        right_panels.append(rrb.Vertical(*joint_views))
    if tcp_tracking_views:
        right_panels.append(rrb.Vertical(*tcp_tracking_views))
    if text_views:
        right_panels.append(rrb.Vertical(*text_views))

    layout = (
        rrb.Horizontal(views[0], rrb.Vertical(*right_panels), column_shares=[3, 2])
        if right_panels
        else views[0]
    )
    rr.send_blueprint(
        rrb.Blueprint(
            layout,
            rrb.TimePanel(state="expanded", timeline="policy_time"),
            collapse_panels=True,
        ),
        recording=recording,
    )
