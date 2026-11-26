"""Rerun blueprint layout for policy execution visualization."""

from __future__ import annotations

from typing import TYPE_CHECKING, Any

if TYPE_CHECKING:
    from rerun import RecordingStream

import rerun as rr
import rerun.blueprint as rrb

# The timeline every plot is pinned to, not the viewer's selected one (ADR 0020).
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
    extra_3d_contents: list[str] | None = None,
) -> None:
    """Send a Rerun blueprint with 3D view, camera panels, joints, and text logs.

    ``extra_3d_contents``: further entity roots for the 3D view, e.g. a controller-wide URDF.
    """

    # Filters must use the escaped form ("0@ur10e" is stored as "0\@ur10e") or match nothing.
    escaped_ids = [rr.escape_entity_path_part(mg_id) for mg_id in motion_group_ids]

    contents_3d = ["coordinate_system_world/**", "motion/**", "collision_scenes/**"]
    for mg_id in escaped_ids:
        contents_3d.extend([f"{mg_id}/**", f"policy/{mg_id}/**"])
    for root in extra_3d_contents or []:
        entry = f"{rr.escape_entity_path_part(root)}/**"
        if entry not in contents_3d:
            contents_3d.append(entry)

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
            # A view each for position and orientation error, norms listed explicitly (ADR 0020).
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
