"""Rerun blueprint layout for policy execution visualization."""

from __future__ import annotations

from typing import Any

import rerun as rr
import rerun.blueprint as rrb


def send_blueprint(
    motion_group_ids: list[str],
    camera_names: list[str],
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
        rrb.TimeSeriesView(contents=[f"policy/{mg_id}/joints/**"], name=f"Joints {mg_id}")
        for mg_id in escaped_ids
    ]
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
        )
    )
