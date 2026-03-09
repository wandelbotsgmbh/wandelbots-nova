from nova import api
from nova_rerun_bridge.collision_scene import log_colliders_once


def log_safety_zones(
    motion_group_id: str, motion_group_description: api.models.MotionGroupDescription
) -> None:
    """
    Log hull outlines for the safety zones defined in the optimizer configuration.
    """
    if motion_group_description.safety_zones is None:
        return

    zones = motion_group_description.safety_zones.root
    entity_path = f"{motion_group_id}/zones"

    log_colliders_once(
        entity_path,
        zones,
        line_color=[221, 193, 193, 255],
        line_radius=0.75,
        mesh_color=[144, 238, 144, 80],
    )
