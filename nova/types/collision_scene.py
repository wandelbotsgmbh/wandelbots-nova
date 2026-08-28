# TODO Why is this in nova.types? It offers no type, only a utility function.
from nova import api


def compare_collition_scenes(
    scene1: api.models.MultiCollisionSetup, scene2: api.models.MultiCollisionSetup
):
    if scene1.colliders != scene2.colliders:
        return False

    # Compare motion groups
    if (
        scene1.collision_motion_groups_by_motion_group_key
        != scene2.collision_motion_groups_by_motion_group_key
    ):
        return False

    return True
