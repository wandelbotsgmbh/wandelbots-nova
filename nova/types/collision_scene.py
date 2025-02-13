import wandelbots_api_client as wb


def compare_collition_scenes(scene1: wb.models.CollisionScene, scene2: wb.models.CollisionScene):
    if scene1.colliders != scene2.colliders:
        return False

    # Compare motion groups
    if scene1.motion_groups != scene2.motion_groups:
        return False

    return True
