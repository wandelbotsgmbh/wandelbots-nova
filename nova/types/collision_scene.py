# TODO Why is this in nova.types? It offers no type, only a utility function.
from nova import api


def compare_collition_scenes(scene1: api.models.CollisionSetup, scene2: api.models.CollisionSetup):
    if scene1.colliders != scene2.colliders:
        return False

    # Compare motion groups
    if scene1.link_chain != scene2.link_chain:
        return False

    return True
