import rerun as rr

from nova import api
from nova_rerun_bridge import collider_shapes, scene_colors


def log_collision_setups(collision_setups: dict[str, api.models.CollisionSetup]):
    for setup_id, setup in collision_setups.items():
        entity_path = f"collision_setups/{rr.escape_entity_path_part(setup_id)}"
        if setup.colliders:
            for collider_id, collider in setup.colliders.items():
                log_colliders_once(entity_path, {collider_id: collider})


def log_colliders_once(entity_path: str, colliders: dict[str, api.models.Collider]):
    """Draw a collision setup's colliders where the setup places them.

    These are things in the cell rather than a state of the robot, so they get
    the neutral colour; see :mod:`nova_rerun_bridge.scene_colors`.
    """
    for collider_id, collider in colliders.items():
        collider_shapes.log_collider(
            f"{entity_path}/{rr.escape_entity_path_part(collider_id)}",
            collider,
            color=scene_colors.OBSTACLE,
            fill_mode=collider_shapes.IN_CELL,
        )


def extract_link_chain_and_tcp(
    collision_setups: dict[str, api.models.CollisionSetup],
) -> tuple[api.models.LinkChain | None, api.models.Tool | None]:
    """The first collision setup's link chain and tool.

    A reader for callers that want them; the bridge draws a robot's collision
    geometry from the URDF instead.
    """
    for setup in collision_setups.values():
        return setup.link_chain, setup.tool
    return None, None
