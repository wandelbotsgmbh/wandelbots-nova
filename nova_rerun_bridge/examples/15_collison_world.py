import asyncio

from nova.api import models
from nova.core.nova import Nova
from nova_rerun_bridge import NovaRerunBridge


async def build_collision_world(
    nova: Nova, cell_name: str, robot_setup: models.OptimizerSetup
) -> str:
    collision_api = nova._api_client.store_collision_components_api
    scene_api = nova._api_client.store_collision_scenes_api

    # Load all colliders from the JSON data
    colliders = {}

    # Box collider
    box_collider = models.Collider(
        shape=models.ColliderShape(
            models.Box2(size_x=100, size_y=50, size_z=200, shape_type="box", box_type="FULL")
        ),
        pose=models.Pose2(position=[0, 400, 0], orientation=[0, 0, 0]),
    )
    colliders["box"] = box_collider

    # Sphere collider
    sphere_collider = models.Collider(
        shape=models.ColliderShape(models.Sphere2(radius=30, shape_type="sphere")),
        pose=models.Pose2(position=[0, 200, 0]),
    )
    colliders["sphere"] = sphere_collider

    # Cylinder collider
    cylinder_collider = models.Collider(
        shape=models.ColliderShape(models.Cylinder2(radius=30, height=100, shape_type="cylinder")),
        pose=models.Pose2(position=[0, -600, 0]),
    )
    colliders["cylinder"] = cylinder_collider

    # Capsule collider
    capsule_collider = models.Collider(
        shape=models.ColliderShape(
            models.Capsule2(radius=30, cylinder_height=100, shape_type="capsule")
        ),
        pose=models.Pose2(position=[0, -400, 0]),
    )
    colliders["capsule"] = capsule_collider

    # Rectangular capsule collider
    rect_capsule_collider = models.Collider(
        shape=models.ColliderShape(
            models.RectangularCapsule2(
                radius=30,
                sphere_center_distance_x=100,
                sphere_center_distance_y=50,
                shape_type="rectangular_capsule",
            )
        ),
        pose=models.Pose2(position=[0, -200, 0]),
    )
    colliders["rectangular_capsule"] = rect_capsule_collider

    # Rectangle collider
    rectangle_collider = models.Collider(
        shape=models.ColliderShape(
            models.Rectangle2(size_x=30, size_y=100, shape_type="rectangle")
        ),
        pose=models.Pose2(position=[0, 0, 0]),
    )
    colliders["rectangle"] = rectangle_collider

    # Add rotated variants (x-axis 45 degrees)
    for name, collider in list(colliders.items()):
        rotated = models.Collider(
            shape=collider.shape,
            pose=models.Pose2(
                position=[
                    -300,
                    collider.pose.position[1] if collider.pose and collider.pose.position else 0,
                    collider.pose.position[2] if collider.pose and collider.pose.position else 0,
                ],
                orientation=[0.7853981633974484, 0, 0],  # 45 degrees in radians
            ),
        )
        colliders[f"{name}_rot_x_45"] = rotated

    # Add rotated variants (y-axis 45 degrees)
    for name, collider in list(colliders.items()):
        if "_rot_" not in name:  # Only rotate original objects
            rotated = models.Collider(
                shape=collider.shape,
                pose=models.Pose2(
                    position=[
                        300,
                        collider.pose.position[1]
                        if collider.pose and collider.pose.position
                        else 0,
                        collider.pose.position[2]
                        if collider.pose and collider.pose.position
                        else 0,
                    ],
                    orientation=[0, 0.7853981633974484, 0],  # 45 degrees in radians
                ),
            )
            colliders[f"{name}_rot_y_45"] = rotated

    # Store all colliders
    for name, collider in colliders.items():
        await collision_api.store_collider(cell=cell_name, collider=name, collider2=collider)

    # Define TCP collider geometry
    tool_collider = models.Collider(
        shape=models.ColliderShape(
            models.Box2(size_x=100, size_y=100, size_z=100, shape_type="box", box_type="FULL")
        )
    )
    await collision_api.store_collision_tool(
        cell=cell_name, tool="tool_box", request_body={"tool_collider": tool_collider}
    )

    # Define robot link geometries
    robot_link_colliders = await collision_api.get_default_link_chain(
        cell=cell_name, motion_group_model=robot_setup.motion_group_type
    )
    await collision_api.store_collision_link_chain(
        cell=cell_name, link_chain="robot_links", collider=robot_link_colliders
    )

    # Assemble scene with all colliders
    scene = models.CollisionScene(
        colliders=colliders,
        motion_groups={
            robot_setup.motion_group_type: models.CollisionMotionGroup(
                tool={"tool_geometry": tool_collider}, link_chain=robot_link_colliders
            )
        },
    )
    scene_id = "collision_scene"
    await scene_api.store_collision_scene(
        cell_name, scene_id, models.CollisionSceneAssembly(scene=scene)
    )
    return scene_id


async def test():
    async with Nova() as nova, NovaRerunBridge(nova) as bridge:
        await bridge.setup_blueprint()

        cell = nova.cell()
        controller = await cell.ensure_virtual_robot_controller(
            "ur5",
            models.VirtualControllerTypes.UNIVERSALROBOTS_MINUS_UR5E,
            models.Manufacturer.UNIVERSALROBOTS,
        )
        # Connect to the controller and activate motion groups
        async with controller[0] as motion_group:
            await bridge.log_saftey_zones(motion_group)

            tcp = "Flange"

            robot_setup: models.OptimizerSetup = await motion_group._get_optimizer_setup(tcp=tcp)

            await build_collision_world(nova, "cell", robot_setup)

            await bridge.log_collision_scenes()


if __name__ == "__main__":
    asyncio.run(test())
