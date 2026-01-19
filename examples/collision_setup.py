import asyncio

import nova
from nova import Nova, api
from nova.actions import cartesian_ptp
from nova.cell import virtual_controller
from nova.types import Pose
from nova.viewers.utils import extract_collision_setups_from_actions
from nova_rerun_bridge import NovaRerunBridge


async def build_collision_world(
    nova: Nova, cell_name: str, motion_group_description: api.models.MotionGroupDescription
) -> str:
    collision_api = nova._api_client.store_collision_components_api
    scene_api = nova._api_client.store_collision_setups_api

    # Load all colliders from the JSON data
    colliders: dict[str, api.models.Collider] = {}

    # Box collider
    box_collider = api.models.Collider(
        shape=api.models.Box(
            size_x=100, size_y=50, size_z=200, shape_type="box", box_type=api.models.BoxType.FULL
        ),
        pose=api.models.Pose(
            position=api.models.Vector3d([0, 400, 0]),
            orientation=api.models.RotationVector([0, 0, 0]),
        ),
    )
    colliders["box"] = box_collider

    # Sphere collider
    sphere_collider = api.models.Collider(
        shape=api.models.Sphere(radius=30, shape_type="sphere"),
        pose=api.models.Pose(
            position=api.models.Vector3d([0, 200, 0]),
            orientation=api.models.RotationVector([0, 0, 0]),
        ),
    )
    colliders["sphere"] = sphere_collider

    # Cylinder collider
    cylinder_collider = api.models.Collider(
        shape=api.models.Cylinder(radius=30, height=100, shape_type="cylinder"),
        pose=api.models.Pose(
            position=api.models.Vector3d([0, -600, 0]),
            orientation=api.models.RotationVector([0, 0, 0]),
        ),
    )
    colliders["cylinder"] = cylinder_collider

    # Capsule collider
    capsule_collider = api.models.Collider(
        shape=api.models.Capsule(radius=30, cylinder_height=100, shape_type="capsule"),
        pose=api.models.Pose(
            position=api.models.Vector3d([0, -400, 0]),
            orientation=api.models.RotationVector([0, 0, 0]),
        ),
    )
    colliders["capsule"] = capsule_collider

    # Rectangular capsule collider
    rect_capsule_collider = api.models.Collider(
        shape=api.models.RectangularCapsule(
            radius=30,
            sphere_center_distance_x=100,
            sphere_center_distance_y=50,
            shape_type="rectangular_capsule",
        ),
        pose=api.models.Pose(
            position=api.models.Vector3d([0, -200, 0]),
            orientation=api.models.RotationVector([0, 0, 0]),
        ),
    )
    colliders["rectangular_capsule"] = rect_capsule_collider

    # Rectangle collider
    rectangle_collider = api.models.Collider(
        shape=api.models.Rectangle(size_x=30, size_y=100, shape_type="rectangle"),
        pose=api.models.Pose(
            position=api.models.Vector3d([0, 0, 0]),
            orientation=api.models.RotationVector([0, 0, 0]),
        ),
    )
    colliders["rectangle"] = rectangle_collider

    # Add rotated variants (x-axis 45 degrees)
    for name, collider in list(colliders.items()):
        rotated = api.models.Collider(
            shape=collider.shape,
            pose=api.models.Pose(
                position=api.models.Vector3d(
                    [
                        -300,
                        collider.pose.position[1]
                        if collider.pose and collider.pose.position
                        else 0,
                        collider.pose.position[2]
                        if collider.pose and collider.pose.position
                        else 0,
                    ]
                ),
                orientation=api.models.RotationVector(
                    [0.7853981633974484, 0, 0]
                ),  # 45 degrees in radians
            ),
        )
        colliders[f"{name}_rot_x_45"] = rotated

    # Add rotated variants (y-axis 45 degrees)
    for name, collider in list(colliders.items()):
        if "_rot_" not in name:  # Only rotate original objects
            rotated = api.models.Collider(
                shape=collider.shape,
                pose=api.models.Pose(
                    position=api.models.Vector3d(
                        [
                            300,
                            collider.pose.position[1]
                            if collider.pose and collider.pose.position
                            else 0,
                            collider.pose.position[2]
                            if collider.pose and collider.pose.position
                            else 0,
                        ]
                    ),
                    orientation=api.models.RotationVector(
                        [0, 0.7853981633974484, 0]
                    ),  # 45 degrees in radians
                ),
            )
            colliders[f"{name}_rot_y_45"] = rotated

    # Store all colliders
    for name, collider in colliders.items():
        await collision_api.store_collider(cell=cell_name, collider=name, collider2=collider)

    # Define TCP collider geometry
    tool_collider = api.models.Collider(
        shape=api.models.Box(
            size_x=100, size_y=100, size_z=100, shape_type="box", box_type=api.models.BoxType.FULL
        )
    )
    await collision_api.store_collision_tool(
        cell=cell_name, tool="tool_box", request_body={"tool_collider": tool_collider}
    )

    # Define robot link geometries
    robot_link_colliders = await nova.api.motion_group_models_api.get_motion_group_collision_model(
        motion_group_model=motion_group_description.motion_group_model.root
    )
    link_chain = api.models.LinkChain(
        [api.models.Link(robot_link_collider) for robot_link_collider in robot_link_colliders]
    )
    robot_tool = api.models.Tool({"tool_geometry": tool_collider})

    # Assemble scene with all colliders
    collision_setup = api.models.CollisionSetup(
        colliders=api.models.ColliderDictionary(colliders), tool=robot_tool, link_chain=link_chain
    )
    setup_id = "collision_scene"
    await scene_api.store_collision_setup(
        cell=cell_name, setup=setup_id, collision_setup=collision_setup
    )
    return setup_id


@nova.program(
    name="15_collison_world",
    preconditions=nova.ProgramPreconditions(
        controllers=[
            virtual_controller(
                name="ur5",
                manufacturer=api.models.Manufacturer.UNIVERSALROBOTS,
                type=api.models.VirtualControllerTypes.UNIVERSALROBOTS_UR5E,
            )
        ],
        cleanup_controllers=False,
    ),
)
async def test(ctx: nova.ProgramContext):
    async with NovaRerunBridge(ctx.nova) as bridge:
        await bridge.setup_blueprint()

        cell = ctx.nova.cell()
        controller = await cell.controller("ur5")
        # Connect to the controller and activate motion groups
        motion_group = controller[0]
        await bridge.log_safety_zones(motion_group)

        motion_group_description = await motion_group.get_description()

        await build_collision_world(ctx.nova, cell.id, motion_group_description)
        collision_setups = (
            await ctx.nova.api.store_collision_setups_api.list_stored_collision_setups(cell=cell.id)
        )
        collision_setup = list(collision_setups.values())[0]

        bridge.log_collision_setups(collision_setups=collision_setups)

        tcp_names = await motion_group.tcp_names()
        tcp = tcp_names[0]
        current_pose = await motion_group.tcp_pose(tcp)
        target_pose = current_pose @ Pose((100, 0, 0, 0, 0, 0))

        actions = [
            cartesian_ptp(target=target_pose, collision_setup=collision_setup),
            cartesian_ptp(target=current_pose, collision_setup=collision_setup),
        ]
        joint_trajectory = await motion_group.plan(actions=actions, tcp=tcp)

        await bridge.log_trajectory(
            joint_trajectory,
            tcp,
            motion_group,
            collision_setups=extract_collision_setups_from_actions(actions),
        )


if __name__ == "__main__":
    asyncio.run(test())
