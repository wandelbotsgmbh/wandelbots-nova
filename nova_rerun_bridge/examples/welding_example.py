import asyncio

import numpy as np
import rerun as rr
import trimesh

import nova
from nova import Nova, api
from nova.actions import collision_free, linear
from nova.cell import virtual_controller
from nova.program import ProgramPreconditions
from nova.types import MotionSettings, Pose

"""
Simple example to demonstrate how to add a welding part to the collision world and move the robot to a two seams.
"""


async def load_and_transform_mesh(filepath: str, pose: api.models.Pose) -> trimesh.Geometry:
    """Load mesh and transform to desired position."""
    scene = trimesh.load_mesh(filepath, file_type="stl")

    # Create transformation matrix from Pose
    transform = np.eye(4)
    transform[:3, 3] = pose.position.root
    scene.apply_transform(transform)
    return scene


async def log_mesh_to_rerun(scene: trimesh.Trimesh) -> None:
    """Log mesh to rerun visualization."""
    vertices = scene.vertices
    faces = scene.faces
    vertex_normals = scene.vertex_normals
    vertex_colors = np.ones((len(vertices), 3), dtype=np.float32)

    rr.log(
        "motion/welding_benchmark",
        rr.Mesh3D(
            vertex_positions=vertices,
            triangle_indices=faces,
            vertex_normals=vertex_normals,
            albedo_factor=vertex_colors,
        ),
        static=True,
    )


async def add_mesh_to_collision_world(
    collision_api, cell_name: str, scene: trimesh.Trimesh, collider_name: str = "welding_part"
) -> api.models.Collider:
    """Add mesh as convex hull to collision world."""
    # Create convex hull
    convex_hull = scene.convex_hull

    # Create collider from convex hull vertices
    mesh_collider = api.models.Collider(
        shape=api.models.ConvexHull(
            vertices=convex_hull.vertices.tolist(), shape_type="convex_hull"
        ),
        margin=10,  # add 10mm margin to the convex hull
    )

    await collision_api.store_collider(
        cell=cell_name, collider=collider_name, collider2=mesh_collider
    )
    return mesh_collider


async def build_collision_world(
    nova: Nova,
    cell_name: str,
    motion_group_description: api.models.MotionGroupDescription,
    additional_colliders: dict = {},
) -> str:
    """Build collision world with robot, environment and optional additional colliders.

    Args:
        nova: Nova instance
        cell_name: Name of the cell
        motion_group_description: Motion group description
        additional_colliders: Optional dictionary of additional colliders to add
    """
    collision_api = nova._api_client.store_collision_components_api
    scene_api = nova._api_client.store_collision_setups_api

    # define robot base
    base_collider = api.models.Collider(
        shape=api.models.Cylinder(radius=200, height=300, shape_type="cylinder"),
        pose=api.models.Pose(
            position=api.models.Vector3d([0, 0, -155]),
            orientation=api.models.RotationVector([0, 0, 0]),
        ),
    )
    await collision_api.store_collider(cell=cell_name, collider="base", collider2=base_collider)

    # define floor
    floor_collider = api.models.Collider(
        shape=api.models.Box(
            size_x=2000, size_y=2000, size_z=10, shape_type="box", box_type=api.models.BoxType.FULL
        ),
        pose=api.models.Pose(
            position=api.models.Vector3d([0, 0, -310]),
            orientation=api.models.RotationVector([0, 0, 0]),
        ),
    )
    await collision_api.store_collider(cell=cell_name, collider="floor", collider2=floor_collider)

    # define TCP collider geometry
    tool_collider = api.models.Collider(
        shape=api.models.Box(
            size_x=5, size_y=5, size_z=100, shape_type="box", box_type=api.models.BoxType.FULL
        ),
        pose=api.models.Pose(
            position=api.models.Vector3d([0, 0, 50]),
            orientation=api.models.RotationVector([0, 0, 0]),
        ),
    )
    await collision_api.store_collision_tool(
        cell=cell_name, tool="tool_box", request_body={"tool_collider": tool_collider}
    )

    # Prepare colliders dictionary
    colliders = {"base": base_collider, "floor": floor_collider}

    # Add additional colliders if provided
    if additional_colliders:
        colliders.update(additional_colliders)

    # assemble scene
    robot_link_colliders = await nova.api.motion_group_models_api.get_motion_group_collision_model(
        motion_group_model=motion_group_description.motion_group_model.root
    )
    link_chain = api.models.LinkChain(
        [api.models.Link(robot_link_collider) for robot_link_collider in robot_link_colliders]
    )
    robot_tool = api.models.Tool({"tool_geometry": tool_collider})

    collision_setup = api.models.CollisionSetup(
        colliders=api.models.ColliderDictionary(colliders), tool=robot_tool, link_chain=link_chain
    )
    setup_id = "collision_scene"
    await scene_api.store_collision_setup(
        cell=cell_name, setup=setup_id, collision_setup=collision_setup
    )
    return setup_id


async def calculate_seam_poses(mesh_pose: api.models.Pose) -> tuple[Pose, Pose, Pose, Pose]:
    """Calculate seam poses relative to the mesh pose using @ operator.

    Args:
        mesh_pose: Position and orientation of the welding piece
    Returns:
        tuple containing start and end poses for both seams
    """
    # Convert mesh_pose to Pose for @ operator usage
    mesh_transform = Pose(mesh_pose)

    # Define seams in local coordinates (relative to mesh center)
    local_seam1_start = Pose((150, -6, 3, -np.pi / 2 - np.pi / 4, 0, 0))  # -135° around X
    local_seam1_end = Pose((30, -6, 3, -np.pi / 2 - np.pi / 4, 0, 0))
    local_seam2_start = Pose((150, 6, 3, np.pi / 2 + np.pi / 4, 0, 0))  # 135° around X
    local_seam2_end = Pose((30, 6, 3, np.pi / 2 + np.pi / 4, 0, 0))

    # Transform to global coordinates using @ operator
    seam1_start = mesh_transform @ local_seam1_start
    seam1_end = mesh_transform @ local_seam1_end
    seam2_start = mesh_transform @ local_seam2_start
    seam2_end = mesh_transform @ local_seam2_end

    return seam1_start, seam1_end, seam2_start, seam2_end


@nova.program(
    name="14_welding_example",
    viewer=nova.viewers.Rerun(application_id="14_welding_example"),
    preconditions=ProgramPreconditions(
        controllers=[
            virtual_controller(
                name="ur10",
                manufacturer=api.models.Manufacturer.UNIVERSALROBOTS,
                type=api.models.VirtualControllerTypes.UNIVERSALROBOTS_MINUS_UR10E,
            )
        ],
        cleanup_controllers=False,
    ),
)
async def test():
    async with Nova() as nova:
        # Define position for the welding part
        mesh_pose = api.models.Pose(
            position=api.models.Vector3d([500, 0, -300]),
            orientation=api.models.RotationVector([0, 0, 0]),
        )  # in front of robot, on floor

        # Load and transform mesh
        scene = await load_and_transform_mesh(
            "nova_rerun_bridge/example_data/Welding_Benchmark_USA_01.stl", mesh_pose
        )

        # Log to rerun
        await log_mesh_to_rerun(scene)

        cell = nova.cell()
        controller = await cell.controller("ur10")

        await nova.api.virtual_controller_api.set_virtual_controller_mounting(
            cell="cell",
            controller=controller.controller_id,
            motion_group=f"0@{controller.controller_id}",
            coordinate_system=api.models.CoordinateSystem(
                coordinate_system="world",
                name="mounting",
                reference_uid="",
                position=api.models.Vector3d([0, 0, 0]),
                orientation=api.models.Orientation([0, 0, 0]),
                orientation_type=api.models.OrientationType.EULER_ANGLES_EXTRINSIC_XYZ,
            ),
        )

        await nova.api.virtual_controller_api.add_virtual_controller_tcp(
            cell="cell",
            controller="ur10",
            motion_group=f"0@{controller.controller_id}",
            robot_tcp=api.models.RobotTcp(
                id="torch",
                position=api.models.Vector3d([0, 0, 100]),
                orientation=api.models.Orientation([0, 0, 0]),
                orientation_type=api.models.OrientationType.EULER_ANGLES_EXTRINSIC_XYZ,
            ),
        )

        await asyncio.sleep(5)

        # Connect to the controller and activate motion groups
        async with controller[0] as motion_group:
            tcp = "torch"

            motion_group_description: api.models.MotionGroupDescription = (
                await motion_group.get_description(tcp=tcp)
            )

            # Add mesh to collision world
            mesh_collider = await add_mesh_to_collision_world(
                nova._api_client.store_collision_components_api, "cell", scene
            )

            # Build collision world with welding part included
            collision_setup_id = await build_collision_world(
                nova,
                "cell",
                motion_group_description,
                additional_colliders={"welding_part": mesh_collider},
            )
            scene_api = nova.api.store_collision_setups_api
            collision_setup = await scene_api.get_stored_collision_setup(
                cell="cell", setup=collision_setup_id
            )
            # Calculate seam positions based on mesh pose
            seam1_start, seam1_end, seam2_start, seam2_end = await calculate_seam_poses(mesh_pose)

            # Define approach offset in local coordinates
            approach_offset = Pose((0, 0, -60, 0, 0, 0))

            # Create approach and departure poses using @ operator
            seam1_approach = seam1_start @ approach_offset
            seam1_departure = seam1_end @ approach_offset
            seam2_approach = seam2_start @ approach_offset
            seam2_departure = seam2_end @ approach_offset

            welding_actions = [
                # First seam
                collision_free(
                    target=seam1_approach,
                    collision_setup=collision_setup,
                    settings=MotionSettings(tcp_velocity_limit=30),
                ),
                linear(
                    target=seam1_start,
                    settings=MotionSettings(tcp_velocity_limit=30, blend_radius=10),
                ),
                linear(
                    target=seam1_end,
                    settings=MotionSettings(tcp_velocity_limit=30, blend_radius=10),
                ),
                linear(
                    target=seam1_departure,
                    settings=MotionSettings(tcp_velocity_limit=30, blend_radius=10),
                ),
                # Move to second seam
                collision_free(
                    target=seam2_approach,
                    collision_setup=collision_setup,
                    settings=MotionSettings(tcp_velocity_limit=30),
                ),
                # Second seam with collision checking
                linear(
                    target=seam2_start,
                    settings=MotionSettings(tcp_velocity_limit=30, blend_radius=10),
                ),
                linear(
                    target=seam2_end,
                    settings=MotionSettings(tcp_velocity_limit=30, blend_radius=10),
                ),
                linear(
                    target=seam2_departure,
                    settings=MotionSettings(tcp_velocity_limit=30, blend_radius=10),
                ),
                collision_free(
                    target=(0, -np.pi / 2, np.pi / 2, 0, 0, 0),
                    collision_setup=collision_setup,
                    settings=MotionSettings(tcp_velocity_limit=30),
                ),
            ]

            await motion_group.plan(
                welding_actions, tcp=tcp, start_joint_position=(0, -np.pi / 2, np.pi / 2, 0, 0, 0)
            )


if __name__ == "__main__":
    asyncio.run(test())
