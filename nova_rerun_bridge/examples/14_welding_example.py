import asyncio

import numpy as np
import rerun as rr
import trimesh
from wandelbots_api_client.models import (
    CoordinateSystem,
    RobotTcp,
    RotationAngles,
    RotationAngleTypes,
    Vector3d,
)

from nova import MotionSettings
from nova.actions import collision_free, linear
from nova.api import models
from nova.core.exceptions import PlanTrajectoryFailed
from nova.core.nova import Nova
from nova.types import Pose
from nova_rerun_bridge import NovaRerunBridge

"""
Simple example to demonstrate how to add a welding part to the collision world and move the robot to a two seams.
"""


async def load_and_transform_mesh(filepath: str, pose: models.Pose2) -> trimesh.Geometry:
    """Load mesh and transform to desired position."""
    scene = trimesh.load_mesh(filepath, file_type="stl")

    # Create transformation matrix from Pose2
    transform = np.eye(4)
    transform[:3, 3] = pose.position
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
) -> models.Collider:
    """Add mesh as convex hull to collision world."""
    # Create convex hull
    convex_hull = scene.convex_hull

    # Create collider from convex hull vertices
    mesh_collider = models.Collider(
        shape=models.ColliderShape(
            models.ConvexHull2(vertices=convex_hull.vertices.tolist(), shape_type="convex_hull")
        ),
        margin=10,  # add 10mm margin to the convex hull
    )

    await collision_api.store_collider(
        cell=cell_name, collider=collider_name, collider2=mesh_collider
    )
    return mesh_collider


async def build_collision_world(
    nova: Nova, cell_name: str, robot_setup: models.OptimizerSetup, additional_colliders: dict = {}
) -> str:
    """Build collision world with robot, environment and optional additional colliders.

    Args:
        nova: Nova instance
        cell_name: Name of the cell
        robot_setup: Robot optimizer setup
        additional_colliders: Optional dictionary of additional colliders to add
    """
    collision_api = nova._api_client.store_collision_components_api
    scene_api = nova._api_client.store_collision_scenes_api

    # define robot base
    base_collider = models.Collider(
        shape=models.ColliderShape(models.Cylinder2(radius=200, height=300, shape_type="cylinder")),
        pose=models.Pose2(position=[0, 0, -155]),
    )
    await collision_api.store_collider(cell=cell_name, collider="base", collider2=base_collider)

    # define floor
    floor_collider = models.Collider(
        shape=models.ColliderShape(
            models.Box2(size_x=2000, size_y=2000, size_z=10, shape_type="box", box_type="FULL")
        ),
        pose=models.Pose2(position=[0, 0, -310]),
    )
    await collision_api.store_collider(cell=cell_name, collider="floor", collider2=floor_collider)

    # define TCP collider geometry
    tool_collider = models.Collider(
        shape=models.ColliderShape(
            models.Box2(size_x=5, size_y=5, size_z=100, shape_type="box", box_type="FULL")
        ),
        pose=models.Pose2(position=[0, 0, 50]),
    )
    await collision_api.store_collision_tool(
        cell=cell_name, tool="tool_box", request_body={"tool_collider": tool_collider}
    )

    # define robot link geometries
    robot_link_colliders = await collision_api.get_default_link_chain(
        cell=cell_name, motion_group_model=robot_setup.motion_group_type
    )
    await collision_api.store_collision_link_chain(
        cell=cell_name, link_chain="robot_links", collider=robot_link_colliders
    )

    # Prepare colliders dictionary
    colliders = {"base": base_collider, "floor": floor_collider}

    # Add additional colliders if provided
    if additional_colliders:
        colliders.update(additional_colliders)

    # assemble scene
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


async def calculate_seam_poses(mesh_pose: models.Pose2) -> tuple[Pose, Pose, Pose, Pose]:
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


async def test():
    async with Nova() as nova, NovaRerunBridge(nova) as bridge:
        await bridge.setup_blueprint()

        # Define position for the welding part
        mesh_pose = models.Pose2(
            position=[500, 0, -300], orientation=[0, 0, 0]
        )  # in front of robot, on floor

        # Load and transform mesh
        scene = await load_and_transform_mesh(
            "nova_rerun_bridge/example_data/Welding_Benchmark_USA_01.stl", mesh_pose
        )

        # Log to rerun
        await log_mesh_to_rerun(scene)

        cell = nova.cell()
        controller = await cell.ensure_virtual_robot_controller(
            "ur10",
            models.VirtualControllerTypes.UNIVERSALROBOTS_MINUS_UR10E,
            models.Manufacturer.UNIVERSALROBOTS,
        )

        await nova._api_client.virtual_robot_setup_api.set_virtual_robot_mounting(
            cell="cell",
            controller=controller.controller_id,
            id=0,
            coordinate_system=CoordinateSystem(
                coordinate_system="world",
                name="mounting",
                reference_uid="",
                position=Vector3d(x=0, y=0, z=0),
                rotation=RotationAngles(
                    angles=[0, 0, 0], type=RotationAngleTypes.EULER_ANGLES_EXTRINSIC_XYZ
                ),
            ),
        )

        await nova._api_client.virtual_robot_setup_api.add_virtual_robot_tcp(
            cell="cell",
            controller="ur10",
            id=0,
            robot_tcp=RobotTcp(
                id="torch",
                position=Vector3d(x=0, y=0, z=100),
                rotation=RotationAngles(
                    angles=[0, 0, 0], type=RotationAngleTypes.EULER_ANGLES_EXTRINSIC_XYZ
                ),
            ),
        )

        await asyncio.sleep(5)

        # Connect to the controller and activate motion groups
        async with controller[0] as motion_group:
            await bridge.log_saftey_zones(motion_group)

            tcp = "torch"

            robot_setup: models.OptimizerSetup = await motion_group._get_optimizer_setup(tcp=tcp)

            # Add mesh to collision world
            mesh_collider = await add_mesh_to_collision_world(
                nova._api_client.store_collision_components_api, "cell", scene
            )

            # Build collision world with welding part included
            collision_scene_id = await build_collision_world(
                nova, "cell", robot_setup, additional_colliders={"welding_part": mesh_collider}
            )
            scene_api = nova._api_client.store_collision_scenes_api
            collision_scene = await scene_api.get_stored_collision_scene(
                cell="cell", scene=collision_scene_id
            )
            await bridge.log_collision_scene(collision_scene_id)

            # Calculate seam positions based on mesh pose
            seam1_start, seam1_end, seam2_start, seam2_end = await calculate_seam_poses(mesh_pose)

            # Define approach offset in local coordinates
            approach_offset = Pose((0, 0, -60, 0, 0, 0))

            # Create approach and departure poses using @ operator
            seam1_approach = seam1_start @ approach_offset
            seam1_departure = seam1_end @ approach_offset
            seam2_approach = seam2_start @ approach_offset
            seam2_departure = seam2_end @ approach_offset

            try:
                welding_actions = [
                    # First seam
                    collision_free(
                        target=seam1_approach,
                        collision_scene=collision_scene,
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
                        collision_scene=collision_scene,
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
                        collision_scene=collision_scene,
                        settings=MotionSettings(tcp_velocity_limit=30),
                    ),
                ]

                trajectory_plan_combined = await motion_group.plan(
                    welding_actions,
                    tcp=tcp,
                    start_joint_position=(0, -np.pi / 2, np.pi / 2, 0, 0, 0),
                )

                await bridge.log_actions(welding_actions)
                await bridge.log_trajectory(trajectory_plan_combined, tcp, motion_group)

            except PlanTrajectoryFailed as e:
                await bridge.log_trajectory(e.error.joint_trajectory, tcp, motion_group)
                await bridge.log_error_feedback(e.error.error_feedback)
                raise


if __name__ == "__main__":
    asyncio.run(test())
