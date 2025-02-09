import asyncio

import numpy as np
import rerun as rr
import trimesh
from wandelbots_api_client.models import (
    PlanCollisionFreePTPRequest,
    RobotTcp,
    RotationAngles,
    RotationAngleTypes,
    Vector3d,
)

from nova import MotionSettings
from nova.actions import Linear, jnt
from nova.api import models
from nova.core.exceptions import PlanTrajectoryFailed
from nova.core.nova import Nova
from nova.types import Pose
from nova_rerun_bridge import NovaRerunBridge

"""
Simple example to demonstrate how to add a welding part to the collision world and move the robot to a two seams.
"""


async def load_and_transform_mesh(filepath: str, pose: models.Pose2) -> trimesh.Trimesh:
    """Load mesh and transform to desired position."""
    scene = trimesh.load(filepath, file_type="stl")

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
        timeless=True,
        static=True,
    )


async def add_mesh_to_collision_world(
    collision_api,
    cell_name: str,
    scene: trimesh.Trimesh,
    pose: models.Pose2,
    collider_name: str = "welding_part",
) -> None:
    """Add mesh as convex hull to collision world."""
    # Create convex hull
    convex_hull = scene.convex_hull

    # Create collider from convex hull vertices
    mesh_collider = models.Collider(
        shape=models.ColliderShape(
            models.ConvexHull2(vertices=convex_hull.vertices.tolist(), shape_type="convex_hull")
        ),
        pose=pose,
    )

    await collision_api.store_collider(
        cell=cell_name, collider=collider_name, collider2=mesh_collider
    )
    return mesh_collider


async def build_collision_world(
    nova: Nova,
    cell_name: str,
    robot_setup: models.OptimizerSetup,
    additional_colliders: dict = None,
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

    # define box around welding part
    sphere_collider = models.Collider(
        shape=models.ColliderShape(
            models.Box2(size_x=400, size_y=50, size_z=100, box_type="FULL", shape_type="box")
        ),
        pose=models.Pose2(position=[500, 0, -250]),
    )
    await collision_api.store_collider(
        cell=cell_name, collider="annoying_obstacle", collider2=sphere_collider
    )

    # define robot base
    base_collider = models.Collider(
        shape=models.ColliderShape(models.Cylinder2(radius=200, height=300, shape_type="cylinder")),
        pose=models.Pose2(position=[0, 0, -300]),
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
    colliders = {
        "base": base_collider,
        "floor": floor_collider,
        "annoying_obstacle": sphere_collider,
    }

    # Add additional colliders if provided
    if additional_colliders:
        colliders.update(additional_colliders)

    # assemble scene
    scene = models.CollisionScene(
        colliders=colliders,
        motion_groups={
            "motion_group": models.CollisionMotionGroup(
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
    mesh_transform = Pose((*mesh_pose.position, *mesh_pose.orientation))

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


async def plan_collision_free_movement(
    nova: Nova,
    robot_setup: models.OptimizerSetup,
    collision_scene: models.CollisionScene,
    start_joints: list[float],
    target_pose: Pose,
) -> models.JointTrajectory:
    """Plan collision-free PTP movement.

    Args:
        nova: Nova instance
        robot_setup: Robot optimizer setup
        collision_scene: Current collision scene
        start_joints: Starting joint positions
        target_pose: Target pose to reach

    Returns:
        Planned joint trajectory
    """
    plan_result = await nova._api_client.motion_api.plan_collision_free_ptp(
        cell="cell",
        plan_collision_free_ptp_request=PlanCollisionFreePTPRequest(
            robot_setup=robot_setup,
            start_joint_position=start_joints,
            target=models.PlanCollisionFreePTPRequestTarget(target_pose._to_wb_pose2()),
            static_colliders=collision_scene.colliders,
            collision_motion_group=collision_scene.motion_groups["motion_group"],
        ),
    )

    if isinstance(plan_result.response.actual_instance, models.PlanTrajectoryFailedResponse):
        raise PlanTrajectoryFailed(plan_result.response.actual_instance)

    return plan_result.response.actual_instance


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
            tcp = "torch"

            robot_setup: models.OptimizerSetup = await motion_group._get_optimizer_setup(tcp=tcp)
            robot_setup.safety_setup.global_limits.tcp_velocity_limit = 200

            # Add mesh to collision world
            mesh_collider = await add_mesh_to_collision_world(
                nova._api_client.store_collision_components_api, "cell", scene, mesh_pose
            )

            # Build collision world with welding part included
            collision_scene_id = await build_collision_world(
                nova, "cell", robot_setup, additional_colliders={"welding_part": mesh_collider}
            )
            scene_api = nova._api_client.store_collision_scenes_api
            collision_scene = await scene_api.get_stored_collision_scene(
                cell="cell", scene=collision_scene_id
            )
            await bridge.log_collision_scenes()

            home = await motion_group.tcp_pose(tcp)

            # Calculate seam positions based on mesh pose
            seam1_start, seam1_end, seam2_start, seam2_end = await calculate_seam_poses(mesh_pose)

            # Define approach offset in local coordinates
            approach_offset = Pose((0, 0, -50, 0, 0, 0))

            # Create approach and departure poses using @ operator
            seam1_approach = seam1_start @ approach_offset
            seam1_departure = seam1_end @ approach_offset
            seam2_approach = seam2_start @ approach_offset
            seam2_departure = seam2_end @ approach_offset

            try:
                # Move to default pose
                default_pose_actions = [jnt(target=[0, -np.pi / 2, np.pi / 2, 0, 0, 0])]
                default_pose_trajectory = await motion_group.plan(default_pose_actions, tcp)
                await bridge.log_actions(default_pose_actions)
                await bridge.log_trajectory(default_pose_trajectory, tcp, motion_group)
                async for _ in motion_group.execute(default_pose_trajectory, tcp, actions=None):
                    pass

                # 1. Collision-free movement to first seam approach
                trajectory1 = await plan_collision_free_movement(
                    nova, robot_setup, collision_scene, await motion_group.joints(), seam1_approach
                )
                await bridge.log_trajectory(trajectory1, tcp, motion_group)
                async for _ in motion_group.execute(trajectory1, tcp, actions=None):
                    pass

                # 2. Normal planning for first seam
                seam1_actions = [
                    Linear(target=seam1_approach),
                    Linear(target=seam1_start),
                    Linear(target=seam1_end),
                    Linear(target=seam1_departure),
                ]
                for action in seam1_actions:
                    action.settings = MotionSettings(tcp_velocity_limit=30, blend_radius=10)
                seam1_trajectory = await motion_group.plan(seam1_actions, tcp)
                await bridge.log_actions(seam1_actions)
                await bridge.log_trajectory(seam1_trajectory, tcp, motion_group)
                async for _ in motion_group.execute(seam1_trajectory, tcp, actions=None):
                    pass

                # 3. Collision-free movement to second seam approach
                trajectory2 = await plan_collision_free_movement(
                    nova, robot_setup, collision_scene, await motion_group.joints(), seam2_approach
                )
                await bridge.log_trajectory(trajectory2, tcp, motion_group)
                async for _ in motion_group.execute(trajectory2, tcp, actions=None):
                    pass

                # 4. Normal planning for second seam
                seam2_actions = [
                    Linear(target=seam2_approach),
                    Linear(target=seam2_start),
                    Linear(target=seam2_end),
                    Linear(target=seam2_departure),
                ]
                for action in seam2_actions:
                    action.settings = MotionSettings(tcp_velocity_limit=30, blend_radius=10)
                seam2_trajectory = await motion_group.plan(seam2_actions, tcp)
                await bridge.log_actions(seam2_actions)
                await bridge.log_trajectory(seam2_trajectory, tcp, motion_group)
                async for _ in motion_group.execute(seam2_trajectory, tcp, actions=None):
                    pass

                # 5. Collision-free movement back home
                trajectory3 = await plan_collision_free_movement(
                    nova, robot_setup, collision_scene, await motion_group.joints(), home
                )
                await bridge.log_trajectory(trajectory3, tcp, motion_group)
                async for _ in motion_group.execute(trajectory3, tcp, actions=None):
                    pass

            except PlanTrajectoryFailed as e:
                await bridge.log_trajectory(e.error.joint_trajectory, tcp, motion_group)
                await bridge.log_error_feedback(e.error.error_feedback)
                raise
            # await cell.delete_robot_controller(controller.controller_id)


if __name__ == "__main__":
    asyncio.run(test())
