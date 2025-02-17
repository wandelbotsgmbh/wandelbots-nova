import asyncio
import time
from typing import Any, Dict, List

import numpy as np
import rerun as rr
from robometrics.datasets import demo_raw
from scipy.spatial.transform import Rotation
from wandelbots_api_client import models
from wandelbots_api_client.models import (
    CoordinateSystem,
    RotationAngles,
    RotationAngleTypes,
    Vector3d,
)

from nova.actions.motions import collision_free
from nova.core.nova import Nova
from nova.types import Pose
from nova.types.vector3d import Vector3d as Vector3d_nova
from nova_rerun_bridge import NovaRerunBridge


def m_to_mm(value: float) -> float:
    """Convert centimeters to millimeters."""
    return value * 1000.0


def convert_position(position: List[float]) -> List[float]:
    """Convert position coordinates from m to mm."""
    return [m_to_mm(p) for p in position]


def quaternion_to_angle_axis(quaternion: List[float]) -> List[float]:
    """Convert quaternion [w, x, y, z] to angle-axis [rx, ry, rz].

    Args:
        quaternion: Quaternion in [w, x, y, z] format as used by robometrics
    Returns:
        Angle-axis representation [rx, ry, rz] in radians
    """
    # Convert [w,x,y,z] to [x,y,z,w] for scipy
    w, x, y, z = quaternion
    rot = Rotation.from_quat([x, y, z, w])
    return rot.as_rotvec()


def convert_pose_quaternion(pose: List[float]) -> tuple[List[float], List[float]]:
    """Convert pose from robometrics format to Nova format.

    The pose transformation order is:
    1. Convert position to mm
    2. Calculate orientation (quaternion to angle-axis)

    Args:
        pose: [x, y, z, w, x, y, z] list containing position in meters
             and quaternion in [w,x,y,z] format
    Returns:
        tuple of (position_mm, orientation_rad)
    """
    # First convert position from meters to mm
    position = convert_position(pose[:3])

    # Then handle orientation - quaternion already in [w,x,y,z] format
    angles = quaternion_to_angle_axis(pose[3:])

    return position, angles


def create_box_collider(name: str, cube: Dict[str, Any]) -> tuple[str, models.Collider]:
    """Create a box collider with mm dimensions using ConvexHull2."""
    position, orientation = convert_pose_quaternion(cube["pose"])
    dims = convert_position(cube["dims"])

    # Create box vertices (8 corners)
    half_x, half_y, half_z = [d / 2 for d in dims]
    vertices = [
        [-half_x, -half_y, -half_z],
        [-half_x, -half_y, half_z],
        [-half_x, half_y, -half_z],
        [-half_x, half_y, half_z],
        [half_x, -half_y, -half_z],
        [half_x, -half_y, half_z],
        [half_x, half_y, -half_z],
        [half_x, half_y, half_z],
    ]

    # Apply rotation to vertices first
    rot = Rotation.from_rotvec(np.array(orientation))
    rotated_vertices = [rot.apply(v) for v in vertices]

    return name, models.Collider(
        shape=models.ColliderShape(
            models.ConvexHull2(vertices=rotated_vertices, shape_type="convex_hull")
        ),
        pose=models.Pose2(
            position=position,
            orientation=[0, 0, 0],  # Orientation already applied to vertices
        ),
    )


def create_cylinder_collider(name: str, cylinder: Dict[str, Any]) -> tuple[str, models.Collider]:
    """Create a cylinder collider with mm dimensions using ConvexHull2."""
    position, orientation = convert_pose_quaternion(cylinder["pose"])
    radius = m_to_mm(cylinder["radius"])
    height = m_to_mm(cylinder["height"])

    # Create cylinder vertices (discretized circle at top and bottom)
    num_points = 16  # Number of points to approximate the circular cross-section
    angles = np.linspace(0, 2 * np.pi, num_points, endpoint=False)

    # Create points for top and bottom circles
    top_points = [[radius * np.cos(a), radius * np.sin(a), height / 2] for a in angles]
    bottom_points = [[radius * np.cos(a), radius * np.sin(a), -height / 2] for a in angles]
    vertices = top_points + bottom_points

    # Apply rotation to vertices first
    rot = Rotation.from_rotvec(np.array(orientation))
    rotated_vertices = [rot.apply(v) for v in vertices]

    return name, models.Collider(
        shape=models.ColliderShape(
            models.ConvexHull2(vertices=rotated_vertices, shape_type="convex_hull")
        ),
        pose=models.Pose2(
            position=position,
            orientation=[0, 0, 0],  # Orientation already applied to vertices
        ),
    )


class NovaMetrics:
    def __init__(
        self, success=False, time=0.0, position_error=0.0, orientation_error=0.0, motion_time=0.0
    ):
        self.success = success
        self.time = time
        self.position_error = position_error
        self.orientation_error = orientation_error
        self.motion_time = motion_time


async def setup_collision_scene(
    nova: Nova,
    obstacles: Dict[str, Any],
    cell_name: str,
    motion_group_type: str,
    robot_setup: models.OptimizerSetup,
    scene_key: str,
) -> str:
    """Convert robometrics obstacles to Nova collision world format."""
    collision_api = nova._api_client.store_collision_components_api
    scene_api = nova._api_client.store_collision_scenes_api

    colliders = {}

    # Convert cuboid obstacles
    if "cuboid" in obstacles:
        for name, cube in obstacles["cuboid"].items():
            name, collider = create_box_collider(name, cube)
            colliders[name] = collider

    # Convert cylinder obstacles
    if "cylinder" in obstacles:
        for name, cylinder in obstacles["cylinder"].items():
            name, collider = create_cylinder_collider(name, cylinder)
            colliders[name] = collider

    # Define TCP collider geometry
    tool_collider = models.Collider(
        shape=models.ColliderShape(
            models.Box2(
                size_x=0.1,  # 10cm box for TCP
                size_y=0.1,
                size_z=0.1,
                shape_type="box",
                box_type="FULL",
            )
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

    # Create and store collision scene with motion group
    scene = models.CollisionScene(
        colliders=colliders,
        motion_groups={
            motion_group_type: models.CollisionMotionGroup(
                tool={"tool_geometry": tool_collider}, link_chain=robot_link_colliders
            )
        },
    )
    scene_id = f"benchmark_scene_{scene_key}"
    await scene_api.store_collision_scene(
        cell_name, scene_id, models.CollisionSceneAssembly(scene=scene)
    )

    return scene_id


async def run_benchmark():
    async with Nova() as nova, NovaRerunBridge(nova, spawn=False) as bridge:
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
                position=Vector3d(x=0, y=0, z=10),
                rotation=RotationAngles(
                    angles=[0, 0, 0], type=RotationAngleTypes.EULER_ANGLES_EXTRINSIC_XYZ
                ),
            ),
        )

        # NC-1047
        await asyncio.sleep(3)

        # Load benchmark datasets
        problems = demo_raw()

        results = []
        for key, scene_problems in problems.items():
            print(f"\nProcessing scene: {key}")
            for i, problem in enumerate(scene_problems):
                rr.init(application_id="nova", recording_id=f"nova_{key}_{i}", spawn=True)
                await bridge.setup_blueprint()

                print(f"Problem {i + 1}/{len(scene_problems)}", end="\r")
                start_time = time.time()

                # Get start and goal configurations
                q_start = problem["start"]
                goal_pose = (
                    problem["goal_pose"]["position_xyz"] + problem["goal_pose"]["quaternion_wxyz"]
                )

                goal_position = convert_position(problem["goal_pose"]["position_xyz"])
                goal_quat = problem["goal_pose"]["quaternion_wxyz"]

                # Create arrow to show goal pose
                # Convert quaternion to rotation matrix
                w, x, y, z = goal_quat
                rot = Rotation.from_quat([x, y, z, w])  # [x,y,z,w] format for scipy

                # Create arrow pointing in x direction (length 100mm)
                arrow_length = 50.0  # mm
                arrow_vectors = [
                    rot.apply([arrow_length, 0, 0]),  # X axis - Red
                    rot.apply([0, arrow_length, 0]),  # Y axis - Green
                    rot.apply([0, 0, arrow_length]),  # Z axis - Blue
                ]

                coordinate_colors = np.array(
                    [
                        [1.0, 0.125, 0.376, 1.0],  # #ff2060 - Red/Pink for X
                        [0.125, 0.875, 0.502, 1.0],  # #20df80 - Green for Y
                        [0.125, 0.502, 1.0, 1.0],  # #2080ff - Blue for Z
                    ]
                )

                rr.log(
                    "motion/target_orientation",
                    rr.Arrows3D(
                        origins=[goal_position] * 3,  # Same origin for all arrows
                        vectors=arrow_vectors,
                        colors=coordinate_colors,
                        radii=[2.5] * 3,
                    ),
                    static=True,
                )

                # Log goal pose position (convert from m to mm)
                goal_position_mm = convert_position(problem["goal_pose"]["position_xyz"])
                rr.log(
                    "motion/target_orientation",
                    rr.Points3D(
                        positions=[goal_position_mm],
                        radii=[5],
                        colors=[(0, 255, 0, 255)],  # Green with full opacity
                    ),
                )

                try:
                    async with controller[0] as motion_group:
                        tcp = "Flange"

                        # Get robot setup for collision scene
                        robot_setup: models.OptimizerSetup = (
                            await motion_group._get_optimizer_setup(tcp=tcp)
                        )

                        # Add collision objects from benchmark
                        collision_scene_id = await setup_collision_scene(
                            nova,
                            problem["obstacles"],
                            "cell",
                            robot_setup.motion_group_type,
                            robot_setup,
                            key,
                        )
                        await bridge.log_collision_scene(collision_scene_id)

                        scene_api = nova._api_client.store_collision_scenes_api
                        collision_scene = await scene_api.get_stored_collision_scene(
                            cell="cell", scene=collision_scene_id
                        )

                        # Convert start configuration to Pose object
                        position = convert_position(problem["goal_pose"]["position_xyz"])
                        orientation = quaternion_to_angle_axis(
                            problem["goal_pose"]["quaternion_wxyz"]
                        )

                        start_pose = Pose(
                            position=Vector3d_nova(x=position[0], y=position[1], z=position[2]),
                            orientation=Vector3d_nova(
                                x=orientation[0], y=orientation[1], z=orientation[2]
                            ),
                        )  # Combine position and orientation

                        # Set start configuration using the converted pose
                        trajectory = await motion_group.plan(
                            [collision_free(target=start_pose, collision_scene=collision_scene)],
                            tcp=tcp,
                        )

                        await bridge.log_trajectory(trajectory, tcp, motion_group)

                        # Calculate metrics
                        end_time = time.time()
                        metrics = NovaMetrics(
                            success=True,
                            time=end_time - start_time,
                            position_error=0,
                            orientation_error=0,
                            motion_time=trajectory.duration if trajectory else 0.0,
                        )
                        results.append(metrics)

                except Exception as e:
                    print(f"\nFailed planning: {e}")
                    results.append(NovaMetrics())

        # Calculate overall statistics
        if results:
            success_rate = sum(r.success for r in results) / len(results)
            successful_results = [r for r in results if r.success]

            if successful_results:
                avg_time = sum(r.time for r in successful_results) / len(successful_results)
                avg_pos_error = sum(r.position_error for r in successful_results) / len(
                    successful_results
                )
                avg_orient_error = sum(r.orientation_error for r in successful_results) / len(
                    successful_results
                )
                avg_motion_time = sum(r.motion_time for r in successful_results) / len(
                    successful_results
                )

                print("\nBenchmark Results:")
                print(f"Success rate: {success_rate:.2%}")
                print(f"Average planning time: {avg_time:.3f}s")
                print(f"Average motion time: {avg_motion_time:.3f}s")
                print(f"Average position error: {avg_pos_error:.3f}m")
                print(f"Average orientation error: {avg_orient_error:.3f}rad")
            else:
                print("\nNo successful results")
        else:
            print("\nNo results collected")


if __name__ == "__main__":
    asyncio.run(run_benchmark())
