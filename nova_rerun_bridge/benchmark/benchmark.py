import asyncio
import time
from typing import Any, Dict

import numpy as np
import rerun as rr
from robometrics.datasets import motion_benchmaker_raw
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
from nova_rerun_bridge.benchmark.robometrics_helper import (
    convert_position,
    create_box_collider,
    create_cylinder_collider,
    quaternion_to_angle_axis,
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


async def log_successful_planning(
    bridge, collision_scene_id, problem, key, i, trajectory, tcp, motion_group
):
    # Log successful planning
    rr.init(application_id="nova", recording_id=f"nova_{key}_{i}", spawn=True)
    await bridge.setup_blueprint()
    await bridge.log_collision_scene(collision_scene_id)

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

    await bridge.log_trajectory(trajectory, tcp, motion_group)


def print_separator(char="=", width=80):
    """Print a separator line."""
    print(char * width)


def print_statistics(results, name="Overall", show_separator=True):
    """Print statistics for a group of results with improved formatting."""
    if show_separator:
        print_separator("-")

    if not results:
        print(f"\n📊 {name} Statistics: No results collected")
        return

    success_rate = sum(r.success for r in results) / len(results)
    successful_results = [r for r in results if r.success]

    print(f"\n📊 {name} Statistics:")
    print(f"├── Total attempts: {len(results)}")
    print(
        f"├── Success rate:  {'🟢' if success_rate > 0.8 else '🟡' if success_rate > 0.5 else '🔴'} {success_rate:.2%}"
    )

    if successful_results:
        avg_time = sum(r.time for r in successful_results) / len(successful_results)
        avg_pos_error = sum(r.position_error for r in successful_results) / len(successful_results)
        avg_orient_error = sum(r.orientation_error for r in successful_results) / len(
            successful_results
        )
        avg_motion_time = sum(r.motion_time for r in successful_results) / len(successful_results)

        print("├── Averages for successful attempts:")
        print(f"│   ├── Planning time:     {avg_time:.3f}s")
        print(f"│   ├── Motion time:       {avg_motion_time:.3f}s")
        print(f"│   ├── Position error:    {avg_pos_error:.3f}m")
        print(f"│   └── Orientation error: {avg_orient_error:.3f}rad")

    if show_separator:
        print_separator("-")


def print_progressive_statistics(
    results, problem_results, current_key=None, total_problems=0, current_problem=0
):
    """Print both current problem and overall statistics with progress indicator."""
    print("\033[2J\033[H")  # Clear screen and move cursor to top
    print_separator()
    print(f"🤖 Benchmark Progress: {current_problem}/{total_problems} problems")
    print_separator()

    # Print current problem statistics if available
    if current_key and current_key in problem_results:
        print_statistics(
            problem_results[current_key], f"Current Scene ({current_key})", show_separator=False
        )

    # Print overall progress
    print_statistics(results, "Overall Progress", show_separator=False)
    print_separator()


async def run_benchmark():
    async with Nova() as nova, NovaRerunBridge(nova, spawn=False) as bridge:
        cell = nova.cell()

        controller = await cell.ensure_virtual_robot_controller(
            "ur10",
            models.VirtualControllerTypes.UNIVERSALROBOTS_MINUS_UR10E,
            models.Manufacturer.UNIVERSALROBOTS,
        )

        # move the robot up to avoid collision with the floor
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
        problems = motion_benchmaker_raw()

        # Store results per problem
        problem_results = {}
        results = []

        total_problems = sum(len(probs) for probs in problems.values())
        current_problem = 0

        for key, scene_problems in problems.items():
            print(f"\nProcessing scene: {key}")
            problem_results[key] = []

            for i, problem in enumerate(scene_problems):
                current_problem += 1

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

                        start_time = time.time()

                        # Set start configuration using the converted pose
                        trajectory = await motion_group.plan(
                            [collision_free(target=start_pose, collision_scene=collision_scene)],
                            tcp=tcp,
                        )

                        # Calculate metrics
                        end_time = time.time()
                        metrics = NovaMetrics(
                            success=True,
                            time=end_time - start_time,
                            position_error=0,  # not supported
                            orientation_error=0,  # not supported
                            motion_time=trajectory.times[-1] if trajectory else 0.0,
                        )
                        results.append(metrics)
                        problem_results[key].append(metrics)

                        print_progressive_statistics(
                            results, problem_results, key, total_problems, current_problem
                        )

                        await log_successful_planning(
                            bridge,
                            collision_scene_id,
                            problem,
                            key,
                            i,
                            trajectory,
                            tcp,
                            motion_group,
                        )

                except Exception:
                    print(f"\nFailed planning: {key} - {i}")
                    failed_metrics = NovaMetrics()
                    results.append(failed_metrics)
                    problem_results[key].append(failed_metrics)

                    print_progressive_statistics(
                        results, problem_results, key, total_problems, current_problem
                    )

        # Print final statistics
        print("\033[2J\033[H")  # Clear screen
        print_separator("=")
        print("🎯 Final Benchmark Results")
        print_separator("=")

        for key, prob_results in problem_results.items():
            print_statistics(prob_results, f"Scene: {key}")

        print_separator("=")
        print("📈 Overall Results")
        print_separator("=")
        print_statistics(results, "Complete Benchmark")


if __name__ == "__main__":
    asyncio.run(run_benchmark())
