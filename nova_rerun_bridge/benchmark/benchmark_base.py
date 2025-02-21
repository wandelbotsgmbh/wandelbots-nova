import asyncio
import time
from dataclasses import dataclass
from typing import Any, Protocol

import numpy as np
from wandelbots_api_client import models
from wandelbots_api_client.models import (
    CoordinateSystem,
    RotationAngles,
    RotationAngleTypes,
    Vector3d,
)

from nova.core.motion_group import MotionGroup
from nova.core.nova import Nova
from nova.types import Pose
from nova.types.vector3d import Vector3d as Vector3d_nova
from nova_rerun_bridge import NovaRerunBridge
from nova_rerun_bridge.benchmark.datasets import motion_benchmaker_raw
from nova_rerun_bridge.benchmark.log_successful_planning import log_successful_planning
from nova_rerun_bridge.benchmark.robometrics_helper import (
    convert_position,
    create_box_collider,
    create_cylinder_collider,
    quaternion_to_angle_axis,
)


@dataclass
class NovaMetrics:
    """Metrics for benchmark results."""

    success: bool = False
    time: float = 0.0
    position_error: float = 0.0
    orientation_error: float = 0.0
    motion_time: float = 0.0


class BenchmarkStrategy(Protocol):
    """Protocol for benchmark strategies."""

    name: str

    async def plan(
        self,
        motion_group: MotionGroup,
        target: Pose,
        collision_scene: models.CollisionScene,
        tcp: str,
        optimizer_setup: models.OptimizerSetup,
        nova: Nova,
        start_joint_position: tuple[float, ...],
    ) -> Any: ...


async def setup_collision_scene(
    nova: Nova,
    obstacles: dict[str, Any],
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


def print_separator(char="=", width=40):
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
    results,
    problem_results,
    current_key=None,
    total_problems=0,
    current_problem=0,
    path_to_rrd=None,
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
    if path_to_rrd:
        print(f"💾 Logged rerun scene: {path_to_rrd}")
        print_separator()


async def run_single_benchmark(strategy: BenchmarkStrategy):
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
        problem_results: dict[str, list[NovaMetrics]] = {}
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

                        # Use the provided strategy
                        trajectory = await strategy.plan(
                            motion_group=motion_group,
                            target=start_pose,
                            collision_scene=collision_scene,
                            tcp=tcp,
                            optimizer_setup=robot_setup,
                            nova=nova,
                            start_joint_position=(0, -np.pi / 2, np.pi / 2, 0, 0, 0),
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

                        path_to_rrd = await log_successful_planning(
                            bridge,
                            strategy.name,
                            collision_scene_id,
                            problem,
                            key,
                            i,
                            trajectory,
                            tcp,
                            motion_group,
                        )

                        print_progressive_statistics(
                            results,
                            problem_results,
                            key,
                            total_problems,
                            current_problem,
                            path_to_rrd,
                        )

                except Exception:
                    print(f"\nFailed planning: {key} - {i}")
                    failed_metrics = NovaMetrics()
                    results.append(failed_metrics)
                    problem_results[key].append(failed_metrics)

                    print_progressive_statistics(
                        results, problem_results, key, total_problems, current_problem
                    )

        return problem_results, results


async def run_benchmark(strategy: BenchmarkStrategy):
    """Run benchmark for a specific strategy."""
    print(f"\n🚀 Running benchmark for {strategy.name}")
    print_separator("=")

    problem_results, results = await run_single_benchmark(strategy)

    # Print results for this strategy
    print(f"\n📊 Results for {strategy.name}")
    print_separator("=")

    for key, prob_results in problem_results.items():
        print_statistics(prob_results, f"Scene: {key}")

    print_separator("=")
    print(f"📈 Overall Results for {strategy.name}")
    print_separator("=")
    print_statistics(results, f"Complete Benchmark - {strategy.name}")
