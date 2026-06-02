"""
Two-Robot Multi-Motion-Group RRT Example

Two virtual KUKA KR16 R2010 robots placed 1m apart along the Y axis.
Collision-free joint trajectories are planned simultaneously using the
Nova multi-motion-group RRT-Connect API, which checks inter-robot collisions
at every explored state. The resulting time-synchronized trajectory is then
executed on both virtual controllers concurrently.

This demonstrates:
- Setting up two virtual robot controllers in one cell
- Using the multi-motion-group collision-free planning API (no external deps)
- Coordinating both robots so they never collide with each other
- Executing pre-planned trajectories on both robots concurrently
"""

import asyncio

from pydantic import Field

from nova import api, run_program
from nova.actions import joint_ptp
from nova.cell import virtual_controller
from nova.cell.motion_group import MotionGroup
from nova.types import MotionSettings
from nova_rerun_bridge.trajectory import log_multi_motion_group_trajectory
ROBOT_SEPARATION_MM = 1000.0

# Safe neutral start: both arms point in +X, away from each other.
START_JOINTS = (0.0, -1.0, 1.0, 0.0, 0.5, 0.0)

# Robot 1 (at Y=0) swings J1 to +π/2 → arm extends toward +Y, into robot 2's space.
# Robot 2 (at Y=1000mm) swings J1 to -π/2 → arm extends toward -Y, into robot 1's space.
# Without collision avoidance both TCPs would meet at the midpoint (~Y=500mm).
TARGET_JOINTS_1 = (1.5708, -1.0, 1.0, 0.0, 0.5, 0.0)   # robot 1 → toward +Y
TARGET_JOINTS_2 = (-1.5708, -1.0, 1.0, 0.0, 0.5, 0.0)  # robot 2 → toward -Y

# ROBOT_SEPARATION_MM defined above


async def set_robot_base(motion_group: MotionGroup, x: float, y: float, z: float) -> None:
    """Move a virtual robot's base (mounting) to the given world-frame position (mm)."""
    coord_sys_id = f"base_{motion_group._controller_id}"
    await motion_group._api_client.virtual_controller_api.set_virtual_controller_mounting(
        cell=motion_group._cell,
        controller=motion_group._controller_id,
        motion_group=motion_group.id,
        coordinate_system=api.models.CoordinateSystem(
            coordinate_system=coord_sys_id,
            name=coord_sys_id,
            reference_coordinate_system="",  # world frame
            position=api.models.Vector3d([x, y, z]),
            orientation=api.models.Orientation([0.0, 0.0, 0.0]),
            orientation_type=api.models.OrientationType.ROTATION_VECTOR,
        ),
    )


def _max_joint_accelerations(
    description: api.models.MotionGroupDescription,
) -> tuple[float, ...]:
    """Return the per-joint maximum accelerations from the motion group description."""
    auto_limits = description.operation_limits.auto_limits if description.operation_limits else None
    joints = auto_limits.joints if auto_limits else None
    if not joints:
        raise ValueError("Motion group description has no auto joint limits")
    return tuple(j.acceleration or 0.0 for j in joints)


async def reset_robot(motion_group: MotionGroup, tcp: str, label: str) -> None:
    description = await motion_group.get_description()
    max_joint_accelerations = _max_joint_accelerations(description)
    normal = MotionSettings(
        tcp_velocity_limit=100,
        joint_acceleration_limits=tuple(a * 0.5 for a in max_joint_accelerations),
    )
    print(f"[{label}] moving to start position...")
    await motion_group.plan_and_execute(joint_ptp(START_JOINTS, settings=normal), tcp)
    print(f"[{label}] done.")




async def plan_multi_robot_rrt(ctx, 
    motion_group_1: MotionGroup,
    motion_group_2: MotionGroup,
    tcp_1: str,
    tcp_2: str,
) -> api.models.MultiJointTrajectory | None:
    """Plan collision-free, time-synchronized paths for both robots simultaneously.

    The multi-motion-group RRT-Connect endpoint coordinates both arms in a shared
    joint space, checking inter-robot collisions at every explored state. On success
    it returns a single trajectory with shared timestamps so both robots move in a
    temporally coordinated fashion.
    """
    # Fetch motion group setups and collision link chains for both robots in parallel
    setup_1, setup_2, link_chain_1, link_chain_2 = await asyncio.gather(
        motion_group_1.get_setup(tcp_name=tcp_1),
        motion_group_2.get_setup(tcp_name=tcp_2),
        motion_group_1.get_default_collision_link_chain(),
        motion_group_2.get_default_collision_link_chain(),
    )

    request = api.models.MultiSearchCollisionFreeRequest(
        motion_group_setups_by_motion_group_key=api.models.MotionGroupSetupDictionary({
            motion_group_1.id: setup_1,
            motion_group_2.id: setup_2,
        }),
        path_definitions_by_motion_group_key=api.models.JointPTPMotionDictionary({
            motion_group_1.id: api.models.JointPTPMotion(
                start_joint_position=api.models.DoubleArray(list(START_JOINTS)),
                target_joint_position=api.models.DoubleArray(list(TARGET_JOINTS_1)),
            ),
            motion_group_2.id: api.models.JointPTPMotion(
                start_joint_position=api.models.DoubleArray(list(START_JOINTS)),
                target_joint_position=api.models.DoubleArray(list(TARGET_JOINTS_2)),
            ),
        }),
        # Cross-group collision setup: each robot's link chain is checked against
        # the other robot's links, providing robot-robot collision avoidance.
        collision_setups=api.models.MultiCollisionSetupDictionary({
            "world": api.models.MultiCollisionSetup(
                collision_motion_groups_by_motion_group_key=api.models.CollisionMotionGroupDictionary({
                    motion_group_1.id: api.models.CollisionMotionGroup(
                        link_chain=link_chain_1,
                        self_collision_detection=True,
                    ),
                    motion_group_2.id: api.models.CollisionMotionGroup(
                        link_chain=link_chain_2,
                        self_collision_detection=True,
                    ),
                }),
            ),
        }),
        algorithm_settings=api.models.RRTConnectAlgorithm(
            max_iterations=20000,
            max_step_size=0.1,
            apply_smoothing=True,
            apply_blending=True,
        ),
    )

    print("Planning collision-free paths for both robots simultaneously (RRT-Connect)...")
    response = await motion_group_1._api_client.trajectory_planning_api.search_collision_free_multi_motion_group(
        cell=motion_group_1._cell,
        multi_search_collision_free_request=request,
    )

    if response.response is None or isinstance(
        response.response, api.models.PlanCollisionFreeFailedResponse
    ):
        print(f"ERROR: RRT planning failed — {response.response}")
        return None

    await log_multi_motion_group_trajectory(ctx.nova, response.response, request.motion_group_setups_by_motion_group_key, request.collision_setups)

    return response.response


async def execute_robot_from_multi_trajectory(
    motion_group: MotionGroup,
    multi_trajectory: api.models.MultiJointTrajectory,
    tcp: str,
    label: str,
    count: int,
    target_joints: tuple[float, ...],
) -> None:
    """Execute the pre-planned multi-robot trajectory."""
    description = await motion_group.get_description()
    max_joint_accelerations = _max_joint_accelerations(description)
    fast = MotionSettings(
        tcp_velocity_limit=250,
        joint_acceleration_limits=max_joint_accelerations,
    )

    # Extract this robot's joint positions from the shared multi-trajectory
    joint_positions = multi_trajectory.joint_positions_by_motion_group_key.root[motion_group.id].root
    single_trajectory = api.models.JointTrajectory(
        joint_positions=joint_positions,
        times=multi_trajectory.times,
        locations=multi_trajectory.locations,
    )
    # Actions represent the motion endpoints — used by Nova for viewer/movement controller
    rrt_actions = [
        joint_ptp(START_JOINTS, settings=fast),
        joint_ptp(target_joints, settings=fast),
    ]

    print(f"[{label}] executing {count} RRT cycle(s) — trajectory: {multi_trajectory.times[-1]:.2f}s")
    for i in range(count):
        print(f"[{label}] movement {i + 1}/{count}")
        await motion_group.execute(single_trajectory, tcp, actions=rrt_actions)
        await asyncio.sleep(1)
    print(f"[{label}] done.")


@nova.program(
    id="start_here_two_robots_rrt",
    name="Start Here — Two Robots RRT",
    viewer=nova.viewers.Rerun(),
    preconditions=nova.ProgramPreconditions(
        controllers=[
            virtual_controller(
                name="kuka-robot-1",
                manufacturer=api.models.Manufacturer.KUKA,
                type="kuka-kr16_r2010_2",
            ),
            virtual_controller(
                name="kuka-robot-2",
                manufacturer=api.models.Manufacturer.KUKA,
                type="kuka-kr16_r2010_2",
            ),
        ],
        cleanup_controllers=False,
    ),
)
async def start(
    ctx: nova.ProgramContext,
    count: int = Field(
        default=1, ge=1, le=10, description="The number of times to repeat the movement"
    ),
):
    
    """Move both robots to the safe neutral start position."""
    cell = ctx.cell

    controller_1 = await cell.controller("kuka-robot-1")
    controller_2 = await cell.controller("kuka-robot-2")
    motion_group_1 = controller_1[0]
    motion_group_2 = controller_2[0]

    print("Positioning robots 1m apart...")
    await asyncio.gather(
        set_robot_base(motion_group_1, x=0, y=0, z=0),
        set_robot_base(motion_group_2, x=0, y=ROBOT_SEPARATION_MM, z=0),
    )
    await asyncio.sleep(5)

    tcp_names_1, tcp_names_2 = await asyncio.gather(
        motion_group_1.tcp_names(),
        motion_group_2.tcp_names(),
    )
    tcp_1, tcp_2 = tcp_names_1[0], tcp_names_2[0]

    await asyncio.gather(
        reset_robot(motion_group_1, tcp_1, "robot-1 (Y=0mm)"),
        reset_robot(motion_group_2, tcp_2, "robot-2 (Y=1000mm)"),
    )

    print("Both robots reset to start position.")

    """Control two robots 1m apart with coordinated, inter-robot collision-free RRT paths."""
    cycle = ctx.cycle(extra={"app": "visual-studio-code"})

    # Place robot 1 at world origin and robot 2 exactly 1 m away along Y
    print("Positioning robots 1m apart...")
    await asyncio.gather(
        set_robot_base(motion_group_1, x=0, y=0, z=0),
        set_robot_base(motion_group_2, x=0, y=ROBOT_SEPARATION_MM, z=0),
    )
    # Give both virtual controllers time to restart with the new mounting
    await asyncio.sleep(5)

    tcp_names_1, tcp_names_2 = await asyncio.gather(
        motion_group_1.tcp_names(),
        motion_group_2.tcp_names(),
    )
    tcp_1, tcp_2 = tcp_names_1[0], tcp_names_2[0]

    # Plan collision-free, time-synchronized paths for both robots at once
    multi_trajectory = await plan_multi_robot_rrt(ctx,motion_group_1, motion_group_2, tcp_1, tcp_2)
    if multi_trajectory is None:
        print("Aborting: RRT planning failed.")
        return

    print(f"Planning successful! Trajectory duration: {multi_trajectory.times[-1]:.2f}s")

    await cycle.start()

    await asyncio.gather(
        execute_robot_from_multi_trajectory(
            motion_group_1, multi_trajectory, tcp_1, "robot-1 (Y=0mm)", count, TARGET_JOINTS_1
        ),
        execute_robot_from_multi_trajectory(
            motion_group_2, multi_trajectory, tcp_2, "robot-2 (Y=1000mm)", count, TARGET_JOINTS_2
        ),
    )

    await cycle.finish()
    print("Both robots completed!")


if __name__ == "__main__":
    run_program(start)
