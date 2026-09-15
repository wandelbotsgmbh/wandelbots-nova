"""Plan a Cartesian line through a KUKA shoulder singularity.

The example mirrors the low-level planner example with the SDK API:

1. Calculate the pose at a known KUKA shoulder singularity.
2. Build a Cartesian line from 200 mm before that pose to 200 mm after it.
3. Show that default singularity handling fails.
4. Show that adaptive sampling produces an executable trajectory.
5. Execute the adaptive trajectory.

The planner controls Cartesian sampling internally; unlike the low-level
planner API, the SDK does not expose a sampling-time argument.
"""

import asyncio
import math

import nova
from nova import api, run_program
from nova.actions import cartesian_ptp, joint_ptp, linear
from nova.cell import virtual_controller
from nova.exceptions import PlanTrajectoryFailed
from nova.types import MotionSettings, Pose

KUKA_CONTROLLER = "kuka-kr16-r2010"
SINGULARITY_OFFSET_MM = 200.0
SINGULARITY_OFFSET_SIDE_MM = 0
# This is the KUKA shoulder singularity reported by the planner. The start
# joints are the same wrist branch, 200 mm before that singularity pose.
SINGULARITY_JOINTS = (0, -2.5888, 2.0697, 0.0, 0.2, 0)
START_JOINTS = (0, -2.9311, 1.653, 0, 1.2914, 0)


@nova.program(
    name="Adaptive Singularity Handling",
    preconditions=nova.ProgramPreconditions(
        controllers=[
            virtual_controller(
                name=KUKA_CONTROLLER,
                manufacturer=api.models.Manufacturer.KUKA,
                type="kuka-kr16_r2010_2",
                position=[*START_JOINTS, 0.0],
            )
        ],
        cleanup_controllers=False,
    ),
)
async def adaptive_singularity_handling(ctx: nova.ProgramContext) -> None:
    """Plan a line through a KUKA shoulder singularity."""
    controller = await ctx.cell.controller(KUKA_CONTROLLER)
    motion_group = controller[0]
    tcp = "Flange"
    settings = MotionSettings(tcp_velocity_limit=500)

    # Reset the virtual robot so reruns also work when the controller already exists.
    async def move_to_start() -> None:
        positioning_action = joint_ptp(START_JOINTS, settings=settings)
        positioning_trajectory = await motion_group.plan([positioning_action], tcp)
        await motion_group.execute(positioning_trajectory, tcp, actions=[positioning_action])
        await asyncio.sleep(3)

    await move_to_start()

    singularity_poses = await motion_group.forward_kinematics([list(SINGULARITY_JOINTS)], tcp)
    if len(singularity_poses) != 1:
        raise RuntimeError("KUKA forward kinematics returned no singularity pose")
    singularity_pose = singularity_poses[0]
    orientation = singularity_pose.orientation.to_tuple()

    start_pose = Pose(
        (
            singularity_pose.position.x - 2 * SINGULARITY_OFFSET_MM,
            singularity_pose.position.y + SINGULARITY_OFFSET_SIDE_MM,
            singularity_pose.position.z,
            *orientation,
        )
    )
    print(start_pose)
    target_pose = Pose(
        (
            start_pose.position.x + 5 * SINGULARITY_OFFSET_MM,
            start_pose.position.y + SINGULARITY_OFFSET_SIDE_MM,
            start_pose.position.z,
            *start_pose.orientation.to_tuple(),
        )
    )
    print(target_pose)
    actions = [cartesian_ptp(start_pose, settings=settings), linear(target_pose, settings=settings)]

    print("Planning the Cartesian line with default singularity handling...")
    try:
        no_handling_trajectory = await motion_group.plan(
            actions=actions,
            tcp=tcp,
            start_joint_position=START_JOINTS,
            singularity_handling=api.models.SingularityHandling.NONE,
        )
    except PlanTrajectoryFailed as exc:
        print("  Expected: planning failed at the shoulder singularity.")
        error = exc.error
        partial_trajectory = (
            error.joint_trajectory
            if isinstance(error, api.models.PlanTrajectoryFailedResponse)
            else None
        )
        if partial_trajectory is not None and partial_trajectory.joint_positions:
            print(
                "  Executing the viable part of the trajectory with "
                f"{len(partial_trajectory.joint_positions)} samples."
            )
            await motion_group.execute(partial_trajectory, tcp, actions=actions)
            await asyncio.sleep(3)
            print("  Executed the viable part of the trajectory.")
            await move_to_start()
        else:
            print("  No viable part of the trajectory to execute.")
    else:
        print("  Success: the line planned without singularity handling; executing it.")
        await motion_group.execute(no_handling_trajectory, tcp, actions=actions)
        await asyncio.sleep(3)
        print("  Executed the trajectory planned without singularity handling.")
        await move_to_start()

    print("Planning it again with ADAPTIVE_SAMPLING...")
    try:
        trajectory = await motion_group.plan(
            actions=actions,
            tcp=tcp,
            start_joint_position=START_JOINTS,
            singularity_handling=api.models.SingularityHandling.ADAPTIVE_SAMPLING,
        )
    except PlanTrajectoryFailed as exc:
        print("  Planning with adaptive sampling failed at the shoulder singularity.")
        error = exc.error
        partial_trajectory = (
            error.joint_trajectory
            if isinstance(error, api.models.PlanTrajectoryFailedResponse)
            else None
        )
        if partial_trajectory is not None and partial_trajectory.joint_positions:
            print(
                "  Executing the viable part of the trajectory with "
                f"{len(partial_trajectory.joint_positions)} samples."
            )
            await motion_group.execute(partial_trajectory, tcp, actions=actions)
            await asyncio.sleep(3)
            print("  Executed the viable part of the trajectory.")
            await move_to_start()
        else:
            print("  No viable part of the trajectory to execute.")
        return

    if not trajectory.joint_positions:
        raise RuntimeError("Adaptive singularity handling returned an empty trajectory")

    closest_sample = min(
        trajectory.joint_positions, key=lambda joints: math.dist(joints.root, SINGULARITY_JOINTS)
    )
    singularity_distance = math.dist(closest_sample.root, SINGULARITY_JOINTS)

    print(
        "  Success: adaptive sampling produced an executable trajectory with "
        f"{len(trajectory.joint_positions)} samples."
    )
    print(f"  Closest sample to the shoulder singularity: {singularity_distance:.6f} rad.")

    await motion_group.execute(trajectory, tcp, actions=actions)
    await asyncio.sleep(3)
    print("  Executed the adaptive trajectory through the shoulder singularity.")

    await move_to_start()


if __name__ == "__main__":
    run_program(adaptive_singularity_handling)
