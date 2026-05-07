"""
Example: Plan multiple trajectories and merge them with collision-aware blending.

This example demonstrates the MergeTrajectories endpoint which combines
separately planned trajectories into one smooth trajectory with blending
between segments.

Prerequisites:
- Set env variables (you can specify them in an .env file):
    - NOVA_API=<api>
    - NOVA_ACCESS_TOKEN=<token>
"""

import nova
from nova import api, run_program
from nova.actions import joint_ptp, linear
from nova.cell import virtual_controller
from nova.types import Pose


@nova.program(
    name="Merge Trajectories",
    preconditions=nova.ProgramPreconditions(
        controllers=[
            virtual_controller(
                name="ur5e",
                manufacturer=api.models.Manufacturer.UNIVERSALROBOTS,
                type="universalrobots-ur5e",
            )
        ],
        cleanup_controllers=False,
    ),
)
async def merge_trajectories(ctx: nova.ProgramContext):
    nova_client = ctx.nova
    cell = nova_client.cell()
    controller = await cell.controller("ur5e")
    motion_group = controller[0]

    tcp_names = await motion_group.tcp_names()
    tcp = tcp_names[0]

    # Fixed start joints for deterministic behavior
    start_joints = (1.169, -1.57, 1.36, 1.029, 1.289, 1.279)

    # Absolute target poses known to be reachable from start_joints
    pose_1 = Pose((400, 0, 100, 0, 0, 0))
    pose_2 = Pose((400, 300, 100, 0, 0, 0))
    pose_3 = Pose((300, 200, 200, 0, 0, 0))

    # Get motion group setup from controller and adjust TCP velocity to match
    # the SDK's default tcp_velocity_limit (50 mm/s) for consistent merge behavior
    setup = await motion_group.get_setup(tcp)
    if setup.global_limits and setup.global_limits.tcp:
        setup.global_limits.tcp.velocity = 50.0

    # Plan 3 separate trajectories using the same setup as merge
    traj_1 = await motion_group.plan(
        [linear(pose_1)], tcp, start_joint_position=start_joints, motion_group_setup=setup
    )

    end_joints_1 = tuple(traj_1.joint_positions[-1].root)
    traj_2 = await motion_group.plan(
        [linear(pose_2)], tcp, start_joint_position=end_joints_1, motion_group_setup=setup
    )

    end_joints_2 = tuple(traj_2.joint_positions[-1].root)
    traj_3 = await motion_group.plan(
        [linear(pose_3)], tcp, start_joint_position=end_joints_2, motion_group_setup=setup
    )

    # Build merge request with blending between segments
    # Blending on a segment means: blend end of this segment into start of next segment
    # Last segment has no blending (nothing to blend into)
    merge_request = api.models.MergeTrajectoriesRequest(
        motion_group_setup=setup,
        trajectory_segments=[
            api.models.MergeTrajectoriesSegment(
                trajectory=traj_1, blending=api.models.BlendingPosition(position_zone_radius=100.0)
            ),
            api.models.MergeTrajectoriesSegment(
                trajectory=traj_2, blending=api.models.BlendingPosition(position_zone_radius=100.0)
            ),
            api.models.MergeTrajectoriesSegment(trajectory=traj_3),
        ],
    )

    # Call merge trajectories endpoint
    response = await nova_client.api.trajectory_planning_api.merge_trajectories(
        cell=cell.id, merge_trajectories_request=merge_request
    )

    if response.joint_trajectory is None:
        raise RuntimeError("Merge returned no trajectory")
    merged_trajectory = response.joint_trajectory
    print(f"Merged trajectory has {len(merged_trajectory.joint_positions)} points")

    if response.feedback:
        for fb in response.feedback:
            print(f"  Feedback: {fb}")

    # Move to start configuration before executing merged trajectory
    await motion_group.execute(
        await motion_group.plan([joint_ptp(start_joints)], tcp),
        tcp,
        actions=[joint_ptp(start_joints)],
    )

    # Execute the merged trajectory
    motion_iter = motion_group.stream_execute(merged_trajectory, tcp, actions=[])
    async for _ in motion_iter:
        pass


if __name__ == "__main__":
    run_program(merge_trajectories)
