"""Demonstrate the palletizing wrist singularity handling flag.

The flange points down while the tool mimics a palletizing pick-and-drop:
approach, descend to pick, lift, transfer, descend to drop, and lift away.
The Cartesian path crosses the KUKA wrist singularity.

Without special handling, the planner stops at the singularity. With
``PALLETIZING_WRIST``, it bridges the singularity by changing the wrist branch.

Prerequisites:
- A NOVA instance
- ``NOVA_API`` and ``NOVA_ACCESS_TOKEN`` set in the environment
"""

import asyncio

import nova
from nova import api, run_program
from nova.actions import joint_ptp, linear
from nova.cell import virtual_controller
from nova.exceptions import PlanTrajectoryFailed
from nova.types import MotionSettings, Pose

KUKA_CONTROLLER = "kuka-kr16-r2010"
START_JOINTS = (0.0, -1.5, 2.2708, 0.0, 0.8, 0.0)
PICKUP_TRAVEL = 1000
TRANSFER_DISTANCE = 400
DROP_TRAVEL = 1000


@nova.program(
    name="Singularity Handling",
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
async def singularity_handling(ctx: nova.ProgramContext) -> None:
    """Plan and execute a palletizing pick-and-drop through a wrist singularity."""
    controller = await ctx.cell.controller(KUKA_CONTROLLER)
    motion_group = controller[0]
    tcp = "Flange"
    settings = MotionSettings(tcp_velocity_limit=500)

    # Reset the virtual robot to the approach pose so reruns also work when the
    # controller already exists from an earlier execution.
    positioning_action = joint_ptp(START_JOINTS, settings=settings)
    positioning_trajectory = await motion_group.plan([positioning_action], tcp)
    await motion_group.execute(positioning_trajectory, tcp, actions=[positioning_action])

    # The flange points down, so positive local Z moves the tool down in base Z.
    start_poses = await motion_group.forward_kinematics([list(START_JOINTS)], tcp)
    if len(start_poses) != 1:
        raise RuntimeError("KUKA forward kinematics returned no start pose")
    approach_pose = start_poses[0]
    pickup_pose = approach_pose @ Pose((0, 0, PICKUP_TRAVEL, 0, 0, 0))
    transfer_pose = approach_pose @ Pose((0, TRANSFER_DISTANCE, 0, 0, 0, 0))
    drop_pose = transfer_pose @ Pose((0, 0, DROP_TRAVEL, 0, 0, 0))
    actions = [
        linear(pickup_pose, settings=settings),
        linear(approach_pose, settings=settings),
        linear(transfer_pose, settings=settings),
        linear(drop_pose, settings=settings),
        linear(transfer_pose, settings=settings),
    ]

    print("Planning pick-and-drop motion without singularity handling...")
    try:
        await motion_group.plan(actions=actions, tcp=tcp, start_joint_position=START_JOINTS)
    except PlanTrajectoryFailed as exc:
        print(f"  Expected: planning failed at the wrist singularity: {exc.error.error_feedback}")
    else:
        raise RuntimeError("The motion unexpectedly planned without singularity handling")

    print("Planning the same pick-and-drop with PALLETIZING_WRIST...")
    trajectory = await motion_group.plan(
        actions=actions,
        tcp=tcp,
        start_joint_position=START_JOINTS,
        singularity_handling=api.models.SingularityHandling.PALLETIZING_WRIST,
    )
    print(f"  Success: planned {len(trajectory.joint_positions)} trajectory points.")

    await motion_group.execute(trajectory, tcp, actions=actions)
    await asyncio.sleep(1)
    print("  Executed the pick-and-drop through the wrist singularity.")


if __name__ == "__main__":
    run_program(singularity_handling)
