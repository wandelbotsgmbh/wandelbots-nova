"""
Example: Kinematic configuration controls which IK branch the planner uses.

A 6-axis robot can reach the same TCP pose with multiple joint configurations
(up to 8 for a standard 6-axis arm). The kinematic configuration specifies which
"branch" of the inverse kinematics to use: shoulder (FRONT/BACK), elbow (UP/DOWN),
and wrist (FLIP/NO_FLIP).

Key insight: When a CartesianPTP is planned WITHOUT a kinematic configuration, the
planner stays in the robot's current branch. If the target pose is only reachable
in a DIFFERENT branch, the plan fails. An explicit kinematic configuration on the
CartesianPTP forces the planner to switch into the specified branch.

This example demonstrates:
  1. Switch elbow config at the current pose (UP -> DOWN) — same TCP position,
     only joints change visibly (uses get_kinematic_configuration to query current branch)
  2. Plan to a distant target WITHOUT config -> PlanTrajectoryFailed
     (target not reachable in the current branch)
  3. Plan the same motion WITH explicit config (wrist FLIP) -> succeeds and executes;
     verified afterwards with get_kinematic_configuration

Prerequisites:
- Set env variables (you can specify them in an .env file):
    - NOVA_API=<api>
    - NOVA_ACCESS_TOKEN=<token>
"""

import asyncio
from math import degrees, pi

import nova
from nova import api, run_program
from nova.actions import cartesian_ptp, joint_ptp
from nova.cell import virtual_controller
from nova.exceptions import PlanTrajectoryFailed
from nova.types import MotionSettings, Pose

Shoulder = api.models.KinematicBranchShoulder
Elbow = api.models.KinematicBranchElbow
Wrist = api.models.KinematicBranchWrist


@nova.program(
    name="Kinematic Configuration",
    preconditions=nova.ProgramPreconditions(
        controllers=[
            virtual_controller(
                name="ur10e",
                manufacturer=api.models.Manufacturer.UNIVERSALROBOTS,
                type="universalrobots-ur10e",
            )
        ],
        cleanup_controllers=False,
    ),
)
async def kinematic_configuration(ctx: nova.ProgramContext):
    cell = ctx.nova.cell()
    controller = await cell.controller("ur10e")
    motion_group = controller[0]
    tcp_names = await motion_group.tcp_names()
    tcp = tcp_names[0]

    # Init position: FRONT/UP/NO_FLIP branch (simple joint values)
    init_joints = (pi, -pi / 2, pi / 2, -pi / 2, pi / 2, -pi / 2)
    fast = MotionSettings(tcp_velocity_limit=250)
    setup = await motion_group.get_setup(tcp)

    # Target pose: not reachable in FRONT/UP/NO_FLIP, requires a wrist FLIP
    target = Pose((1300, 0, 100, -pi, 0, 0))

    async def move_to_init():
        action = joint_ptp(init_joints, settings=fast)
        traj = await motion_group.plan([action], tcp, motion_group_setup=setup)
        await motion_group.execute(traj, tcp, actions=[action])

    # --- Demo ---

    await move_to_init()

    # Document starting configuration
    init_deg = [round(degrees(j), 1) for j in init_joints]
    print("Init: FRONT/UP/NO_FLIP")
    print(f"  Joints: {init_deg}")

    # Step 1: Query current config, then switch only elbow (UP -> DOWN).
    # The robot stays at the same TCP position, but joints change visibly.
    print("\nStep 1: Switch elbow config at current pose (UP -> DOWN)")
    state = await motion_group.get_state(tcp)
    current_pose = state.pose
    # Query the current kinematic configuration from joints
    [current_config] = await motion_group.get_kinematic_configuration([state.joints])
    print(
        f"  Current config: shoulder={current_config.kinematic_branch.shoulder_branch.value}, "
        f"elbow={current_config.kinematic_branch.elbow_branch.value}, "
        f"wrist={current_config.kinematic_branch.wrist_branch.value}"
    )
    # Flip only the elbow, keep shoulder and wrist from the queried config
    switched_config = api.models.KinematicConfiguration(
        kinematic_branch=api.models.KinematicBranch(
            shoulder_branch=current_config.kinematic_branch.shoulder_branch,
            elbow_branch=Elbow.DOWN,
            wrist_branch=current_config.kinematic_branch.wrist_branch,
        )
    )
    configured_pose = Pose(current_pose.to_tuple(), kinematic_configuration=switched_config)
    action = cartesian_ptp(configured_pose, settings=fast)
    traj = await motion_group.plan([action], tcp, motion_group_setup=setup)
    joints_deg = [round(degrees(j), 1) for j in traj.joint_positions[-1]]
    print(f"  Before (UP):   {init_deg}")
    print(f"  After (DOWN):  {joints_deg}")
    print("  -> J1, J5, J6 unchanged — only elbow flipped")
    await motion_group.execute(traj, tcp, actions=[action])
    await asyncio.sleep(3)
    await move_to_init()
    await asyncio.sleep(3)

    # Step 2: Plan to target WITHOUT config — planner stays in current branch, fails.
    print("\nStep 2: Plan to target WITHOUT kinematic configuration")
    try:
        action = cartesian_ptp(target, settings=fast)
        await motion_group.plan([action], tcp, motion_group_setup=setup)
    except PlanTrajectoryFailed:
        print("  PlanTrajectoryFailed — target not reachable in current branch")

    # Step 3: Plan to target WITH explicit config — forces branch switch, succeeds.
    print("\nStep 3: Plan to target WITH kinematic configuration (FRONT/UP/FLIP)")
    configured_target = Pose(
        target.to_tuple(),
        kinematic_configuration=api.models.KinematicConfiguration(
            kinematic_branch=api.models.KinematicBranch(
                shoulder_branch=Shoulder.FRONT, elbow_branch=Elbow.UP, wrist_branch=Wrist.FLIP
            )
        ),
    )
    action = cartesian_ptp(configured_target, settings=fast)
    traj = await motion_group.plan([action], tcp, motion_group_setup=setup)
    joints_deg = [round(degrees(j), 1) for j in traj.joint_positions[-1]]
    print(f"  Success! Joints: {joints_deg}")
    await motion_group.execute(traj, tcp, actions=[action])
    await asyncio.sleep(3)
    # Verify the robot actually landed in FLIP config
    final_joints = await motion_group.joints()
    [final_config] = await motion_group.get_kinematic_configuration([final_joints])
    print(
        f"  Verified: shoulder={final_config.kinematic_branch.shoulder_branch.value}, "
        f"elbow={final_config.kinematic_branch.elbow_branch.value}, "
        f"wrist={final_config.kinematic_branch.wrist_branch.value}"
    )
    await move_to_init()

    print("\nWithout explicit config, the planner cannot switch branches.")
    print("With kinematic configuration, you control which IK solution is used.")


if __name__ == "__main__":
    run_program(kinematic_configuration)
