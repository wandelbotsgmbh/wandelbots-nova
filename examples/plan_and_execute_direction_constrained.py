"""
Example: Plan and execute a direction-constrained motion.

To plan with direction constraints the robot first has to move to a valid position that satisfies the constraint.
Such a valid position can be determined by projecting the current position to the constraint.
Either in cartesian space or joint space.

In this example first a cartesian projection is used (z-axis up). And then a joint projection is used (z-axis down).
Every time the robot first moves into a valid start position and then executes a constrained motion
(either direction_constrained_cartesian_ptp or direction_constrained_joint_ptp).

Prerequisites:
- Create an NOVA instance
- Set env variables (you can specify them in an .env file):
    - NOVA_API=<api>
    - NOVA_ACCESS_TOKEN=<token>
"""

import numpy as np
from scipy.spatial.transform import Rotation as R

import nova
from nova import api, run_program, viewers
from nova.actions import (
    direction_constrained_cartesian_ptp,
    direction_constrained_joint_ptp,
    joint_ptp,
    linear,
)
from nova.cell import virtual_controller
from nova.utils import shift_joint_position_close_to_reference
from nova.types import MotionSettings, Pose, Vector3d


def project_cartesian_pose_direction_constraint(
    world_tcp_pose: Pose, constraint: api.models.DirectionConstraint
) -> Pose:
    """Rotate a pose so the constrained TCP direction aligns with the target world direction."""
    constraint_tcp = np.array(constraint.tcp)
    target_constraint_world = np.array(constraint.world)

    world_tcp_rotation = R.from_rotvec(np.array(world_tcp_pose.orientation))
    current_constraint_world = world_tcp_rotation.apply(constraint_tcp)

    rotation_correction, _ = R.align_vectors([target_constraint_world], [current_constraint_world])
    projected_rotation = rotation_correction * world_tcp_rotation

    return Pose(
        position=world_tcp_pose.position,
        orientation=Vector3d.from_tuple(tuple(projected_rotation.as_rotvec())),
        kinematic_configuration=world_tcp_pose.kinematic_configuration,
    )


@nova.program(
    name="Plan and Execute (Direction Constrained)",
    viewer=viewers.Rerun(),
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
async def plan_and_execute_direction_constrained(ctx: nova.ProgramContext):
    cell = ctx.nova.cell()
    controller = await cell.controller("ur10e")
    motion_group = controller[0]

    normal = MotionSettings(tcp_velocity_limit=120)
    fast = MotionSettings(tcp_velocity_limit=250)

    home_joints = await motion_group.joints()
    tcp = (await motion_group.tcp_names())[0]
    current_pose = await motion_group.tcp_pose(tcp)

    # first phase:
    # project current pose to direction constraint (tcp +z-axis) in cartesian space
    direction_constraint_pos_z = api.models.DirectionConstraint(
        world=api.models.Vector3d([0.0, 0.0, 1.0]),  # world z-axis
        tcp=api.models.Vector3d([0.0, 0.0, 1.0]),  # tcp z-axis
        tolerance=0.05,
    )

    projected_pose = project_cartesian_pose_direction_constraint(
        current_pose, direction_constraint_pos_z
    )

    # move robot to cartesian target pose while satisfying constraints on the tcp orientation
    target_pose = projected_pose @ Pose((100.0, 0.0, 0.0, 0.0, 0.0, 0.0))
    constrained_target_pose = api.models.ConstrainedPose(
        position=target_pose.position.to_api_model(), orientation=0.5
    )
    first_phase_actions = [
        joint_ptp(home_joints, settings=normal),
        linear(projected_pose, settings=normal),  # keep tcp position fixed
        direction_constrained_cartesian_ptp(
            constrained_target_pose, constraint=direction_constraint_pos_z, settings=fast
        ),
    ]
    first_phase_trajectory = await motion_group.plan(first_phase_actions, tcp)
    await motion_group.execute(first_phase_trajectory, tcp, actions=first_phase_actions)

    # second phase:
    # project current target and home joint positions to a new direction constraint (tcp -z-axis) in joint space,
    # move to the projected current position (keep first 3 joints fixed), then return via direction-constrained joint PTP.
    current_joints = await motion_group.joints()
    direction_constraint_neg_z = api.models.DirectionConstraint(
        world=api.models.Vector3d([0.0, 0.0, 1.0]),  # world z-axis
        tcp=api.models.Vector3d([0.0, 0.0, -1.0]),  # tcp -z-axis
        tolerance=0.05,
    )
    projected_joint_positions = await motion_group.project_joint_position_direction_constraint(
        joints=[current_joints, home_joints], constraint=direction_constraint_neg_z, tcp=tcp
    )

    projected_target_joints, projected_home_joints = projected_joint_positions
    if projected_target_joints is None or projected_home_joints is None:
        raise ValueError(
            "Failed to project current/home joints for direction constraint with tcp axis -z"
        )

    # ensure start and target are nearby
    setup = await motion_group.get_setup(tcp)
    joint_limits = setup.global_limits.joints if setup.global_limits is not None else None
    projected_home_joints = shift_joint_position_close_to_reference(
        np.array(projected_home_joints), np.array(projected_target_joints), joint_limits
    ).tolist()
    projected_target_joints = shift_joint_position_close_to_reference(
        np.array(projected_target_joints), np.array(projected_home_joints), joint_limits
    ).tolist()

    second_phase_actions = [
        joint_ptp(projected_target_joints, settings=normal),
        direction_constrained_joint_ptp(
            projected_home_joints, constraint=direction_constraint_neg_z, settings=fast
        ),
    ]
    second_phase_trajectory = await motion_group.plan(second_phase_actions, tcp)
    await motion_group.execute(second_phase_trajectory, tcp, actions=second_phase_actions)


if __name__ == "__main__":
    run_program(plan_and_execute_direction_constrained)
