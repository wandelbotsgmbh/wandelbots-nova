"""
Example: Plan and execute a direction-constrained Cartesian motion.

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
from nova.actions import cartesian_ptp, direction_constrained_cartesian_ptp, joint_ptp
from nova.cell import virtual_controller
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

    direction_constraint = api.models.DirectionConstraint(
        world=api.models.Vector3d([0.0, 0.0, 1.0]),  # world z-axis
        tcp=api.models.Vector3d([0.0, 1.0, 0.0]),  # tcp y-axis
        tolerance=0.05,
    )

    # move robot into a valid start pose
    projected_pose = project_cartesian_pose_direction_constraint(current_pose, direction_constraint)
    normal_cartesian_actions = [
        joint_ptp(home_joints, settings=normal),
        cartesian_ptp(projected_pose, settings=normal),
    ]
    trajectory_to_projected_pose = await motion_group.plan(normal_cartesian_actions, tcp)
    await motion_group.execute(trajectory_to_projected_pose, tcp, actions=normal_cartesian_actions)

    # move robot while satisfying constraints on the tcp orientation
    target_pose = projected_pose @ Pose((100.0, 0.0, -50.0, 0.0, 0.0, 0.0))
    constrained_target_pose = api.models.ConstrainedPose(
        position=target_pose.position.to_api_model(), orientation=0.0
    )
    direction_constrained_actions = [
        direction_constrained_cartesian_ptp(
            constrained_target_pose, constraint=direction_constraint, settings=fast
        )
    ]
    trajectory_to_target_pose = await motion_group.plan(direction_constrained_actions, tcp)
    await motion_group.execute(
        trajectory_to_target_pose, tcp, actions=direction_constrained_actions
    )


if __name__ == "__main__":
    run_program(plan_and_execute_direction_constrained)
