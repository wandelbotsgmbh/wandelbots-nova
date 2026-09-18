"""
Example: Blend between motion commands so the velocity does not drop to zero.

`MotionSettings.blending` takes the API blending messages directly, so every option the
NOVA API offers is available: `api.models.BlendingAuto` for auto-blending and
`api.models.BlendingPosition` for zone-based blending in position, orientation or joint space.

Prerequisites:
- Create an NOVA instance
- Set env variables (you can specify them in an .env file):
    - NOVA_API=<api>
    - NOVA_ACCESS_TOKEN=<token>
"""

import nova
from nova import api, run_program, viewers
from nova.actions import cartesian_ptp, joint_ptp
from nova.cell import virtual_controller
from nova.types import MotionSettings, Pose


@nova.program(
    name="Blending",
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
async def blending(ctx: nova.ProgramContext):
    cell = ctx.nova.cell()
    controller = await cell.controller("ur10e")

    motion_group = controller[0]
    home_joints = await motion_group.joints()
    tcp = (await motion_group.tcp_names())[0]

    current_pose = await motion_group.tcp_pose(tcp)

    # Auto-blending keeps at least 30% of the original velocity at the target point.
    auto = MotionSettings(blending=api.models.BlendingAuto(min_velocity_in_percent=30))

    # Position blending restricts the blending zone to a radius around the target point.
    # Use the percentage variant to express the zone relative to the trajectory length instead.
    position = MotionSettings(
        blending=api.models.BlendingPosition(position_zone_radius=25.0, orientation_zone_radius=0.2)
    )

    # Blending can also be expressed in joint space.
    joint_space = MotionSettings(
        blending=api.models.BlendingPosition(
            joints_zone_radius=0.1,
            joints_zone_percentage=20.0,
            space=api.models.BlendingSpace.JOINT,
        )
    )

    actions = [
        joint_ptp(home_joints),
        cartesian_ptp(current_pose @ Pose((100, 0, 0, 0, 0, 0)), settings=auto),
        cartesian_ptp(current_pose @ Pose((100, 100, 0, 0, 0, 0)), settings=position),
        cartesian_ptp(current_pose @ Pose((0, 100, 0, 0, 0, 0)), settings=joint_space),
        joint_ptp(home_joints),
    ]

    trajectory = await motion_group.plan(actions, tcp)
    await motion_group.execute(trajectory, tcp, actions=actions)


if __name__ == "__main__":
    run_program(blending)
