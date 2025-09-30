"""
Example: Perform relative movements with a robot.

Prerequisites:
- Create an NOVA instance
- Set env variables (you can specify them in an .env file):
    - NOVA_API=<api>
    - NOVA_ACCESS_TOKEN=<token>
"""

import nova
from nova import Nova, api, run_program, viewers
from nova.actions import TrajectoryBuilder, cartesian_ptp, io_write, joint_ptp
from nova.cell import virtual_controller
from nova.program import ProgramPreconditions
from nova.types import MotionSettings, Pose


@nova.program(
    name="Plan and Execute",
    viewer=viewers.Rerun(),
    preconditions=ProgramPreconditions(
        controllers=[
            virtual_controller(
                name="ur10",
                manufacturer=api.models.Manufacturer.UNIVERSALROBOTS,
                type=api.models.VirtualControllerTypes.UNIVERSALROBOTS_MINUS_UR10E,
            )
        ],
        cleanup_controllers=False,
    ),
)
async def plan_and_execute():
    async with Nova() as nova:
        cell = nova.cell()
        controller = await cell.controller("ur10")

        # Connect to the controller and activate motion groups
        async with controller[0] as motion_group:
            home_joints = await motion_group.joints()
            tcp_names = await motion_group.tcp_names()
            tcp = tcp_names[0]

            # Get current TCP pose and offset it slightly along the x-axis
            current_pose = await motion_group.tcp_pose(tcp)
            target_pose = current_pose @ Pose((1, 0, 0, 0, 0, 0))

            slow = MotionSettings(tcp_velocity_limit=50)
            normal = MotionSettings(tcp_velocity_limit=250)
            fast = MotionSettings(tcp_velocity_limit=500)

            # The trajectory builder is a context manager that can be used to build a trajectory with fine-grained control over the settings
            t = TrajectoryBuilder(settings=normal)

            # First move to the home position, since we passed "slow" settings to the motion it will override the one from the trajectory builder
            t.move(joint_ptp(home_joints, settings=slow))

            # It's possible to use the sequence method to add multiple actions to the trajectory. Since no settings are specified it takes
            #   the settings from the trajectory builder
            t.sequence(
                cartesian_ptp(target_pose),
                joint_ptp(home_joints),
                cartesian_ptp(target_pose @ Pose((0, 100, 100, 0, 0, 0))),
                cartesian_ptp(
                    target_pose @ Pose.from_euler((0, 100, 200), (0, 45, 0), "xyz", degrees=True)
                ),  # used from_euler util function to specify orientation in euler angles
                io_write(key="tool_out[0]", value=False),
            )

            # You can use the set(...) context manager to set settings for a block of actions
            with t.set(settings=fast):
                t.move(
                    cartesian_ptp(
                        target_pose @ Pose.from_euler((0, 0, 200), (0, 45, 0), "xyz", degrees=True),
                        settings=slow,
                    )
                )  # moving with slow setting
                t.move(joint_ptp(home_joints))
                t.move(cartesian_ptp(target_pose @ Pose((0, 50, 0, 0, 0, 0))))

            t.move(joint_ptp(home_joints, settings=slow))

        joint_trajectory = await motion_group.plan(t.actions, tcp)
        motion_iter = motion_group.stream_execute(joint_trajectory, tcp, actions=t.actions)
        async for motion_state in motion_iter:
            print(motion_state)

        # Read and write IO values
        io_value = await controller.read("tool_out[0]")
        print(io_value)
        await controller.write("tool_out[0]", True)
        written_io_value = await controller.read("tool_out[0]")
        print(written_io_value)


if __name__ == "__main__":
    run_program(plan_and_execute)
