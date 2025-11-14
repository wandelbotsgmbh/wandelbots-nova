"""
Example: Getting the current state of a robot.

Prerequisites:
- Create an NOVA instance
- Set env variables (you can specify them in an .env file):
    - NOVA_API=<api>
    - NOVA_ACCESS_TOKEN=<token>
"""

import asyncio

import nova
from nova import Nova
from nova.actions.motions import joint_ptp
from nova.actions.trajectory_builder import TrajectoryBuilder
from nova.cell.controllers import abb_controller
from nova.program import ProgramPreconditions
from nova.types.motion_settings import MotionSettings


@nova.program(
    name="Basic Program",
    preconditions=ProgramPreconditions(
        controllers=[
            abb_controller(
                name="abb",
                controller_ip="192.168.124.10",
                egm_server_ip="192.168.124.1",
                egm_server_port=30100,
            )
        ],
        cleanup_controllers=False,
    ),
)
async def main():
    async with Nova() as nova:
        cell = nova.cell()
        controller = await cell.controller("abb")

        async with controller[0] as motion_group:
            tcp = "user/ZiGri45"

            tcp_names = await motion_group.tcp_names()
            print(tcp_names)
            state = await motion_group.get_state(tcp)
            print(state)
            joints = await motion_group.joints()
            print(joints)
            tcp_pose = await motion_group.tcp_pose(tcp)
            print(tcp_pose)

            slow = MotionSettings(tcp_velocity_limit=50)
            t = TrajectoryBuilder(settings=slow)
            t.move(joint_ptp(joints, settings=slow))

        joint_trajectory = await motion_group.plan(t.actions, tcp)
        motion_group.execute(joint_trajectory, tcp, actions=t.actions)


if __name__ == "__main__":
    asyncio.run(main())
