import asyncio

import nova
from nova import Nova
from nova.actions import lin
from nova.cell.controllers import kuka_controller
from nova.program import ProgramPreconditions
from nova.types import MotionSettings
from nova.types.pose import Pose


@nova.program(
    name="Basic Program",
    preconditions=ProgramPreconditions(
        controllers=[
            kuka_controller(
                name="kuka",
                controller_ip="10.101.200.100",
                controller_port=54600,
                rsi_server_ip="10.101.201.99",
                rsi_server_port=30152,
            )
        ],
        cleanup_controllers=False,
    ),
)
async def main():
    async with Nova() as nova:
        cell = nova.cell()
        controller = await cell.controller("kuka")

        # Connect to the controller and activate motion groups
        async with controller[0] as motion_group:
            # home_joints = await motion_group.joints()
            tcp_names = await motion_group.tcp_names()
            tcp = tcp_names[0]

            # Get current TCP pose and offset it slightly along the x-axis
            # home_pose = await motion_group.tcp_pose(tcp)
            home_pose = Pose(1004.2, -171.3, 1683.7, 2.1019, 2.2946, 0.0619)
            approach_1 = (999.6, -106.9, 1447.5, -2.2403, -2.1963, -0.0251)
            ap_1 = (999.6, -106.9, 1404.8, -2.2405, -2.1964, -0.0254)
            tp_1 = (999.6, -428.9, 1404.8, -2.2405, -2.1964, -0.0254)
            approach_2 = (989.4, -662.4, 1069.1, -1.2287, -1.1942, -1.1987)
            ap_2 = (989.5, -585.4, 1069.1, -1.2288, -1.1942, -1.1988)

            actions = [
                # lin(home_pose @ Pose((0, 0, 0, 0, 0, 0))),
                lin(home_pose),
                lin(Pose(999.6, -106.9, 1447.5, -2.2403, -2.1963, -0.0251)),
                lin(Pose(999.6, -106.9, 1404.8, -2.2405, -2.1964, -0.0254)),
                lin(Pose(999.6, -106.9, 1447.5, -2.2403, -2.1963, -0.0251)),
                lin(Pose(999.6, -428.9, 1404.8, -2.2405, -2.1964, -0.0254)),
                lin(Pose(989.4, -662.4, 1069.1, -1.2287, -1.1942, -1.1987)),
                lin(
                    Pose(
                        989.499946287328,
                        -585.4049471015012,
                        1034.1538949430264,
                        -1.2288147212968314,
                        -1.194169320801483,
                        -1.198766056907108,
                    )
                ),
                lin(Pose(989.4, -662.4, 1069.1, -1.2287, -1.1942, -1.1987)),
                lin(Pose(999.6, -428.9, 1404.8, -2.2405, -2.1964, -0.0254)),
                lin(home_pose),
            ]
        # you can update the settings of the action
        for action in actions:
            action.settings = MotionSettings(tcp_velocity_limit=100)

        joint_trajectory = await motion_group.plan(actions, tcp)
        await motion_group.execute(joint_trajectory, tcp, actions=actions)

        # await cell.delete_robot_controller(controller.controller_id)


if __name__ == "__main__":
    asyncio.run(main())
