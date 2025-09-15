import asyncio

import nova
from nova import Nova
from nova.actions import lin
from nova.cell.controllers import kuka_controller
from nova.program import ProgramPreconditions
from nova.types import MotionSettings
from nova.types.pose import Pose

"""
Example: Perform relative movements with a robot.

Prerequisites:
- Create an NOVA instance
- Set env variables (you can specify them in an .env file):
    - NOVA_API=<api>
    - NOVA_ACCESS_TOKEN=<token>
"""

from datetime import datetime

from icecream import ic

ic.configureOutput(includeContext=True, prefix=lambda: f"{datetime.now()} | ")


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
            # home_joints = [0, -pi / 2, -pi / 2, -pi / 2, pi / 2, -pi / 2]
            tcp_names = await motion_group.tcp_names()
            ic(tcp_names)
            tcp = tcp_names[0]
            # await motion_group.plan_and_execute(
            #     # [jnt(home_joints), jnt([home_joints[0] + pi / 4] + home_joints[1:])],
            #     [jnt(home_joints)],
            #     tcp,
            # )

            # Get current TCP pose and offset it slightly along the x-axis
            # home_pose = await motion_group.tcp_pose(tcp)
            home_pose = Pose(1004.2, -171.3, 1683.7, 2.1019, 2.2946, 0.0619)
            approach_1 = (999.6, -106.9, 1447.5, -2.2403, -2.1963, -0.0251)
            ap_1 = (999.6, -106.9, 1404.8, -2.2405, -2.1964, -0.0254)
            tp_1 = (999.6, -428.9, 1404.8, -2.2405, -2.1964, -0.0254)
            approach_2 = (989.4, -662.4, 1069.1, -1.2287, -1.1942, -1.1987)
            ap_2 = (989.5, -585.4, 1069.1, -1.2288, -1.1942, -1.1988)

            # dist = 100
            ic(home_pose)
            actions = [
                # lin(home_pose @ Pose((0, 0, 0, 0, 0, 0))),
                lin(home_pose),
                lin(Pose(999.6, -106.9, 1447.5, -2.2403, -2.1963, -0.0251)),
                lin(Pose(999.6, -106.9, 1389, -2.2403, -2.1963, -0.0251)),
                lin(Pose(999.6, -428.9, 1404.8, -2.2405, -2.1964, -0.0254)),
                lin(Pose(989.4, -662.4, 1069.1, -1.2287, -1.1942, -1.1987)),
                lin(
                    Pose(
                        989.5029123379152,
                        -585.4850067558077,
                        1034.4932789441912,
                        -1.2288896024576759,
                        -1.194290507618143,
                        -1.198872217108755,
                    )
                ),
                lin(Pose(989.4, -662.4, 1069.1, -1.2287, -1.1942, -1.1987)),
                lin(Pose(999.6, -428.9, 1404.8, -2.2405, -2.1964, -0.0254)),
                lin(home_pose),
            ]
            ic(actions)
        ic(actions[0].metas)
        # you can update the settings of the action
        for action in actions:
            action.settings = MotionSettings(tcp_velocity_limit=100)

        joint_trajectory = await motion_group.plan(actions, tcp)
        # TODO this probaly not consumes the state stream immediately and thus might cause issues
        # only start consuming the state stream when the trajectory is actually executed
        await motion_group.execute(joint_trajectory, tcp, actions=actions)

        ic("Program finished")
        # await cell.delete_robot_controller(controller.controller_id)


if __name__ == "__main__":
    asyncio.run(main())
