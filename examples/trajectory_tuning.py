import asyncio
from math import pi

import nova
from nova import Nova, api
from nova.actions import jnt, lin
from nova.cell.controllers import virtual_controller
from nova.program import ProgramPreconditions
from nova.types import MotionSettings, Pose

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
            virtual_controller(
                name="kuka",
                manufacturer=api.models.Manufacturer.KUKA,
                type=api.models.VirtualControllerTypes.KUKA_MINUS_KR16_R1610_2,
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
            # home_joints = await motion_group.joints()
            home_joints = [-1.8047, -1.8209, 1.8212, 0.2806, 1.0064, -0.4003]
            tcp_names = await motion_group.tcp_names()
            ic(tcp_names)
            tcp = tcp_names[0]
            await motion_group.plan_and_execute([jnt(home_joints)], tcp)

            # Get current TCP pose and offset it slightly along the x-axis
            current_pose = await motion_group.tcp_pose(tcp)
            dist = 300
            ic(current_pose)

            pose1 = Pose((-294.72441795539453, 857.5360255082029, 1571.6088053149488, -1.75879100697647, 1.9140012914288607, 0.5060621281068792))
            pose2 = Pose((-746.1253358991424, 1200.613557032049, 1270.642099155898, -1.7588031006603488, 1.9139892257910591, 0.506079131889))
            pose3 = Pose((-63.0910254674847, 1361.600279974159, 1039.0400574918995, -1.7587694971285424, 1.9139937306883776, 0.5060999965134398))
            pose4 = Pose((-63.093070301797916, 1361.5991921028099, 1713.2416879448233, -1.758808332372446, 1.9140146130789166, 0.5061352892931338))

            actions = [
                lin(current_pose @ Pose((0, 0, 0, 0, 0, 0))),
                lin(current_pose @ Pose((0, dist, 0, 0, 0, 0))),
                lin(current_pose @ Pose((0, dist, dist, 0, 0, 0))),
                lin(current_pose @ Pose((0, 0, dist, 0, 0, 0))),
                lin(current_pose @ Pose((0, 0, 0, 0, 0, 0))),
                # jnt(home_joints),
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
