"""
Example: Getting the current state of a robot.

Prerequisites:
- Create an NOVA instance
- Set env variables (you can specify them in an .env file):
    - NOVA_API=<api>
    - NOVA_ACCESS_TOKEN=<token>
"""

import asyncio

from wandelbots_api_client.models import RobotTcp, RotationAngles, RotationAngleTypes, Vector3d

from nova import Nova
from nova.api import models
from nova.cell import virtual_controller


async def main():
    async with Nova() as nova:
        cell = nova.cell()
        controller = await cell.ensure_controller(
            robot_controller=virtual_controller(
                name="ur",
                manufacturer=models.Manufacturer.UNIVERSALROBOTS,
                type=models.VirtualControllerTypes.UNIVERSALROBOTS_MINUS_UR10E,
            )
        )

        async with controller[0] as motion_group:
            tcp_names = await motion_group.tcp_names()
            print(tcp_names)

            tcp = tcp_names[0]

            test_tcp = RobotTcp(
                id="test_gripper",
                position=Vector3d(x=0, y=0, z=150),
                rotation=RotationAngles(
                    angles=[0, 0, 0], type=RotationAngleTypes.EULER_ANGLES_EXTRINSIC_XYZ
                ),
            )

            await motion_group.ensure_virtual_tcp(tcp=test_tcp)

            updated_tcp_names = await motion_group.tcp_names()
            assert "test_gripper" in updated_tcp_names

            # Current motion group state
            state = await motion_group.get_state(tcp)
            print(state)

            # Current joints positions
            joints = await motion_group.joints()
            print(joints)

            # Current TCP pose
            tcp_pose = await motion_group.tcp_pose(tcp)
            print(tcp_pose)

        await cell.delete_robot_controller(controller.controller_id)


if __name__ == "__main__":
    asyncio.run(main())
