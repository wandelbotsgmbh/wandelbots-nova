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

import nova
from nova import Nova, api
from nova.cell import virtual_controller
from nova.program import ProgramPreconditions


@nova.program(
    name="01 Basic Program",
    preconditions=ProgramPreconditions(
        controllers=[
            virtual_controller(
                name="ur10e",
                manufacturer=api.models.Manufacturer.UNIVERSALROBOTS,
                type=api.models.VirtualControllerTypes.UNIVERSALROBOTS_MINUS_UR10E,
            )
        ],
        cleanup_controllers=True,
    ),
)
async def main():
    async with Nova() as nova:
        cell = nova.cell()
        controller = await cell.controller("ur10e")

        async with controller[0] as motion_group:
            await motion_group.ensure_virtual_tcp(
                tcp=RobotTcp(
                    id="test_gripper",
                    position=Vector3d(x=0, y=0, z=150),
                    rotation=RotationAngles(
                        angles=[0, 0, 0], type=RotationAngleTypes.EULER_ANGLES_EXTRINSIC_XYZ
                    ),
                )
            )

            tcp_names = await motion_group.tcp_names()
            print(tcp_names)

            tcp = "test_gripper"

            # Current motion group state
            state = await motion_group.get_state(tcp)
            print(state)

            # Current joints positions
            joints = await motion_group.joints()
            print(joints)

            # Current TCP pose
            tcp_pose = await motion_group.tcp_pose(tcp)
            print(tcp_pose)


if __name__ == "__main__":
    asyncio.run(main())
