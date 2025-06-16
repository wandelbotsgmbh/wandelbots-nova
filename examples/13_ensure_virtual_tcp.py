"""
Integration test for ensure_virtual_tcp functionality.

Prerequisites:
- Create a NOVA instance
- Set env variables (you can specify them in an .env file):
    - NOVA_API=<api>
    - NOVA_ACCESS_TOKEN=<token>
"""

import asyncio

from wandelbots_api_client.models import RobotTcp, RotationAngles, RotationAngleTypes, Vector3d

from nova import Nova
from nova.api import models


async def main():
    async with Nova() as nova:
        cell = nova.cell()

        controller = await cell.ensure_virtual_robot_controller(
            "test-robot",
            models.VirtualControllerTypes.UNIVERSALROBOTS_MINUS_UR10E,
            models.Manufacturer.UNIVERSALROBOTS,
        )

        robot_tcp = RobotTcp(
            id="test_gripper",
            position=Vector3d(x=0, y=0, z=150),
            rotation=RotationAngles(
                angles=[0, 0, 0], type=RotationAngleTypes.EULER_ANGLES_EXTRINSIC_XYZ
            ),
        )

        result_tcp = await cell.ensure_virtual_tcp(
            tcp=robot_tcp, controller_name="test-robot", motion_group_idx=0
        )

        async with controller[0] as motion_group:
            tcp_names = await motion_group.tcp_names()
            
            assert "test_gripper" in tcp_names, "TCP 'test_gripper' should exist"
            
            existing_tcps = await motion_group.tcps()
            found_tcp = None
            for tcp in existing_tcps:
                if tcp.id == "test_gripper":
                    found_tcp = tcp
                    break
            
            assert found_tcp is not None, "TCP 'test_gripper' should be found in tcps list"
            assert found_tcp.position.x == 0, f"Expected x=0, got {found_tcp.position.x}"
            assert found_tcp.position.y == 0, f"Expected y=0, got {found_tcp.position.y}"
            assert found_tcp.position.z == 150, f"Expected z=150, got {found_tcp.position.z}"

        await cell.delete_robot_controller("test-robot")


if __name__ == "__main__":
    asyncio.run(main())
