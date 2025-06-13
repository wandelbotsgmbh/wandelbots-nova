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
            "test_robot",
            models.VirtualControllerTypes.UNIVERSALROBOTS_MINUS_UR10E,
            models.Manufacturer.UNIVERSALROBOTS,
        )

        test_tcp = RobotTcp(
            id="test_gripper",
            position=Vector3d(x=0, y=0, z=150),
            rotation=RotationAngles(
                angles=[0, 0, 0], type=RotationAngleTypes.EULER_ANGLES_EXTRINSIC_XYZ
            ),
        )

        print("Testing ensure_virtual_tcp functionality...")

        print("Test 1: Creating new TCP...")
        result_tcp = await cell.ensure_virtual_tcp(
            tcp=test_tcp, controller_name="test_robot", motion_group_idx=0
        )
        print(f"Created TCP: {result_tcp.id}")

        print("Test 2: Ensuring existing TCP with same config...")
        result_tcp2 = await cell.ensure_virtual_tcp(
            tcp=test_tcp, controller_name="test_robot", motion_group_idx=0
        )
        print(f"Ensured TCP: {result_tcp2.id}")

        print("Test 3: Verifying TCP is available...")
        async with controller[0] as motion_group:
            tcp_names = await motion_group.tcp_names()
            print(f"Available TCPs: {tcp_names}")

            if "test_gripper" in tcp_names:
                print("✓ TCP successfully created and available")

                tcp_pose = await motion_group.tcp_pose("test_gripper")
                print(f"TCP pose: {tcp_pose}")
            else:
                print("✗ TCP not found in motion group")

        await cell.delete_robot_controller("test_robot")
        print("Cleanup completed")


if __name__ == "__main__":
    asyncio.run(main())
