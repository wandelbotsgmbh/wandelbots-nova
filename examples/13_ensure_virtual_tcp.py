"""
Example demonstrating ensure_virtual_tcp functionality.

This example shows how to ensure a virtual TCP exists with the expected configuration
on a motion group. The TCP will be created if it doesn't exist, or updated if the
configuration differs.

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

CONTROLLER_NAME = "test-robot"
TCP_ID = "test_gripper"


async def main():
    async with Nova() as nova:
        cell = nova.cell()

        print(f"Creating virtual robot controller '{CONTROLLER_NAME}'...")
        controller = await cell.ensure_virtual_robot_controller(
            CONTROLLER_NAME,
            models.VirtualControllerTypes.UNIVERSALROBOTS_MINUS_UR10E,
            models.Manufacturer.UNIVERSALROBOTS,
        )

        robot_tcp = RobotTcp(
            id=TCP_ID,
            position=Vector3d(x=0, y=0, z=150),
            rotation=RotationAngles(
                angles=[0, 0, 0], type=RotationAngleTypes.EULER_ANGLES_EXTRINSIC_XYZ
            ),
        )

        async with controller[0] as motion_group:
            print(f"Ensuring TCP '{TCP_ID}' exists with specified configuration...")

            result_tcp = await motion_group.ensure_virtual_tcp(tcp=robot_tcp)
            print(f"TCP '{result_tcp.id}' is now available")

            tcp_names = await motion_group.tcp_names()
            print(f"Available TCPs: {tcp_names}")

            existing_tcps = await motion_group.tcps()
            for tcp in existing_tcps:
                if tcp.id == TCP_ID:
                    print(f"TCP '{TCP_ID}' configuration:")
                    print(f"  Position: x={tcp.position.x}, y={tcp.position.y}, z={tcp.position.z}")
                    print(f"  Rotation: {tcp.rotation.angles} ({tcp.rotation.type})")
                    break

        print(f"Cleaning up: deleting controller '{CONTROLLER_NAME}'...")
        await cell.delete_robot_controller(CONTROLLER_NAME)
        print("Example completed successfully!")


if __name__ == "__main__":
    asyncio.run(main())
