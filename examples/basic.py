"""
Example: Getting the current state of a robot.

Prerequisites:
- Create an NOVA instance
- Set env variables (you can specify them in an .env file):
    - NOVA_API=<api>
    - NOVA_ACCESS_TOKEN=<token>
"""

import nova
from nova import Nova, api, run_program
from nova.cell import virtual_controller
from nova.program import ProgramPreconditions


@nova.program(
    name="Basic Program",
    preconditions=ProgramPreconditions(
        controllers=[
            virtual_controller(
                name="ur10e",
                manufacturer=api.models.Manufacturer.UNIVERSALROBOTS,
                type=api.models.VirtualControllerTypes.UNIVERSALROBOTS_MINUS_UR10E,
            )
        ],
        cleanup_controllers=False,
    ),
)
async def basic():
    async with Nova() as nova:
        cell = nova.cell()
        controller = await cell.controller("ur10e")

        async with controller[0] as motion_group:
            tcp = "test_gripper"

            await motion_group.ensure_virtual_tcp(
                tcp=api.models.RobotTcp(
                    id=tcp,
                    name=f"{tcp} Name",
                    position=[0, 0, 150],
                    orientation=[0, 0, 0],
                    orientation_type=api.models.orientation_type.OrientationType.ROTATION_VECTOR,
                ),
                timeout=10,
            )

            tcp_names = await motion_group.tcp_names()
            print(tcp_names)

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
    run_program(basic)
