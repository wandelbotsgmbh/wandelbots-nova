import asyncio
from math import pi

from nova import MotionGroup, Nova
from nova.actions import Action, cartesian_ptp
from nova.api import models
from nova.types import Pose
from nova_rerun_bridge import NovaRerunBridge
from nova_rerun_bridge.trajectory import TimingMode

"""
Example: Move multiple robots to perform coordinated movements.

Prerequisites:
- Create an NOVA instance
- Set env variables (you can specify them in an .env file):
    - NOVA_API=<api>
    - NOVA_ACCESS_TOKEN=<token>
"""


async def move_robot(
    motion_group: MotionGroup, tcp: str, bridge: NovaRerunBridge, timing_mode: TimingMode
):
    await bridge.log_saftey_zones(motion_group)

    home_pose = Pose((200, 200, 600, 0, pi, 0))
    target_pose = home_pose @ (100, 0, 0, 0, 0, 0)
    actions: list[Action] = [
        cartesian_ptp(home_pose),
        cartesian_ptp(target_pose),
        cartesian_ptp(target_pose @ (0, 0, 100, 0, 0, 0)),
        cartesian_ptp(target_pose @ (0, 100, 0, 0, 0, 0)),
        cartesian_ptp(home_pose),
    ]

    trajectory = await motion_group.plan(actions, tcp)
    await bridge.log_trajectory(trajectory, tcp, motion_group, timing_mode=timing_mode)


async def main():
    async with Nova() as nova, NovaRerunBridge(nova) as bridge:
        await bridge.setup_blueprint()
        cell = nova.cell()
        ur10 = await cell.ensure_virtual_robot_controller(
            "ur10",
            models.VirtualControllerTypes.UNIVERSALROBOTS_MINUS_UR10E,
            models.Manufacturer.UNIVERSALROBOTS,
        )
        ur5 = await cell.ensure_virtual_robot_controller(
            "ur5",
            models.VirtualControllerTypes.UNIVERSALROBOTS_MINUS_UR5E,
            models.Manufacturer.UNIVERSALROBOTS,
        )
        tcp = "Flange"

        # NC-1047
        await asyncio.sleep(5)

        flange_state = await ur10[0].get_state(tcp)
        print(flange_state)

        # activate all motion groups
        async with ur10:
            await move_robot(ur10.motion_group("0@ur10"), tcp, bridge, TimingMode.CONTINUE)

        # activate motion group 0
        async with ur10.motion_group("0@ur10") as mg_0:
            await move_robot(mg_0, tcp, bridge, TimingMode.CONTINUE)

        # activate motion group 0
        async with ur10[0] as mg_0:
            await move_robot(mg_0, tcp, bridge, TimingMode.CONTINUE)

        # activate motion group 0 from two different controllers
        async with ur10[0] as ur_0_mg, ur5[0] as kuka_0_mg:
            await asyncio.gather(
                move_robot(ur_0_mg, tcp, bridge, TimingMode.SYNC),
                move_robot(kuka_0_mg, tcp, bridge, TimingMode.SYNC),
            )

        bridge.continue_after_sync()

        # activate motion group 0 from two different controllers
        mg_0 = ur10.motion_group("0@ur10")
        mg_1 = ur5.motion_group("0@ur5")
        async with mg_0, mg_1:
            await asyncio.gather(
                move_robot(mg_0, tcp, bridge, TimingMode.SYNC),
                move_robot(mg_1, tcp, bridge, TimingMode.SYNC),
            )


if __name__ == "__main__":
    asyncio.run(main())
