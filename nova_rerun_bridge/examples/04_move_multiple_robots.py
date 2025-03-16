import asyncio

from wandelbots_api_client.models import (
    CoordinateSystem,
    RotationAngles,
    RotationAngleTypes,
    Vector3d,
)

from nova import Controller, Nova
from nova.actions import cartesian_ptp, joint_ptp
from nova.api import models
from nova_rerun_bridge import NovaRerunBridge
from nova_rerun_bridge.trajectory import TimingMode

"""
Example: Move multiple robots simultaneously.

Prerequisites:
- A cell with two robots: one named "ur" and another named "kuka".
"""


async def move_robot(controller: Controller, bridge: NovaRerunBridge):
    async with controller[0] as motion_group:
        await bridge.log_saftey_zones(motion_group)

        home_joints = await motion_group.joints()
        tcp_names = await motion_group.tcp_names()
        tcp = tcp_names[0]

        current_pose = await motion_group.tcp_pose(tcp)
        target_pose = current_pose @ (100, 0, 0, 0, 0, 0)
        actions = [joint_ptp(home_joints), cartesian_ptp(target_pose), joint_ptp(home_joints)]

        trajectory = await motion_group.plan(actions, tcp)
        await bridge.log_trajectory(trajectory, tcp, motion_group, timing_mode=TimingMode.SYNC)


async def main():
    async with Nova() as nova, NovaRerunBridge(nova) as bridge:
        cell = nova.cell()

        ur10 = await cell.ensure_virtual_robot_controller(
            "ur10",
            models.VirtualControllerTypes.UNIVERSALROBOTS_MINUS_UR10E,
            models.Manufacturer.UNIVERSALROBOTS,
        )
        kuka = await cell.ensure_virtual_robot_controller(
            "kuka", models.VirtualControllerTypes.KUKA_MINUS_KR16_R1610_2, models.Manufacturer.KUKA
        )

        # NC-1047
        await asyncio.sleep(3)

        await nova._api_client.virtual_robot_setup_api.set_virtual_robot_mounting(
            cell="cell",
            controller=kuka.controller_id,
            id=0,
            coordinate_system=CoordinateSystem(
                coordinate_system="world",
                name="mounting",
                reference_uid="",
                position=Vector3d(x=1000, y=0, z=0),
                rotation=RotationAngles(
                    angles=[0, 0, 0], type=RotationAngleTypes.EULER_ANGLES_EXTRINSIC_XYZ
                ),
            ),
        )

        # NC-1047
        await asyncio.sleep(5)

        await bridge.setup_blueprint()
        await asyncio.gather(move_robot(kuka, bridge=bridge), move_robot(ur10, bridge=bridge))


if __name__ == "__main__":
    asyncio.run(main())
