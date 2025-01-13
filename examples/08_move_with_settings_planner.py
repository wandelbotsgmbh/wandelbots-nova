from nova import Nova
from nova.trajectory import Trajectory
import asyncio

"""
Example: Perform relative movements with a robot using settings.

Prerequisites:
- At least one robot added to the cell.
"""


async def main():
    nova = Nova()
    cell = nova.cell()
    controllers = await cell.controllers()
    controller = controllers[0]

    async with controller[0] as motion_group:
        home_joints = await motion_group.joints()
        tcp_names = await motion_group.tcp_names()
        tcp = tcp_names[0]

        current_pose = await motion_group.tcp_pose(tcp)

        # Start building a trajectory
        trajectory = Trajectory()

        # You can provide settings which are applied to that specific motion only
        trajectory.move(via="jnt", to=home_joints, velocity=10)
        trajectory.move(via="ptp", to=current_pose @ (1,0,0,0,0,0), velocity=50)

        # You can move without any additional settings
        trajectory.move(via="ptp", to=current_pose @ (100, 0, 0, 0, 0, 0))
        trajectory.move(via="jnt", to=home_joints, velocity=100)

        # You can start a movement block which uses the same settings
        with trajectory.using_settings(velocity=400):
            trajectory.move(via="ptp", to=current_pose @ (100, 100, 0, 0, 0, 0))
            trajectory.move(via="jnt", to=home_joints)

            # You can override the settings in a specific motion
            trajectory.move(via="ptp", to=current_pose @ (0, 100, 0, 0, 0, 0), velocity=30)

        # You can continue building your trajectory
        trajectory.move(via="jnt", to=home_joints)


        # Build you actions to execute on a real/virtual robot
        actions = trajectory.build()

        joint_trajectory = await motion_group.plan(actions, tcp)
        await motion_group.execute(joint_trajectory, tcp, actions=actions)


if __name__ == "__main__":
    asyncio.run(main())
