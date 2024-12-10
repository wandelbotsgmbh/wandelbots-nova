from nova import Nova, Pose, ptp, jnt, pi

# TODO: public interface
from nova.types.action import WriteAction
import asyncio


async def main():
    nova = Nova()
    cell = nova.cell()
    controller = await cell.controller("ur")

    # Define a home position
    home_joints = (0, -pi / 4, -pi / 4, -pi / 4, pi / 4, 0)

    # Connect to the controller and activate motion groups
    async with controller:
        motion_group = controller.get_motion_group()

        # Get current TCP pose and offset it slightly along the x-axis
        current_pose = await motion_group.tcp_pose("Flange")
        target_pose = current_pose @ Pose((100, 0, 0, 0, 0, 0))
        actions = [
            jnt(home_joints),
            # controller.write_on_path("digital_out[0]", value=False),
            WriteAction(device_id="ur", key="digital_out[0]", value=False),
            ptp(target_pose),
            jnt(home_joints),
        ]

        # io_value = await controller.read_io("digital_out[0]")

        # plan_response = await motion_group.plan(trajectory, tcp="Flange")
        # print(plan_response)

        def print_motion(motion):
            print(motion)

        await motion_group.run(actions, tcp="Flange", initial_movement_consumer=print_motion)
        await motion_group.run(actions, tcp="Flange")
        await motion_group.run(ptp(target_pose), tcp="Flange")


if __name__ == "__main__":
    asyncio.run(main())
