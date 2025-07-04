"""
This example demonstrates how to stream the robot state into the timeline "time".
This can be used to record the actual motions of the robot.
"""

import asyncio
import signal
from contextlib import asynccontextmanager

import nova
from nova import Nova, api
from nova.actions import cartesian_ptp, joint_ptp
from nova.cell import virtual_controller
from nova.program import ProgramPreconditions
from nova.types import MotionSettings, Pose
from nova_rerun_bridge import NovaRerunBridge


@asynccontextmanager
async def handle_shutdown():
    stop = asyncio.Future()

    def signal_handler():
        if not stop.done():
            stop.set_result(None)

    loop = asyncio.get_event_loop()
    loop.add_signal_handler(signal.SIGINT, signal_handler)
    try:
        yield stop
    finally:
        loop.remove_signal_handler(signal.SIGINT)


@nova.program(
    name="08_stream_robot",
    preconditions=ProgramPreconditions(
        controllers=[
            virtual_controller(
                name="ur10",
                manufacturer=api.models.Manufacturer.UNIVERSALROBOTS,
                type=api.models.VirtualControllerTypes.UNIVERSALROBOTS_MINUS_UR10E,
            )
        ],
        cleanup_controllers=False,
    ),
)
async def test():
    async with Nova() as nova, NovaRerunBridge(nova) as bridge:
        await bridge.setup_blueprint()

        cell = nova.cell()
        controller = await cell.controller("ur10")

        # Connect to the controller and activate motion groups
        async with controller[0] as motion_group:
            # this streams the current robot state into the timeline "time"
            # use this to record actual motions of the robot
            await bridge.start_streaming(motion_group)

            # In addition to log the robot state you can still log other data
            # like trajectories into the "time_interval_0.016" timeline
            await bridge.log_saftey_zones(motion_group)

            home_joints = await motion_group.joints()
            tcp_names = await motion_group.tcp_names()
            tcp = tcp_names[0]

            # Get current TCP pose and offset it slightly along the x-axis
            current_pose = await motion_group.tcp_pose(tcp)
            target_pose = current_pose @ Pose((1, 0, 0, 0, 0, 0))

            actions = [
                joint_ptp(home_joints),
                cartesian_ptp(target_pose),
                joint_ptp(home_joints),
                cartesian_ptp(target_pose @ [100, 0, 0, 0, 0, 0]),
                joint_ptp(home_joints),
                cartesian_ptp(target_pose @ (100, 100, 0, 0, 0, 0)),
                joint_ptp(home_joints),
                cartesian_ptp(target_pose @ Pose((0, 100, 0, 0, 0, 0))),
                joint_ptp(home_joints),
            ]

            # you can update the settings of the action
            for action in actions:
                action.settings = MotionSettings(tcp_velocity_limit=200)

            joint_trajectory = await motion_group.plan(actions, tcp)

            await bridge.log_actions(actions)
            await bridge.log_trajectory(joint_trajectory, tcp, motion_group)

            # Keep streaming until Ctrl+C
            async with handle_shutdown() as stop:
                try:
                    await stop
                except asyncio.CancelledError:
                    pass
                finally:
                    await bridge.stop_streaming()


if __name__ == "__main__":
    asyncio.run(test())
