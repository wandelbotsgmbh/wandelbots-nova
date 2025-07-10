"""
Execution Speed Control

Demonstrates dynamic control of robot execution during runtime:
- Speed changes during execution (10% → 50% → 100% → 25%)
- Pause/resume functionality
- Direction control (forward → backward)

Perfect for VS Code extensions, web UIs, or external control applications.
"""

import asyncio

import nova
from nova import Nova, api, viewers
from nova.actions import cartesian_ptp
from nova.cell import virtual_controller
from nova.core.playback_control import PlaybackSpeedPercent, get_playback_manager
from nova.program import ProgramPreconditions
from nova.types import Pose


@nova.program(
    name="Playback Speed Runtime Demo",
    viewer=viewers.Rerun(),
    playback_speed_percent=10,  # Start intentionally at 10% speed
    preconditions=ProgramPreconditions(
        controllers=[
            virtual_controller(
                name="ur",
                manufacturer=api.models.Manufacturer.UNIVERSALROBOTS,
                type=api.models.VirtualControllerTypes.UNIVERSALROBOTS_MINUS_UR10E,
            )
        ],
        cleanup_controllers=False,
    ),
)
async def main():
    async with Nova() as nova:
        cell = nova.cell()
        controller = await cell.controller("ur")

        async with controller[0] as motion_group:
            # Get current TCP pose and create movement
            tcp_names = await motion_group.tcp_names()
            tcp = tcp_names[0]

            current_pose = await motion_group.tcp_pose(tcp)
            target_pose = current_pose @ Pose((50, 0, 0, 0, 0, 0))

            # Create longer movement sequence for demo
            actions = [
                cartesian_ptp(target_pose),
                cartesian_ptp(current_pose),
                cartesian_ptp(target_pose),
                cartesian_ptp(current_pose),
                cartesian_ptp(target_pose),
                cartesian_ptp(current_pose),
            ]

            joint_trajectory = await motion_group.plan(actions, tcp)

            # Start background task for runtime control
            async def runtime_control():
                manager = get_playback_manager()

                # Speed changes during execution
                await asyncio.sleep(1)
                manager.set_external_override(motion_group.id, PlaybackSpeedPercent(50))

                await asyncio.sleep(1)
                manager.set_external_override(motion_group.id, PlaybackSpeedPercent(100))

                await asyncio.sleep(1)
                manager.set_external_override(motion_group.id, PlaybackSpeedPercent(25))

                # Pause/resume demonstration
                await asyncio.sleep(2)
                manager.pause(motion_group.id)

                await asyncio.sleep(2)
                manager.resume(motion_group.id)

                # Direction control demonstration
                await asyncio.sleep(2)
                manager.pause(motion_group.id)
                manager.set_external_override(motion_group.id, PlaybackSpeedPercent(100))
                manager.set_direction_backward(motion_group.id)
                manager.resume(motion_group.id)

                await asyncio.sleep(4)
                manager.pause(motion_group.id)
                manager.set_direction_backward(motion_group.id)

            control_task = asyncio.create_task(runtime_control())

            # Execute movement with runtime control
            async for state in motion_group.stream_execute(joint_trajectory, tcp, actions=actions):
                pass

            await control_task


if __name__ == "__main__":
    asyncio.run(main())
