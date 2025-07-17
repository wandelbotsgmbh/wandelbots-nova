"""
Execution Speed Control with Enhanced WebSocket Integration

Demonstrates comprehensive robot control with the new integrated WebSocket system:
- Automatic robot registration and discovery
- Real-time event broadcasting to external clients
- Speed changes during execution (10% → 50% → 100% → 25%)
- Pause/resume functionality
- Direction control (forward → backward)
- Support for parallel robot executions with async gather
- Program lifecycle events (start/stop notifications)

Perfect for external control applications, web UIs, or third-party integrations.
The enhanced system automatically registers robots when motion groups are created
and provides comprehensive event notifications for external tools.
"""

import asyncio

import nova
from nova import Nova, api
from nova.actions import cartesian_ptp
from nova.actions.base import Action
from nova.cell import virtual_controller
from nova.external_control import WebSocketControl
from nova.playback import PlaybackSpeedPercent, get_playback_manager
from nova.playback.playback_events import PlaybackDirection, PlaybackState
from nova.program import ProgramPreconditions
from nova.types import Pose


@nova.program(
    name="Enhanced WebSocket Speed Control Demo",
    playback_speed_percent=5,  # Start very slow for external control testing
    external_control=WebSocketControl(),  # Enable enhanced WebSocket control with events
    preconditions=ProgramPreconditions(
        controllers=[
            virtual_controller(
                name="urprimary",
                manufacturer=api.models.Manufacturer.UNIVERSALROBOTS,
                type=api.models.VirtualControllerTypes.UNIVERSALROBOTS_MINUS_UR10E,
            ),
            virtual_controller(
                name="ursecondary",
                manufacturer=api.models.Manufacturer.UNIVERSALROBOTS,
                type=api.models.VirtualControllerTypes.UNIVERSALROBOTS_MINUS_UR10E,
            ),
        ],
        cleanup_controllers=False,
    ),
)
async def main():
    async with Nova() as nova:
        cell = nova.cell()

        # Get both controllers for parallel execution demo
        primary_controller = await cell.controller("urprimary")
        secondary_controller = await cell.controller("ursecondary")

        # Demonstrate parallel robot control with async gather
        async with (
            primary_controller[0] as motion_group_primary,
            secondary_controller[0] as motion_group_secondary,
        ):
            print("🤖 Enhanced WebSocket Control Demo Started")
            print(f"✅ Primary robot registered: {motion_group_primary.motion_group_id}")
            print(f"✅ Secondary robot registered: {motion_group_secondary.motion_group_id}")
            print("🌐 WebSocket server running on ws://localhost:8765")

            # Create movement sequences for both robots
            primary_actions = await create_movement_sequence(motion_group_primary)
            secondary_actions = await create_movement_sequence(motion_group_secondary)

            # Plan trajectories for both robots
            tcp_names_primary = await motion_group_primary.tcp_names()
            tcp_names_secondary = await motion_group_secondary.tcp_names()

            joint_trajectory_primary = await motion_group_primary.plan(
                primary_actions, tcp_names_primary[0]
            )
            joint_trajectory_secondary = await motion_group_secondary.plan(
                secondary_actions, tcp_names_secondary[0]
            )

            # Start background task for demonstrating runtime control on both robots
            async def runtime_control():
                manager = get_playback_manager()
                print("\n🎮 Starting runtime control demonstration...")

                # Wait a bit for execution to start
                await asyncio.sleep(1)

                # Demonstrate speed control on primary robot
                print("⚡ Setting primary robot speed to 50%")
                manager.set_external_override(
                    motion_group_primary.motion_group_id, PlaybackSpeedPercent(value=50)
                )

                await asyncio.sleep(2)

                # Demonstrate speed control on secondary robot
                print("⚡ Setting secondary robot speed to 75%")
                manager.set_external_override(
                    motion_group_secondary.motion_group_id, PlaybackSpeedPercent(value=75)
                )

                await asyncio.sleep(2)

                # Pause primary robot
                print("⏸️  Pausing primary robot")
                manager.pause(motion_group_primary.motion_group_id)

                await asyncio.sleep(3)

                # Resume primary robot with different speed
                print("▶️  Resuming primary robot at 100% speed")
                manager.set_external_override(
                    motion_group_primary.motion_group_id,
                    PlaybackSpeedPercent(value=100),
                    state=PlaybackState.PLAYING,
                )

                await asyncio.sleep(2)

                # Demonstrate direction control on secondary robot
                print("⏪ Setting secondary robot to backward direction")
                manager.pause(motion_group_secondary.motion_group_id)
                manager.set_external_override(
                    motion_group_secondary.motion_group_id,
                    PlaybackSpeedPercent(value=25),
                    state=PlaybackState.PLAYING,
                    direction=PlaybackDirection.BACKWARD,
                )

                await asyncio.sleep(3)

                # Reset both robots to normal forward motion
                print("🔄 Resetting both robots to forward motion")
                manager.set_external_override(
                    motion_group_primary.motion_group_id,
                    PlaybackSpeedPercent(value=100),
                    state=PlaybackState.PLAYING,
                    direction=PlaybackDirection.FORWARD,
                )
                manager.set_external_override(
                    motion_group_secondary.motion_group_id,
                    PlaybackSpeedPercent(value=100),
                    state=PlaybackState.PLAYING,
                    direction=PlaybackDirection.FORWARD,
                )

            control_task = asyncio.create_task(runtime_control())

            # Execute both robots in parallel using asyncio.gather
            print("\n🚀 Starting parallel execution of both robots...")
            primary_execution = motion_group_primary.stream_execute(
                joint_trajectory_primary, tcp_names_primary[0], actions=primary_actions
            )
            secondary_execution = motion_group_secondary.stream_execute(
                joint_trajectory_secondary, tcp_names_secondary[0], actions=secondary_actions
            )

            # Run both executions in parallel
            await asyncio.gather(
                execute_robot_stream(primary_execution, "Primary"),
                execute_robot_stream(secondary_execution, "Secondary"),
                control_task,
                return_exceptions=True,
            )

            print("\n✅ Demo completed - both robots finished execution")
            print("📊 Check WebSocket events for complete lifecycle tracking")


async def create_movement_sequence(motion_group) -> list[Action]:
    """Create a movement sequence for the given motion group"""
    tcp_names = await motion_group.tcp_names()
    tcp = tcp_names[0]

    current_pose = await motion_group.tcp_pose(tcp)
    target_pose1 = current_pose @ Pose((200, 0, 0, 0, 0, 0))
    target_pose2 = current_pose @ Pose((200, 200, 0, 0, 0, 0))
    target_pose3 = current_pose @ Pose((0, 200, 0, 0, 0, 0))
    target_pose4 = current_pose @ Pose((0, 200, 100, 0, 0, 0))

    return [
        cartesian_ptp(target_pose1),
        cartesian_ptp(target_pose2),
        cartesian_ptp(target_pose3),
        cartesian_ptp(target_pose4),
        cartesian_ptp(current_pose),
        cartesian_ptp(target_pose1),
        cartesian_ptp(target_pose2),
        cartesian_ptp(current_pose),
    ]


async def execute_robot_stream(robot_stream, robot_name: str):
    """Execute a robot stream and log progress"""
    print(f"🎯 {robot_name} robot execution started")
    async for state in robot_stream:
        # Process execution state
        pass
    print(f"✅ {robot_name} robot execution completed")


if __name__ == "__main__":
    asyncio.run(main())
