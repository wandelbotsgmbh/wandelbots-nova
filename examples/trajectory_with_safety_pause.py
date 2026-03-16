"""
Example: Trajectory execution with safety signal monitoring and state persistence.

This example demonstrates:
- Robot moving along a planned trajectory
- Parallel task monitoring a boolean safety signal (bus IO)
- Pausing motion when safety signal becomes True
- Saving current trajectory location to file
- Resuming motion when signal becomes False

Prerequisites:
- A nova instance with virtual profinet enabled
- Add bus io variable 'safety' (Boolean type) in the setup application
- Run: uv run python trajectory_with_safety.py
"""

import asyncio
import json
import logging
from math import pi
from pathlib import Path

from decouple import config
from icecream import ic
from loguru import logger
from pydantic import BaseModel, Field
from setup import setup_bus_ios

import nova
from nova import api, run_program
from nova.actions import jnt, ptp
from nova.cell.controllers import virtual_controller
from nova.cell.movement_controller.trajectory_cursor import TrajectoryCursor
from nova.types import MotionSettings
from nova.types.pose import Pose
from nova.utils.io import IOChange, get_bus_io_value, set_bus_io_value, wait_for_bus_io

ic.configureOutput(includeContext=True)

# Suppress websockets.client log messages
logging.getLogger("websockets.client").setLevel(logging.WARNING)

STATE_FILE = Path("trajectory_pause_state.json")
LOG_LEVEL = config("LOG_LEVEL", default="INFO")


class TrajectoryState(BaseModel):
    """State of trajectory execution for pause/resume."""

    trajectory_location: float = Field(description="Current location on trajectory")
    is_paused: bool = Field(default=False, description="Whether trajectory is currently paused")
    paused_joint_position: list[float] | None = Field(
        default=None, description="Joint position when trajectory was paused"
    )


def save_state(state: TrajectoryState) -> None:
    """Save trajectory state to file."""
    STATE_FILE.write_text(state.model_dump_json(indent=2))
    logger.info(
        f"Saved state: location={state.trajectory_location:.3f}, paused={state.is_paused}, joint_position={state.paused_joint_position}"
    )


def load_state() -> TrajectoryState | None:
    """Load trajectory state from file."""
    if not STATE_FILE.exists():
        return None
    try:
        data = json.loads(STATE_FILE.read_text())
        state = TrajectoryState(**data)
        logger.info(
            f"Loaded state: location={state.trajectory_location:.3f}, paused={state.is_paused}, joint_position={state.paused_joint_position}"
        )
        return state
    except Exception as e:
        logger.error(f"Failed to load state: {e}")
        return None


async def setup_signals(ctx: nova.ProgramContext) -> None:
    """Setup bus IOs for the program."""
    # Configure the safety bus IO
    io_configs = [{"name": "safety", "type": "bool", "initial_value": False}]

    await setup_bus_ios(ctx.nova.api.bus_ios_api, ctx.cell.id, io_configs)
    logger.info("Bus IO setup completed")


async def safety_monitor(
    cursor: TrajectoryCursor, state: TrajectoryState, motion_group: nova.MotionGroup
) -> None:
    """Monitor safety signal and pause/resume trajectory execution.

    When safety becomes True: pause motion and save current location
    When safety becomes False: resume motion
    """
    logger.info("Safety monitor started")

    while True:
        try:
            # Wait for safety signal to change
            def on_change(changes: dict[str, IOChange]) -> bool:
                if "safety" not in changes:
                    return False

                new_value = changes["safety"].new_value
                old_value = changes["safety"].old_value

                # Trigger on any change
                return new_value != old_value

            await wait_for_bus_io(["safety"], on_change=on_change)

            # Get current signal value
            values = await get_bus_io_value(["safety"])
            should_pause = values["safety"]

            if should_pause and not state.is_paused:
                # Pause requested
                logger.warning("Safety signal activated - PAUSING trajectory")
                # await cursor.pause()
                try:
                    ic(cursor._current_location)
                    await asyncio.shield(cursor.pause())
                    ic(cursor._current_location)
                except asyncio.CancelledError:
                    ic()
                    pass
                ic()
                # Save current location and joint position
                state.trajectory_location = cursor._current_location
                try:
                    current_joints = await asyncio.shield(motion_group.joints())
                except asyncio.CancelledError:
                    ic()
                    pass
                ic(current_joints)
                state.paused_joint_position = list(current_joints)
                state.is_paused = True
                save_state(state)

            elif not should_pause and state.is_paused:
                # Resume requested
                logger.info("Safety signal cleared - RESUMING trajectory")
                await cursor.forward()

                state.is_paused = False
                state.paused_joint_position = None
                save_state(state)

        except Exception as e:
            logger.error(f"Safety monitor error: {e}")
            break


@nova.program(
    name="Trajectory with Safety Pause",
    preconditions=nova.ProgramPreconditions(
        controllers=[
            virtual_controller(
                name="kuka-kr16-r2010",
                manufacturer=api.models.Manufacturer.KUKA,
                type=api.models.VirtualControllerTypes.KUKA_KR16_R2010_2,
            )
        ],
        cleanup_controllers=False,
    ),
)
async def main(ctx: nova.ProgramContext):
    """Main program: Execute trajectory with safety monitoring."""

    # Setup bus IOs
    await setup_signals(ctx)

    # Initialize safety signal to False (no pause)
    await set_bus_io_value({"safety": False})
    logger.info("Initialized safety signal to False")

    # Setup robot
    cell = ctx.cell
    controller = await cell.controller("kuka-kr16-r2010")
    motion_group = controller[0]
    tcp = "Flange"

    # Home position
    home_joints = (pi / 2, -pi / 2, pi / 2, 0, pi / 2, pi)
    home_pose = Pose(-0.0, -1020.0, 1497.0, -2.221, 2.221, 0.0)

    fast = MotionSettings(
        tcp_velocity_limit=800.0,  # mm/s
        tcp_acceleration_limit=600.0,  # mm/s²
    )

    # Define a trajectory with multiple waypoints
    actions = [
        jnt(home_joints, fast),
        ptp(Pose((300, 0, 100, 0, 0, 0)) @ home_pose, fast),
        ptp(Pose((300, 300, 100, 0, 0, 0)) @ home_pose, fast),
        ptp(Pose((0, 300, 100, 0, 0, 0)) @ home_pose, fast),
        ptp(Pose((-300, 300, 100, 0, 0, 0)) @ home_pose, fast),
        ptp(Pose((-300, 0, 100, 0, 0, 0)) @ home_pose, fast),
        ptp(Pose((-300, -300, 100, 0, 0, 0)) @ home_pose, fast),
        ptp(Pose((0, -300, 100, 0, 0, 0)) @ home_pose, fast),
        ptp(Pose((300, -300, 100, 0, 0, 0)) @ home_pose, fast),
        jnt(home_joints, fast),
    ]

    logger.info(f"Planning trajectory with {len(actions)} actions")
    joint_trajectory = await motion_group.plan(actions, tcp, start_joint_position=home_joints)

    # Load or create state
    state = load_state()
    if state is None:
        state = TrajectoryState(trajectory_location=0.0, is_paused=False)
        logger.info("Starting from beginning (no saved state)")
    else:
        logger.info(f"Resuming from saved location: {state.trajectory_location:.3f}")

    # Create trajectory cursor
    motion_id = await motion_group._load_planned_motion(joint_trajectory, tcp)
    cursor = TrajectoryCursor(
        motion_id,
        motion_group.stream_state(),
        joint_trajectory,
        actions,
        initial_location=state.trajectory_location,
        detach_on_standstill=True,
    )

    # Start safety monitor in parallel
    monitor_task = asyncio.create_task(safety_monitor(cursor, state, motion_group))

    # Execute trajectory
    logger.info("Starting trajectory execution")
    exec_api = ctx.nova.api.trajectory_execution_api
    execution_task = asyncio.create_task(
        exec_api.execute_trajectory(
            cell=cell.id, controller=controller.id, client_request_generator=cursor.cntrl
        )
    )

    # Move forward along trajectory
    try:
        await cursor.forward()
    except asyncio.CancelledError:
        ic()
        pass
    ic()
    # await cursor.forward_to(6.0)
    # cursor.detach()

    # Wait for execution to complete
    try:
        await execution_task
        logger.info("Trajectory execution completed successfully")

        # Update state
        # save_state(state)

    except Exception as e:
        logger.error(f"Trajectory execution error: {e}")

        # Save current location on error
        state.trajectory_location = cursor._current_location
        state.is_paused = True
        save_state(state)
        raise

    finally:
        # Cleanup
        cursor.detach()
        ic()
        monitor_task.cancel()
        try:
            await monitor_task
        except asyncio.CancelledError:
            pass

        logger.info("Program finished")


if __name__ == "__main__":
    run_program(main)
