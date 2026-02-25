"""
Simple reset program that moves the robot to home position.

This program moves the robot to a predefined home joint position using
joint space motion (jnt command).

Prerequisites:
- A nova instance with a robot controller
- Run: uv run python reset.py
"""

import logging
from math import pi

from decouple import config
from loguru import logger

import nova
from nova import api, run_program
from nova.actions import jnt
from nova.cell.controllers import virtual_controller
from nova.types import MotionSettings

# Suppress websockets.client log messages
logging.getLogger("websockets.client").setLevel(logging.WARNING)

LOG_LEVEL = config("LOG_LEVEL", default="INFO")


@nova.program(
    name="Reset to Home",
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
    """Reset robot to home position."""

    logger.info("Starting reset program")

    # Setup robot
    cell = ctx.cell
    controller = await cell.controller("kuka-kr16-r2010")
    motion_group = controller[0]
    tcp = "Flange"

    # Home joint position
    home_joints = (pi/2, -pi/2, pi/2, 0, pi/2, pi)

    logger.info(f"Moving to home position: {home_joints}")
    fast = MotionSettings(
        tcp_velocity_limit=800.0,  # mm/s
        tcp_acceleration_limit=600.0,  # mm/s²
    )

    # Move to home position
    await motion_group.plan_and_execute([jnt(home_joints, fast)], tcp)

    logger.info("Robot is now at home position")


if __name__ == "__main__":
    run_program(main)
