"""Example: Using async actions during trajectory execution.

This example demonstrates how to execute arbitrary async functions at specific
locations along a robot trajectory. Async actions can run in parallel with
robot motion (default) or block motion until complete.

Use cases include:
- Logging position data at specific points
- Triggering external systems (cameras, sensors, databases)
- Sending notifications or status updates
- Pausing motion to perform inspection or verification
"""

import asyncio
import logging
from datetime import datetime

from nova import Nova
from nova.actions import ActionExecutionContext, async_action, lin, ptp, register_async_action
from nova.types import Pose

logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)


# ============================================================================
# Define async action handlers
# ============================================================================


async def log_position(ctx: ActionExecutionContext):
    """Log the robot position when this action triggers.

    This is a non-blocking action that runs in parallel with motion.
    """
    logger.info(
        f"[Location {ctx.trigger_location:.2f}] "
        f"Robot at joints: {[f'{j:.2f}' for j in ctx.current_state.joints]}"
    )


async def send_notification(ctx: ActionExecutionContext):
    """Send a notification to an external system.

    Demonstrates passing arguments to async actions.
    """
    message = ctx.action_kwargs.get("message", "No message")
    target = ctx.action_kwargs.get("target", "default")

    # Simulate sending notification
    logger.info(f"[Location {ctx.trigger_location:.2f}] Sending '{message}' to {target}")
    await asyncio.sleep(0.1)  # Simulate network delay


async def capture_image(ctx: ActionExecutionContext):
    """Simulate capturing an image - a blocking action that pauses motion.

    This is a blocking action. The robot will pause at this location
    until the image capture completes.
    """
    logger.info(f"[Location {ctx.trigger_location:.2f}] Capturing image...")
    await asyncio.sleep(0.5)  # Simulate camera capture time
    logger.info(f"[Location {ctx.trigger_location:.2f}] Image captured!")
    return {"timestamp": datetime.now().isoformat(), "status": "captured"}


async def quality_check(ctx: ActionExecutionContext):
    """Perform a quality check with timeout.

    Demonstrates timeout handling for blocking actions.
    """
    check_type = ctx.action_args[0] if ctx.action_args else "default"
    logger.info(f"[Location {ctx.trigger_location:.2f}] Running {check_type} quality check...")
    await asyncio.sleep(0.3)
    logger.info(f"[Location {ctx.trigger_location:.2f}] Quality check passed!")
    return True


# ============================================================================
# Register handlers
# ============================================================================

# Register all action handlers with the global registry
register_async_action("log_position", log_position)
register_async_action("send_notification", send_notification)
register_async_action("capture_image", capture_image)
register_async_action("quality_check", quality_check)


# ============================================================================
# Main example
# ============================================================================


async def main():
    """Demonstrate async actions during trajectory execution."""

    # Define waypoints
    home = Pose(position=(400, 0, 400), orientation=(1, 0, 0, 0))
    point_a = Pose(position=(500, 100, 300), orientation=(1, 0, 0, 0))
    point_b = Pose(position=(500, -100, 300), orientation=(1, 0, 0, 0))
    point_c = Pose(position=(400, 0, 350), orientation=(1, 0, 0, 0))

    # Build action sequence with async actions interspersed
    actions = [
        # Move to start position
        ptp(home),
        # Log position after reaching home (parallel action)
        async_action("log_position"),
        # Move to point A
        lin(point_a),
        # Capture image at point A (blocking - pauses motion)
        async_action("capture_image", blocking=True),
        # Move to point B
        lin(point_b),
        # Send notification (parallel action with kwargs)
        async_action("send_notification", message="Reached point B", target="monitoring_system"),
        # Run quality check with timeout (blocking)
        async_action("quality_check", "visual", blocking=True, timeout=2.0),
        # Move to point C
        lin(point_c),
        # Final position log
        async_action("log_position"),
        # Return to home
        ptp(home),
    ]

    logger.info("=" * 60)
    logger.info("Async Actions Example")
    logger.info("=" * 60)
    logger.info(f"Total actions: {len(actions)}")
    logger.info("Async actions in sequence:")
    for i, action in enumerate(actions):
        if hasattr(action, "action_name"):
            blocking = "BLOCKING" if action.blocking else "parallel"
            logger.info(f"  [{i}] {action.action_name} ({blocking})")

    # Connect to Nova and execute
    async with Nova() as nova:
        cell = nova.cell()
        controller = await cell.controller("ur")
        motion_group = controller[0]

        # Get current TCP
        tcp = await motion_group.tcp()
        logger.info(f"Using TCP: {tcp}")

        # Plan the trajectory
        logger.info("Planning trajectory...")
        trajectory = await motion_group.plan(actions, tcp)
        logger.info(f"Trajectory planned with {len(trajectory.joint_positions)} points")

        # Execute with streaming to see async action results
        logger.info("Executing trajectory with async actions...")
        async for state in motion_group.stream_execute(trajectory, tcp, actions=actions):
            # State updates are streamed during execution
            # Async actions are triggered automatically based on location
            pass

        logger.info("Trajectory execution complete!")


if __name__ == "__main__":
    asyncio.run(main())
