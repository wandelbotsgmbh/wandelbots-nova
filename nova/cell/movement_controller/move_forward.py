import asyncio
import logging

from nova.actions import MovementControllerContext
from nova.cell.movement_controller.trajectory_cursor import TrajectoryCursor
from nova.types import MovementControllerFunction

logger = logging.getLogger(__name__)


def move_forward(context: MovementControllerContext) -> MovementControllerFunction:
    """Default movement controller: run the trajectory forward from start to end.

    This is a thin adapter over :class:`TrajectoryCursor`, which owns the
    ``executeTrajectory`` protocol; ``move_forward`` only configures a cursor for
    one-shot execution and starts it. The name and the plug-in seam
    (``MovementController`` / ``MovementControllerContext``) are kept for
    backwards compatibility.

    Must be called with a running event loop: the cursor schedules its
    background initialization at construction time.
    """
    cursor = TrajectoryCursor(
        motion_id=context.motion_id,
        motion_group_state_stream=context.motion_group_state_stream_gen,
        joint_trajectory=context.joint_trajectory,
        # An empty list carries no action metadata; it must not be mistaken
        # for a zero-length trajectory.
        actions=list(context.combined_actions.items) or None,
        # Server-side IO overlay, attached by the cursor to every start it
        # emits (each start overrides the previously attached overlay).
        set_outputs=context.combined_actions.to_set_io(),
        start_on_io=context.start_on_io,
        pause_on_io=context.pause_on_io,
        initial_location=0.0,
        detach_on_standstill=True,
        emit_motion_events=False,
    )
    # Starting immediately is move_forward policy, not a cursor capability.
    operation = cursor.forward()
    operation.add_done_callback(_consume_operation_outcome)
    return cursor.cntrl


def _consume_operation_outcome(operation: asyncio.Future) -> None:
    """Retrieve the one-shot operation's result so asyncio never warns about it.

    Nobody awaits this future in one-shot execution: movement errors reach the
    protocol caller through ``cntrl`` itself. A state stream that ends before
    the trajectory completes only resolves the future, matching the previous
    ``move_forward`` behaviour of returning once the state monitor is gone.
    """
    if operation.cancelled():
        return
    error = operation.exception()
    if error is not None:
        logger.debug(f"move_forward operation ended with an error: {error!r}")
