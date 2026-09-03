import asyncio
import contextlib
import logging

from nova.actions import MovementControllerContext
from nova.cell.movement_controller.trajectory_cursor import OperationResult, TrajectoryCursor
from nova.types import MovementControllerFunction

logger = logging.getLogger(__name__)


def move_forward(context: MovementControllerContext) -> MovementControllerFunction:
    """Default movement controller: run the trajectory forward from start to end.

    This is a thin adapter over :class:`TrajectoryCursor`, which owns the
    ``executeTrajectory`` protocol; ``move_forward`` only configures a cursor for
    one-shot execution and starts it. The name and the plug-in seam
    (``MovementController`` / ``MovementControllerContext``) are kept for
    backwards compatibility.

    A controller-side IO pause (``context.pause_on_io``) suspends the movement
    without ending it: the controller holds the robot on path and waits for a
    new start, which it only honours once the condition has cleared. This
    adapter waits for that through ``context.wait_for_pause_on_io_release`` and
    starts again, as often as the signal comes and goes, until the trajectory is
    traversed — so ``execute()`` blocks through every pause and returns at the
    target. Without a release waiter (a context built by hand) the pause ends
    the execution early, as it did before IO pauses became resumable.

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
        # emits (each start overrides the previously attached overlay). A
        # context that resolved path triggers carries the finished overlay.
        set_outputs=(
            context.set_outputs
            if context.set_outputs is not None
            else context.combined_actions.to_set_io()
        ),
        start_on_io=context.start_on_io,
        pause_on_io=context.pause_on_io,
        initial_location=0.0,
        detach_on_standstill=True,
        emit_motion_events=False,
    )
    # Starting immediately is move_forward policy, not a cursor capability.
    operation = cursor.forward()

    async def controller(response_stream):
        supervisor = asyncio.create_task(
            _run_to_the_end(cursor, operation, context.wait_for_pause_on_io_release),
            name=f"move_forward-supervisor-{context.motion_id}",
        )
        try:
            async for request in cursor.cntrl(response_stream):
                yield request
        finally:
            supervisor.cancel()
            with contextlib.suppress(asyncio.CancelledError):
                await supervisor

    return controller


async def _run_to_the_end(
    cursor: TrajectoryCursor, operation: asyncio.Future[OperationResult], wait_for_release
) -> None:
    """Drive the one-shot execution through IO pauses until the trajectory ends.

    Nobody else awaits the operation futures in one-shot execution: movement
    errors reach the protocol caller through ``cntrl`` itself, and a state
    stream that ends before the trajectory completes only resolves the future,
    matching the previous ``move_forward`` behaviour of returning once the
    state monitor is gone. Retrieving the results here also keeps asyncio from
    warning about them.
    """
    while True:
        try:
            result = await operation
        except asyncio.CancelledError:
            # Detach (or our own cancellation) — nothing left to drive.
            return
        except Exception as error:  # noqa: BLE001 — surfaced through cntrl
            logger.debug(f"move_forward operation ended with an error: {error!r}")
            return
        if not result.paused_on_io:
            return
        if wait_for_release is None:
            logger.warning(
                "Movement paused on IO at location %s but no release waiter is available; "
                "ending the execution early.",
                result.final_location,
            )
            cursor.detach()
            return
        logger.info(
            "Movement paused on IO at location %s — waiting for the signal to clear",
            result.final_location,
        )
        await wait_for_release()
        logger.info("Pause signal cleared — resuming movement")
        # The start re-carries pause_on_io and the IO overlay, so the pause is
        # re-armed and remaining path outputs stay attached.
        operation = cursor.forward()
