import asyncio
import contextlib
import logging

from nova.actions import MovementControllerContext
from nova.cell.movement_controller.io_pause_driver import IOPauseDriver
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
    target. If the signal's source itself goes away
    (``context.wait_for_pause_signal_loss`` returns), the controller stops
    evaluating the condition, so the adapter pauses the robot in its place and
    resumes the same way once the signal is back. After every resume it also
    watches the condition itself (``context.wait_for_pause_on_io_hold``): a start
    that reaches the controller while the condition holds again leaves the robot
    paused in a way the state stream cannot tell from the previous pause.

    Without a release waiter (``execute(..., resume_on_io_release=False)``, or a
    context built by hand) the condition is a stop condition: the execution ends
    where the robot stopped, as it did before IO pauses became resumable; a lost
    signal source then ends it with ``IOConditionUnavailable``. A waiter that
    fails pauses the robot and ends the execution with its error. See
    :mod:`nova.cell.movement_controller.io_pause_driver`.

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
    driver = _OneShotDriver(cursor, cursor.forward(), context)

    async def controller(response_stream):
        supervisor = asyncio.create_task(
            driver.run(), name=f"move_forward-supervisor-{context.motion_id}"
        )
        try:
            async for request in cursor.cntrl(response_stream):
                yield request
        finally:
            supervisor.cancel()
            with contextlib.suppress(asyncio.CancelledError):
                await supervisor
        if driver.error is not None:
            raise driver.error

    return controller


class _OneShotDriver:
    """Runs an :class:`IOPauseDriver` over the cursor and ends the cursor when it is done.

    Nobody else awaits the operation futures in one-shot execution: movement errors
    reach the protocol caller through ``cntrl`` itself, and a state stream that ends
    before the trajectory completes only resolves the future, matching the previous
    ``move_forward`` behaviour of returning once the state monitor is gone.
    """

    def __init__(
        self,
        cursor: TrajectoryCursor,
        operation: asyncio.Future[OperationResult],
        context: MovementControllerContext,
    ):
        self._cursor = cursor
        self._driver = IOPauseDriver(
            operation,
            start=cursor.forward,
            pause=cursor.pause,
            paused_on_io=lambda result: result.paused_on_io,
            location=lambda: cursor.current_location,
            wait_for_release=context.wait_for_pause_on_io_release,
            wait_for_hold=context.wait_for_pause_on_io_hold,
            wait_for_signal_loss=context.wait_for_pause_signal_loss,
        )
        self.error: BaseException | None = None

    async def run(self) -> None:
        try:
            resumable = await self._driver.run()
        except asyncio.CancelledError:
            raise
        except Exception as error:  # noqa: BLE001 — surfaced to the protocol caller
            logger.error(f"move_forward cannot continue the execution: {error!r}")
            self.error = error
            self._cursor.detach()
            return
        if not resumable:
            # A stop condition fired: the robot stands where it stopped and the
            # execution ends there.
            self._cursor.detach()
