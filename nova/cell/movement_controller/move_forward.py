import asyncio
import contextlib
import logging

from nova.actions import MovementControllerContext
from nova.cell.movement_controller.policy import ExecutionPolicy
from nova.cell.movement_controller.trajectory_cursor import OperationResult, TrajectoryCursor
from nova.exceptions import ResumeNotTakenUp
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
    resumes the same way once the signal is back. Without a release waiter (a
    context built by hand) the pause ends the execution early, as it did before
    IO pauses became resumable. A waiter that fails ends the execution with its
    error.

    A resume the controller does not take up (it keeps reporting the old pause,
    see :attr:`ExecutionPolicy.resume_detect_s`) while the signal allows motion is
    started once more — but never later than
    :attr:`ExecutionPolicy.resume_window_s` after the release edge: a robot that
    starts moving long after the operator released the signal would surprise
    them. After that the adapter waits for a new edge (the signal pausing and
    releasing again). In strict mode the ignored resume fails the execution.

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
        policy=context.execution_policy,
        prismatic_joints=context.prismatic_joints,
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
    """Drives a one-shot execution to the end of the trajectory through IO pauses.

    Nobody else awaits the operation futures in one-shot execution: movement
    errors reach the protocol caller through ``cntrl`` itself, and a state
    stream that ends before the trajectory completes only resolves the future,
    matching the previous ``move_forward`` behaviour of returning once the state
    monitor is gone. Retrieving the results here also keeps asyncio from warning
    about them.
    """

    def __init__(
        self,
        cursor: TrajectoryCursor,
        operation: asyncio.Future[OperationResult],
        context: MovementControllerContext,
    ):
        self._cursor = cursor
        self._operation = operation
        self._wait_for_release = context.wait_for_pause_on_io_release
        self._wait_for_hold = context.wait_for_pause_on_io_hold
        self._wait_for_signal_loss = context.wait_for_pause_signal_loss
        self._policy: ExecutionPolicy = cursor.policy
        self._clock = cursor.clock
        # Watches, from a release edge on, whether the condition holds again.
        self._hold_watch: asyncio.Future[None] | None = None
        # Set by the guard when it paused the robot because the signal source
        # went away; the drive loop then resumes through the release wait.
        self._guard_pause: asyncio.Future[OperationResult] | None = None
        self._signal_lost = False
        self._resumed = asyncio.Event()
        self.error: BaseException | None = None

    async def run(self) -> None:
        guard = (
            asyncio.create_task(self._guard(), name="move_forward-signal-guard")
            if self._wait_for_signal_loss is not None and self._wait_for_release is not None
            else None
        )
        try:
            await self._drive()
        except asyncio.CancelledError:
            raise
        except Exception as error:  # noqa: BLE001 — surfaced to the protocol caller
            logger.error(f"move_forward cannot continue the execution: {error!r}")
            self.error = error
            self._cursor.detach()
        finally:
            for task in (guard, self._hold_watch):
                if task is not None:
                    task.cancel()
                    with contextlib.suppress(asyncio.CancelledError, Exception):
                        await task

    async def _drive(self) -> None:
        operation = self._operation
        # The release edge the current resume answers, and whether it was retried.
        edge_at: float | None = None
        retried = False
        while True:
            try:
                result = await operation
            except asyncio.CancelledError:
                task = asyncio.current_task()
                if task is not None and task.cancelling():
                    raise  # our own cancellation (detach / teardown)
                if self._guard_pause is None:
                    return  # the operation was cancelled from outside — nothing to drive
                # The guard superseded the movement with a pause: let it settle,
                # then resume once the signal is back.
                pause, self._guard_pause = self._guard_pause, None
                with contextlib.suppress(Exception):
                    await pause
                result = None
            except Exception as error:  # noqa: BLE001 — surfaced through cntrl
                logger.debug(f"move_forward operation ended with an error: {error!r}")
                return

            if result is not None and result.resume_not_taken_up:
                if await self._retry_resume(edge_at, retried):
                    retried = True
                    operation = self._cursor.forward()
                    continue
                # Not taken up within the window, or the condition came back: the
                # next start needs a new edge.
            elif result is not None and not result.paused_on_io and not self._signal_lost:
                return
            if self._wait_for_release is None:
                logger.warning(
                    "Movement paused on IO at location %s but no release waiter is available; "
                    "ending the execution early.",
                    result.final_location if result is not None else self._cursor.current_location,
                )
                self._cursor.detach()
                return
            logger.info(
                "Movement paused (%s) at location %s — waiting for the signal to allow motion",
                "signal source lost" if self._signal_lost else "IO condition",
                self._cursor.current_location,
            )
            await self._wait_for_release()
            edge_at = self._clock()
            retried = False
            self._watch_for_hold()
            logger.info("Pause signal cleared — resuming movement")
            self._signal_lost = False
            # The start re-carries pause_on_io and the IO overlay, so the pause is
            # re-armed and remaining path outputs stay attached.
            operation = self._cursor.forward()
            self._resumed.set()

    def _watch_for_hold(self) -> None:
        """From a release edge on, notice the condition holding again (push, no reads)."""
        if self._hold_watch is not None:
            self._hold_watch.cancel()
        self._hold_watch = (
            asyncio.ensure_future(self._wait_for_hold())
            if self._wait_for_hold is not None
            else None
        )

    def _condition_held_again(self) -> bool:
        watch = self._hold_watch
        return (
            watch is not None
            and watch.done()
            and not watch.cancelled()
            and (watch.exception() is None)
        )

    async def _retry_resume(self, edge_at: float | None, retried: bool) -> bool:
        """Decide on a resume the controller did not take up; ``True`` sends one more start.

        ``False`` means: wait for a new edge. Raises in strict mode, and when no new
        edge can be observed.
        """
        location = self._cursor.current_location
        if self._condition_held_again():
            logger.info(
                "Resume at location %s not taken up: the pause condition holds again — "
                "waiting for the signal to allow motion",
                location,
            )
            return False
        if self._policy.strict:
            raise ResumeNotTakenUp(
                f"the controller did not take up the resume at location {location} although "
                "the pause signal allows motion"
            )
        elapsed = self._clock() - edge_at if edge_at is not None else float("inf")
        if not retried and elapsed < self._policy.resume_window_s:
            logger.warning(
                "Resume at location %s not taken up although the signal allows motion — "
                "sending the start once more (%.0f ms after the signal was released)",
                location,
                elapsed * 1000,
            )
            return True
        if self._wait_for_hold is None:
            raise ResumeNotTakenUp(
                f"the controller did not take up the resume at location {location} within "
                f"{self._policy.resume_window_s:.1f} s of the signal release, and a new signal "
                "edge cannot be observed"
            )
        logger.warning(
            "Resume at location %s not taken up within %.1f s of the signal release — "
            "set the signal to pause and release it again to continue",
            location,
            self._policy.resume_window_s,
        )
        assert self._hold_watch is not None
        await asyncio.shield(self._hold_watch)
        return False

    async def _guard(self) -> None:
        """Pause the robot ourselves whenever the signal's source disappears."""
        assert self._wait_for_signal_loss is not None
        while True:
            await self._wait_for_signal_loss()
            logger.warning("Pause signal source lost — pausing the movement from the SDK")
            self._signal_lost = True
            self._resumed.clear()
            pause = self._cursor.pause()  # None when nothing is moving (already paused)
            if pause is not None:
                self._guard_pause = pause
            # Re-arm only after the drive loop resumed the movement; the loss
            # waiter would otherwise return immediately while the bus is down.
            await self._resumed.wait()
