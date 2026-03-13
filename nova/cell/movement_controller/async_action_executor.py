"""Executor for async actions during trajectory execution.

This module provides the AsyncActionExecutor class that manages the lifecycle
of AsyncAction execution during robot motion. It handles triggering actions
when trajectory locations are crossed, managing parallel vs blocking execution,
timeout handling, and error collection.

The executor integrates with movement controllers to:
- Monitor robot location and trigger actions at appropriate times
- Execute actions in parallel with motion (default) or pause motion (blocking)
- Track action results including completion location and timing
- Handle errors according to configured policy
"""

from __future__ import annotations

import asyncio
import logging
from dataclasses import dataclass
from datetime import datetime
from typing import TYPE_CHECKING, Any, Awaitable, Callable

from nova.actions.async_action import (
    ActionExecutionContext,
    AsyncAction,
    AsyncActionResult,
    ErrorHandlingMode,
)
from nova.types.state import RobotState

if TYPE_CHECKING:
    from nova.actions.container import ActionLocation

logger = logging.getLogger(__name__)


# Callback types for pause/resume
PauseCallback = Callable[[], Awaitable[None]]
ResumeCallback = Callable[[], Awaitable[None]]

# Callback type for error handling
ErrorCallback = Callable[[AsyncActionResult], Awaitable[None]]


@dataclass
class PendingAction:
    """An action waiting to be triggered at a specific location.

    Attributes:
        location: Path parameter where action should trigger.
        action: The AsyncAction to execute.
        triggered: Whether this action has been triggered.
    """

    location: float
    action: AsyncAction
    triggered: bool = False


@dataclass
class RunningAction:
    """An action currently being executed.

    Attributes:
        action: The AsyncAction being executed.
        task: The asyncio Task running the action.
        trigger_location: Location where the action was triggered.
        started_at: When execution started.
    """

    action: AsyncAction
    task: asyncio.Task[Any]
    trigger_location: float
    started_at: datetime


class AsyncActionExecutor:
    """Manages execution of AsyncActions during trajectory motion.

    The executor monitors robot location during trajectory execution and triggers
    registered async actions when their target locations are crossed. It supports
    both parallel execution (default) and blocking execution that pauses robot motion.

    Attributes:
        motion_group_id: Identifier of the motion group.
        error_mode: How to handle action errors (raise, collect, or callback).
        results: Completed action results.

    Example:
        ```python
        executor = AsyncActionExecutor(
            motion_group_id="0@ur10",
            async_actions=combined_actions.get_async_actions(),
            error_mode=ErrorHandlingMode.COLLECT,
        )

        # In state monitor loop:
        async for state in motion_group_states:
            needs_pause = await executor.check_and_trigger(
                current_location=state.location,
                current_state=robot_state,
            )
            if needs_pause:
                await pause_motion()
                await executor.wait_for_blocking_actions()
                await resume_motion()

        # After execution:
        for result in executor.results:
            print(f"{result.action.action_name}: {result.succeeded}")
        ```
    """

    def __init__(
        self,
        motion_group_id: str,
        async_actions: list[ActionLocation],
        error_mode: ErrorHandlingMode = ErrorHandlingMode.RAISE,
        error_callback: ErrorCallback | None = None,
        pause_callback: PauseCallback | None = None,
        resume_callback: ResumeCallback | None = None,
    ):
        """Initialize the executor.

        Args:
            motion_group_id: Identifier of the motion group executing trajectory.
            async_actions: List of ActionLocation containing AsyncAction instances.
            error_mode: How to handle action execution errors.
            error_callback: Callback for CALLBACK error mode.
            pause_callback: Called when blocking action needs to pause motion.
            resume_callback: Called when blocking action completes to resume motion.
        """
        self.motion_group_id = motion_group_id
        self.error_mode = error_mode
        self._error_callback = error_callback
        self._pause_callback = pause_callback
        self._resume_callback = resume_callback

        # Initialize pending actions from the provided list
        self._pending_actions: list[PendingAction] = [
            PendingAction(location=al.path_parameter, action=al.action)
            for al in async_actions
            if isinstance(al.action, AsyncAction)
        ]

        # Sort by location for efficient processing
        self._pending_actions.sort(key=lambda pa: pa.location)

        # Currently running actions (parallel, non-blocking)
        self._running_actions: list[RunningAction] = []

        # Completed results
        self._results: list[AsyncActionResult] = []

        # Track last known location for completion tracking
        self._last_location: float = 0.0

        # Lock for thread-safe result collection
        self._results_lock = asyncio.Lock()

        logger.debug(
            f"AsyncActionExecutor initialized with {len(self._pending_actions)} actions "
            f"for motion group {motion_group_id}"
        )

    @property
    def results(self) -> list[AsyncActionResult]:
        """Get completed action results (read-only copy)."""
        return list(self._results)

    @property
    def has_pending_actions(self) -> bool:
        """Check if there are untriggered actions remaining."""
        return any(not pa.triggered for pa in self._pending_actions)

    @property
    def has_running_actions(self) -> bool:
        """Check if there are actions currently executing."""
        return len(self._running_actions) > 0

    async def check_and_trigger(self, current_location: float, current_state: RobotState) -> bool:
        """Check if any actions should be triggered and start them.

        Called by the movement controller on each state update. Triggers any
        pending actions whose location has been crossed.

        Args:
            current_location: Current path parameter on trajectory.
            current_state: Current robot state.

        Returns:
            True if a blocking action was triggered and motion should pause.

        Raises:
            Exception: If error_mode is RAISE and an action fails.
        """
        self._last_location = current_location
        blocking_triggered = False

        # Check for actions to trigger
        for pending in self._pending_actions:
            if pending.triggered:
                continue

            # Trigger if we've reached or passed the action location
            if current_location >= pending.location:
                pending.triggered = True

                logger.info(
                    f"Triggering async action '{pending.action.action_name}' "
                    f"at location {current_location:.3f} (target: {pending.location:.3f})"
                )

                if pending.action.blocking:
                    # Blocking action - execute synchronously
                    blocking_triggered = True
                    if self._pause_callback:
                        await self._pause_callback()

                    await self._execute_blocking_action(
                        pending.action, current_location, current_state
                    )

                    if self._resume_callback:
                        await self._resume_callback()
                else:
                    # Non-blocking action - execute in background
                    self._start_parallel_action(pending.action, current_location, current_state)

        # Clean up completed parallel actions
        await self._collect_completed_actions()

        return blocking_triggered

    async def _execute_blocking_action(
        self, action: AsyncAction, trigger_location: float, current_state: RobotState
    ) -> None:
        """Execute a blocking action and wait for completion.

        Args:
            action: The AsyncAction to execute.
            trigger_location: Location where action was triggered.
            current_state: Robot state at trigger time.
        """
        started_at = datetime.now()
        context = ActionExecutionContext(
            trigger_location=trigger_location,
            current_state=current_state,
            motion_group_id=self.motion_group_id,
            action_name=action.action_name,
            action_args=action.args,
            action_kwargs=action.kwargs,
        )

        error: Exception | None = None
        return_value: Any = None
        timed_out = False

        try:
            handler = action.get_handler()
            if action.timeout:
                return_value = await asyncio.wait_for(handler(context), timeout=action.timeout)
            else:
                return_value = await handler(context)
        except asyncio.TimeoutError:
            timed_out = True
            error = asyncio.TimeoutError(
                f"Async action '{action.action_name}' timed out after {action.timeout}s"
            )
            logger.warning(str(error))
        except Exception as e:
            error = e
            logger.error(f"Async action '{action.action_name}' failed: {e}", exc_info=True)

        completed_at = datetime.now()

        result = AsyncActionResult(
            action=action,
            trigger_location=trigger_location,
            started_at=started_at,
            completed_at=completed_at,
            completion_location=self._last_location,  # For blocking, same as trigger
            return_value=return_value,
            error=error,
            was_blocking=True,
            timed_out=timed_out,
        )

        async with self._results_lock:
            self._results.append(result)

        await self._handle_error(result)

    def _start_parallel_action(
        self, action: AsyncAction, trigger_location: float, current_state: RobotState
    ) -> None:
        """Start a non-blocking action in the background.

        Args:
            action: The AsyncAction to execute.
            trigger_location: Location where action was triggered.
            current_state: Robot state at trigger time.
        """
        context = ActionExecutionContext(
            trigger_location=trigger_location,
            current_state=current_state,
            motion_group_id=self.motion_group_id,
            action_name=action.action_name,
            action_args=action.args,
            action_kwargs=action.kwargs,
        )

        async def execute_action() -> Any:
            handler = action.get_handler()
            if action.timeout:
                return await asyncio.wait_for(handler(context), timeout=action.timeout)
            return await handler(context)

        task = asyncio.create_task(execute_action(), name=f"async-action-{action.action_name}")

        running = RunningAction(
            action=action, task=task, trigger_location=trigger_location, started_at=datetime.now()
        )
        self._running_actions.append(running)

    async def _collect_completed_actions(self) -> None:
        """Collect results from completed parallel actions."""
        completed: list[RunningAction] = []
        still_running: list[RunningAction] = []

        for running in self._running_actions:
            if running.task.done():
                completed.append(running)
            else:
                still_running.append(running)

        self._running_actions = still_running

        for running in completed:
            completed_at = datetime.now()
            error: Exception | None = None
            return_value: Any = None
            timed_out = False

            try:
                return_value = running.task.result()
            except asyncio.TimeoutError:
                timed_out = True
                error = asyncio.TimeoutError(
                    f"Async action '{running.action.action_name}' timed out"
                )
            except Exception as e:
                error = e
                logger.error(
                    f"Async action '{running.action.action_name}' failed: {e}", exc_info=True
                )

            result = AsyncActionResult(
                action=running.action,
                trigger_location=running.trigger_location,
                started_at=running.started_at,
                completed_at=completed_at,
                completion_location=self._last_location,
                return_value=return_value,
                error=error,
                was_blocking=False,
                timed_out=timed_out,
            )

            async with self._results_lock:
                self._results.append(result)

            await self._handle_error(result)

    async def _handle_error(self, result: AsyncActionResult) -> None:
        """Handle action error according to error mode.

        Args:
            result: The completed action result.

        Raises:
            AsyncActionError: If error_mode is RAISE and action failed.
        """
        if result.error is None:
            return

        if self.error_mode == ErrorHandlingMode.RAISE:
            from nova.exceptions import AsyncActionError

            raise AsyncActionError(
                action_name=result.action.action_name,
                trigger_location=result.trigger_location,
                completion_location=result.completion_location,
                cause=result.error,
                was_blocking=result.was_blocking,
            )
        elif self.error_mode == ErrorHandlingMode.CALLBACK and self._error_callback:
            await self._error_callback(result)
        # COLLECT mode: error is already stored in result

    async def wait_for_all_actions(self) -> None:
        """Wait for all parallel actions to complete.

        Should be called at end of trajectory execution to ensure all
        background actions have finished.
        """
        if not self._running_actions:
            return

        logger.debug(f"Waiting for {len(self._running_actions)} parallel actions to complete")

        tasks = [ra.task for ra in self._running_actions]
        await asyncio.gather(*tasks, return_exceptions=True)
        await self._collect_completed_actions()

    async def cancel_all_actions(self) -> None:
        """Cancel all running parallel actions.

        Called when trajectory execution is aborted.
        """
        for running in self._running_actions:
            if not running.task.done():
                running.task.cancel()

        if self._running_actions:
            await asyncio.gather(*[ra.task for ra in self._running_actions], return_exceptions=True)
            self._running_actions.clear()

    def get_summary(self) -> dict[str, Any]:
        """Get execution summary for logging/debugging.

        Returns:
            Dictionary with execution statistics.
        """
        succeeded = sum(1 for r in self._results if r.succeeded)
        failed = sum(1 for r in self._results if not r.succeeded)
        total_duration = sum(r.duration_seconds for r in self._results)

        return {
            "motion_group_id": self.motion_group_id,
            "total_actions": len(self._pending_actions),
            "triggered": sum(1 for pa in self._pending_actions if pa.triggered),
            "completed": len(self._results),
            "succeeded": succeeded,
            "failed": failed,
            "still_running": len(self._running_actions),
            "total_duration_seconds": total_duration,
        }
