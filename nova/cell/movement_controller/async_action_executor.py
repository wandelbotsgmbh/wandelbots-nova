"""Executor for async actions during trajectory execution.

This module provides the AsyncActionExecutor class that manages the lifecycle
of AsyncAction, AwaitAction, and WaitUntilAction execution during robot motion.

The executor integrates with movement controllers to:
- Start async actions in the background at their trajectory locations.
- Pause motion when an AwaitAction is reached and the referenced action is
  still running, then resume once it completes.
- Pause motion when a WaitUntilAction predicate is not yet satisfied, then
  resume once the shared ExecutionState satisfies it.
- Track action results including completion location and timing.
- Handle errors according to configured policy.
"""

from __future__ import annotations

import asyncio
import logging
from dataclasses import dataclass
from datetime import datetime
from typing import TYPE_CHECKING, Any, Awaitable, Callable, Union

from nova.actions.async_action import (
    ActionExecutionContext,
    AsyncAction,
    AsyncActionResult,
    AwaitAction,
    ErrorHandlingMode,
    WaitUntilAction,
)
from nova.actions.execution_state import ExecutionState
from nova.types.state import RobotState

if TYPE_CHECKING:
    from nova.actions.container import ActionLocation

logger = logging.getLogger(__name__)


# Callback types for pause/resume
PauseCallback = Callable[[], Awaitable[None]]
ResumeCallback = Callable[[], Awaitable[None]]

# Callback type for error handling
ErrorCallback = Callable[[AsyncActionResult], Awaitable[None]]

# Union of action types the executor handles
ExecutorAction = Union[AsyncAction, AwaitAction, WaitUntilAction]


@dataclass
class PendingAction:
    """An action waiting to be triggered at a specific location.

    Attributes:
        location: Path parameter where action should trigger.
        action: The action to process.
        triggered: Whether this action has been triggered.
    """

    location: float
    action: ExecutorAction
    triggered: bool = False


@dataclass
class RunningAction:
    """An async action currently being executed in the background.

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
    """Manages execution of actions during trajectory motion.

    The executor monitors robot location during trajectory execution and
    processes three kinds of actions:

    - **AsyncAction**: starts an async callable in the background.
    - **AwaitAction**: checks whether a previously-started AsyncAction has
      completed; if not, pauses robot motion until it does.
    - **WaitUntilAction**: evaluates a predicate on the shared
      ``ExecutionState``; if ``False``, pauses until it becomes ``True``.

    Example:
        ```python
        executor = AsyncActionExecutor(
            motion_group_id="0@ur10",
            executor_actions=combined_actions.get_executor_actions(),
            execution_state=ExecutionState(),
            error_mode=ErrorHandlingMode.COLLECT,
        )

        async for state in motion_group_states:
            await executor.check_and_trigger(
                current_location=state.location,
                current_state=robot_state,
            )

        for result in executor.results:
            print(f"{result.action.action_name}: {result.succeeded}")
        ```
    """

    def __init__(
        self,
        motion_group_id: str,
        executor_actions: list[ActionLocation],
        execution_state: ExecutionState,
        error_mode: ErrorHandlingMode = ErrorHandlingMode.RAISE,
        error_callback: ErrorCallback | None = None,
        pause_callback: PauseCallback | None = None,
        resume_callback: ResumeCallback | None = None,
    ):
        """Initialize the executor.

        Args:
            motion_group_id: Motion group executing the trajectory.
            executor_actions: ActionLocations containing AsyncAction, AwaitAction,
                or WaitUntilAction instances.
            execution_state: Shared per-trajectory state for predicates.
            error_mode: How to handle action execution errors.
            error_callback: Callback for CALLBACK error mode.
            pause_callback: Called when motion must pause.
            resume_callback: Called when motion may resume.

        Raises:
            ValueError: If an AwaitAction references an action_id that has
                no corresponding AsyncAction (dangling await).
        """
        self.motion_group_id = motion_group_id
        self.error_mode = error_mode
        self._error_callback = error_callback
        self._pause_callback = pause_callback
        self._resume_callback = resume_callback
        self._execution_state = execution_state

        # Build pending list from all executor-relevant actions
        self._pending_actions: list[PendingAction] = [
            PendingAction(location=al.path_parameter, action=al.action)
            for al in executor_actions
            if isinstance(al.action, (AsyncAction, AwaitAction, WaitUntilAction))
        ]

        # Sort by location for sequential processing
        self._pending_actions.sort(key=lambda pa: pa.location)

        # Validate: every AwaitAction must reference an existing AsyncAction
        async_ids = {
            pa.action.action_id
            for pa in self._pending_actions
            if isinstance(pa.action, AsyncAction)
        }
        for pa in self._pending_actions:
            if isinstance(pa.action, AwaitAction) and pa.action.action_id not in async_ids:
                raise ValueError(
                    f"AwaitAction references action_id '{pa.action.action_id}' "
                    f"which has no corresponding AsyncAction"
                )

        # Active background tasks keyed by action_id
        self._active_tasks: dict[str, RunningAction] = {}

        # Completed results keyed by action_id (for await lookups)
        self._completed_by_id: dict[str, AsyncActionResult] = {}

        # All completed results in order
        self._results: list[AsyncActionResult] = []

        # Track last known location
        self._last_location: float = 0.0

        # Lock for result collection
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
        return len(self._active_tasks) > 0

    async def check_and_trigger(self, current_location: float, current_state: RobotState) -> bool:
        """Check if any actions should be triggered and process them.

        Called by the movement controller on each state update.

        Args:
            current_location: Current path parameter on trajectory.
            current_state: Current robot state.

        Returns:
            True if motion was paused (blocking await/wait_until triggered).
        """
        self._last_location = current_location
        paused = False

        for pending in self._pending_actions:
            if pending.triggered:
                continue

            if current_location >= pending.location:
                pending.triggered = True

                if isinstance(pending.action, AsyncAction):
                    self._start_async_action(pending.action, current_location, current_state)
                elif isinstance(pending.action, AwaitAction):
                    did_pause = await self._handle_await(
                        pending.action, current_location, current_state
                    )
                    paused = paused or did_pause
                elif isinstance(pending.action, WaitUntilAction):
                    did_pause = await self._handle_wait_until(pending.action, current_location)
                    paused = paused or did_pause

        # Collect any completed background tasks
        await self._collect_completed_tasks()

        return paused

    # -- AsyncAction: start in background ------------------------------------

    def _start_async_action(
        self, action: AsyncAction, trigger_location: float, current_state: RobotState
    ) -> None:
        """Start an async action in the background."""
        logger.info(
            f"Starting async action '{action.action_name}' (id={action.action_id}) "
            f"at location {trigger_location:.3f}"
        )
        context = ActionExecutionContext(
            trigger_location=trigger_location,
            current_state=current_state,
            motion_group_id=self.motion_group_id,
            action_name=action.action_name,
            action_args=action.args,
            action_kwargs=action.kwargs,
            state=self._execution_state,
        )

        async def execute_action() -> Any:
            handler = action.get_handler()
            return await handler(context)

        task = asyncio.create_task(execute_action(), name=f"async-action-{action.action_id}")

        running = RunningAction(
            action=action, task=task, trigger_location=trigger_location, started_at=datetime.now()
        )
        self._active_tasks[action.action_id] = running

    # -- AwaitAction: wait for referenced AsyncAction -------------------------

    async def _handle_await(
        self, await_act: AwaitAction, current_location: float, current_state: RobotState
    ) -> bool:
        """Handle an AwaitAction.  Returns True if motion was paused."""
        action_id = await_act.action_id

        # Already completed?
        if action_id in self._completed_by_id:
            logger.debug(
                f"AwaitAction '{action_id}' at location {current_location:.3f}: "
                "already completed — no pause needed"
            )
            return False

        # Still running?
        running = self._active_tasks.get(action_id)
        if running is None:
            # Should not happen if validation passed, but be safe
            logger.error(f"AwaitAction references unknown action_id '{action_id}'")
            return False

        logger.info(
            f"AwaitAction '{action_id}' at location {current_location:.3f}: "
            "action still running — pausing motion"
        )

        # Pause motion
        if self._pause_callback:
            await self._pause_callback()

        # Wait for the task
        error: Exception | None = None
        return_value: Any = None
        timed_out = False

        try:
            if await_act.timeout is not None:
                return_value = await asyncio.wait_for(running.task, timeout=await_act.timeout)
            else:
                return_value = await running.task
        except asyncio.TimeoutError:
            timed_out = True
            error = asyncio.TimeoutError(
                f"AwaitAction for '{action_id}' timed out after {await_act.timeout}s"
            )
            logger.warning(str(error))
        except Exception as e:
            error = e
            logger.error(f"Async action '{action_id}' failed: {e}", exc_info=True)

        # Record result
        completed_at = datetime.now()
        result = AsyncActionResult(
            action=running.action,
            trigger_location=running.trigger_location,
            started_at=running.started_at,
            completed_at=completed_at,
            completion_location=self._last_location,
            return_value=return_value,
            error=error,
            timed_out=timed_out,
        )
        async with self._results_lock:
            self._results.append(result)
        self._completed_by_id[action_id] = result
        self._active_tasks.pop(action_id, None)

        await self._handle_error(result)

        # Resume motion
        if self._resume_callback:
            await self._resume_callback()

        return True

    # -- WaitUntilAction: wait for predicate ----------------------------------

    async def _handle_wait_until(self, action: WaitUntilAction, current_location: float) -> bool:
        """Handle a WaitUntilAction.  Returns True if motion was paused."""
        # Fast path: predicate already satisfied
        if action.predicate(self._execution_state.snapshot()):
            logger.debug(
                f"WaitUntilAction at location {current_location:.3f}: "
                "predicate already satisfied — no pause"
            )
            return False

        logger.info(
            f"WaitUntilAction at location {current_location:.3f}: "
            "predicate not satisfied — pausing motion"
        )

        if self._pause_callback:
            await self._pause_callback()

        satisfied = await self._execution_state.wait_for(action.predicate, timeout=action.timeout)

        if not satisfied:
            logger.warning(
                f"WaitUntilAction at location {current_location:.3f}: "
                f"timed out after {action.timeout}s"
            )

        if self._resume_callback:
            await self._resume_callback()

        return True

    # -- Background task collection -------------------------------------------

    async def _collect_completed_tasks(self) -> None:
        """Collect results from completed background tasks."""
        completed_ids: list[str] = []

        for action_id, running in self._active_tasks.items():
            if running.task.done():
                completed_ids.append(action_id)

        for action_id in completed_ids:
            running = self._active_tasks.pop(action_id)
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
                timed_out=timed_out,
            )

            async with self._results_lock:
                self._results.append(result)
            self._completed_by_id[running.action.action_id] = result

            await self._handle_error(result)

    # -- Error handling -------------------------------------------------------

    async def _handle_error(self, result: AsyncActionResult) -> None:
        """Handle action error according to error mode."""
        if result.error is None:
            return

        if self.error_mode == ErrorHandlingMode.RAISE:
            from nova.exceptions import AsyncActionError

            raise AsyncActionError(
                action_name=result.action.action_name,
                trigger_location=result.trigger_location,
                completion_location=result.completion_location,
                cause=result.error,
            )
        elif self.error_mode == ErrorHandlingMode.CALLBACK and self._error_callback:
            await self._error_callback(result)
        # COLLECT mode: error is already stored in result

    # -- Lifecycle ------------------------------------------------------------

    async def wait_for_all_actions(self) -> None:
        """Wait for all background actions to complete.

        Should be called at end of trajectory execution to ensure all
        background actions have finished.
        """
        if not self._active_tasks:
            return

        logger.debug(f"Waiting for {len(self._active_tasks)} background actions to complete")

        tasks = [ra.task for ra in self._active_tasks.values()]
        await asyncio.gather(*tasks, return_exceptions=True)
        await self._collect_completed_tasks()

    async def cancel_all_actions(self) -> None:
        """Cancel all running background actions."""
        for running in self._active_tasks.values():
            if not running.task.done():
                running.task.cancel()

        if self._active_tasks:
            await asyncio.gather(
                *[ra.task for ra in self._active_tasks.values()], return_exceptions=True
            )
            self._active_tasks.clear()

    def get_summary(self) -> dict[str, Any]:
        """Get execution summary for logging/debugging."""
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
            "still_running": len(self._active_tasks),
            "total_duration_seconds": total_duration,
        }
