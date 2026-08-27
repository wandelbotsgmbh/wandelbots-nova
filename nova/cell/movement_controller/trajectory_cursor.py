"""Trajectory cursor for controlling robot movement along a planned trajectory.

This module provides the TrajectoryCursor class and supporting types for interactive
control of robot movement execution. It enables forward/backward movement along a
trajectory, pausing, and stepping through individual actions.

Key concepts:
    - **Location**: A float representing position along the trajectory. Integer values
      correspond to action boundaries (e.g., 0.0 is start of first action, 1.0 is start
      of second action).
    - **Operation**: A single movement command (forward, backward, pause) that can be
      awaited for completion.
    - **Action**: A motion primitive (e.g., ptp, lin) that makes up the trajectory.

Example usage:
    ```python
    # With actions (full functionality)
    cursor = TrajectoryCursor(
        motion_id=motion_id,
        motion_group_state_stream=state_stream,
        joint_trajectory=trajectory,
        actions=actions,
        initial_location=0.0,
    )

    # Without actions (location-based navigation only)
    cursor = TrajectoryCursor(
        motion_id=motion_id,
        motion_group_state_stream=state_stream,
        joint_trajectory=trajectory,
        actions=None,  # or omit entirely
        initial_location=0.0,
    )

    # Both support forward/backward navigation
    result = await cursor.forward()

    # Action stepping works with location boundaries when actions absent
    await cursor.forward_to_next_action()  # Next integer location
    await cursor.backward_to_previous_action()

    # Pause current movement
    cursor.pause()
    ```
"""

import asyncio
import logging
from collections.abc import AsyncIterator as AsyncIteratorABC
from dataclasses import dataclass
from enum import Enum, StrEnum, auto
from math import ceil, floor
from typing import AsyncIterator, Callable, Optional, Union, cast

import pydantic
from blinker import signal

from nova import api
from nova.actions.base import Action
from nova.actions.container import CombinedActions
from nova.cell.movement_controller.trajectory_state_machine import TrajectoryExecutionMachine
from nova.exceptions import ErrorDuringMovement, InitMovementFailed
from nova.types import ExecuteTrajectoryRequestStream, ExecuteTrajectoryResponseStream
from nova.utils import SourceLocation

logger = logging.getLogger(__name__)

_STREAM_STARTUP_TIMEOUT = 5.0

# Bound on states buffered for __aiter__ so an unconsumed cursor cannot grow
# without limit; oldest states are dropped first.
_DEFAULT_MAX_QUEUED_STATES = 1024

MotionGroupStateSource = Union[
    AsyncIterator[api.models.MotionGroupState],
    Callable[[], AsyncIterator[api.models.MotionGroupState]],
]
"""A live motion-group state stream, or a zero-argument factory returning one."""


def _resolve_state_stream(
    source: MotionGroupStateSource,
) -> AsyncIterator[api.models.MotionGroupState]:
    """Return a live state stream, invoking ``source`` when it is a factory."""
    stream = source() if not isinstance(source, AsyncIteratorABC) else source
    return cast(AsyncIterator[api.models.MotionGroupState], stream)


def _frame_shows_motion(state: api.models.MotionGroupState) -> bool:
    """True when this frame is evidence that the robot is actually executing.

    Either the robot is physically moving (``standstill`` false) or the
    controller reports the trajectory RUNNING. The mere presence of an
    ``execute`` block is NOT evidence: with level-based execute state
    (robotics/wbr!2262) the block is published persistently from
    InitializeMovementRequest on, including while parked before the start.
    """
    if not state.standstill:
        return True
    return (
        state.execute is not None
        and isinstance(state.execute.details, api.models.TrajectoryDetails)
        and isinstance(state.execute.details.state, api.models.TrajectoryRunning)
    )


class OperationType(Enum):
    """Types of movement operations that can be performed on a trajectory.

    Attributes:
        FORWARD: Move forward along the trajectory (towards end).
        BACKWARD: Move backward along the trajectory (towards start).
        FORWARD_TO: Move forward to a specific location.
        BACKWARD_TO: Move backward to a specific location.
        FORWARD_TO_NEXT_ACTION: Move forward to the start of the next action.
        BACKWARD_TO_PREVIOUS_ACTION: Move backward to the start of the previous action.
        PAUSE: Pause the current movement.
    """

    FORWARD = auto()
    BACKWARD = auto()
    FORWARD_TO = auto()
    BACKWARD_TO = auto()
    FORWARD_TO_NEXT_ACTION = auto()
    BACKWARD_TO_PREVIOUS_ACTION = auto()
    PAUSE = auto()


class OperationState(Enum):
    """State machine states for a movement operation.

    The state transitions are:
        INITIAL -> COMMANDED -> RUNNING -> COMPLETED
                            \\-> FAILED
                            \\-> CANCELLED

    Attributes:
        INITIAL: Operation created but not yet sent to the controller.
        COMMANDED: Command sent to controller, awaiting execution start.
        RUNNING: Robot is actively moving (standstill is False).
        COMPLETED: Movement finished successfully (standstill is True).
        FAILED: Movement failed (e.g., E-STOP triggered).
        CANCELLED: Operation was cancelled before completion.
    """

    INITIAL = auto()
    COMMANDED = auto()
    RUNNING = auto()
    COMPLETED = auto()
    FAILED = auto()
    CANCELLED = auto()


@dataclass
class OperationResult:
    """Result returned when a movement operation completes.

    Attributes:
        operation_type: The type of operation that was performed.
        target_location: The intended destination location (if specified).
        start_location: The location where the operation started.
        final_location: The actual location where the robot stopped.
        error: Exception if the operation failed, None otherwise.
    """

    operation_type: OperationType
    target_location: Optional[float] = None
    start_location: Optional[float] = None
    final_location: Optional[float] = None
    error: Optional[Exception] = None


# Type alias for expected response types in _response_consumer
ExpectedResponseType = Union[
    type[api.models.StartMovementResponse], type[api.models.PauseMovementResponse]
]


@dataclass
class Operation:
    """Encapsulates all state for a single movement operation.

    This dataclass tracks the complete lifecycle of a movement operation,
    from creation through completion or cancellation.

    Attributes:
        future: Async future that resolves when the operation completes.
        operation_type: The type of movement being performed.
        operation_state: Current state in the operation lifecycle.
        start_location: Trajectory location when the operation began.
        expected_response_type: The API response type expected for this operation.
        target_location: Target location for targeted movements (forward_to, backward_to).
        interrupt_requested: Flag indicating if cancellation was requested.
    """

    future: asyncio.Future[OperationResult]
    operation_type: OperationType
    operation_state: OperationState
    start_location: float
    expected_response_type: ExpectedResponseType
    target_location: Optional[float] = None
    interrupt_requested: bool = False


class OperationHandler:
    """Manages the lifecycle of movement operations for a TrajectoryCursor.

    This class handles state transitions for operations, ensuring proper sequencing
    and completion of movement commands. Only one operation can be active at a time;
    starting a new operation cancels any pending operation.

    The handler tracks operation state through the lifecycle:
    INITIAL -> COMMANDED -> RUNNING -> COMPLETED/FAILED/CANCELLED
    """

    def __init__(self):
        self._operation: Optional[Operation] = None

    def start(
        self,
        operation_type: OperationType,
        *,
        start_location: float,
        expected_response_type: ExpectedResponseType,
        target_location: Optional[float] = None,
    ) -> asyncio.Future[OperationResult]:
        """Start a new movement operation.

        If an operation is already in progress, it will be cancelled before
        starting the new one.

        Args:
            operation_type: The type of movement to perform.
            start_location: Current position on the trajectory.
            expected_response_type: API response type to expect.
            target_location: Target position for targeted movements.

        Returns:
            A Future that resolves with OperationResult when the operation completes.
        """
        if self._operation and not self._operation.future.done():
            self._operation.future.cancel()

        future: asyncio.Future[OperationResult] = asyncio.Future()
        self._operation = Operation(
            future=future,
            operation_type=operation_type,
            operation_state=OperationState.INITIAL,
            start_location=start_location,
            expected_response_type=expected_response_type,
            target_location=target_location,
            interrupt_requested=False,
        )
        return future

    def set_commanded(self):
        """Transition operation state to COMMANDED.

        Called when the movement command has been sent to the controller.
        Idempotent from COMMANDED and RUNNING states to handle race conditions
        between state updates and response processing.

        Raises:
            RuntimeError: If transition from current state is invalid.
        """
        if self._operation is None:
            logger.warning("set_commanded() called with no operation — ignoring")
            return
        if self._operation.operation_state == OperationState.INITIAL:
            self._operation.operation_state = OperationState.COMMANDED
        elif self._operation.operation_state in (OperationState.COMMANDED, OperationState.RUNNING):
            # Already commanded or running — no-op.
            # COMMANDED: can happen if a stale response slips through.
            # RUNNING: can happen if state monitor sets RUNNING before the response arrives.
            pass
        else:
            raise RuntimeError(
                f"Cannot set operation to COMMANDED from state {self._operation.operation_state}"
            )

    def set_running(self):
        """Transition operation state to RUNNING.

        Called when the robot begins moving (standstill becomes False).
        This is idempotent and handles race conditions between state updates.

        Raises:
            RuntimeError: If transition from current state is invalid.
        """
        assert self._operation is not None
        if self._operation.operation_state == OperationState.COMMANDED:
            self._operation.operation_state = OperationState.RUNNING
        elif self._operation.operation_state == OperationState.RUNNING:
            # no-op, already running; idempotent
            pass
        elif self._operation.operation_state == OperationState.INITIAL:
            # no-op, this can happen if we process the motion group state with
            # the execute set before the ExecuteTrajectoryResponse which sets COMMANDED
            pass
        else:
            raise RuntimeError(
                f"Cannot set operation to RUNNING from state {self._operation.operation_state}"
            )

    def complete(self, *, final_location: float, error: Optional[Exception] = None) -> None:
        """Complete the current operation and resolve its future.

        Args:
            final_location: The trajectory location where movement stopped.
            error: Optional exception if the operation failed.
        """
        if not self._operation or self._operation.future.done():
            return

        result = OperationResult(
            operation_type=self._operation.operation_type,
            target_location=self._operation.target_location,
            start_location=self._operation.start_location,
            final_location=final_location,
            error=error,
        )
        if error:
            self._operation.future.set_exception(error)
        else:
            logger.debug(f"Operation completed with result: {result}")
            self._operation.future.set_result(result)
        self._reset()

    def in_progress(self) -> bool:
        """Check if an operation is currently active.

        Returns:
            True if an operation exists and its future is not yet resolved.
        """
        return self._operation is not None and not self._operation.future.done()

    def is_commanded(self) -> bool:
        """Whether the current operation's command has actually been sent.

        An operation still in ``INITIAL`` has had nothing put on the wire, so any
        "movement finished" signal observed in that state cannot belong to it.
        Gating on this — rather than on the acknowledgement having been received —
        keeps the check free of races against a fast state stream.
        """
        return (
            self._operation is not None
            and self._operation.operation_state is not OperationState.INITIAL
        )

    def may_complete_as_paused(self) -> bool:
        """Whether a terminal paused state can belong to the current operation.

        A PAUSE operation is completed by exactly the pause it commanded. Any
        other operation may only be concluded by a pause after it demonstrably
        ran: with level-based execute state (robotics/wbr!2262) the controller
        publishes PAUSED_BY_USER persistently between initialization and the
        actual motion start, and those frames must not resolve a movement
        operation that never moved.
        """
        if self._operation is None:
            return False
        return (
            self._operation.operation_type is OperationType.PAUSE
            or self._operation.operation_state is OperationState.RUNNING
        )

    @property
    def current_operation(self) -> Optional[Operation]:
        """Get the current operation, if any."""
        return self._operation

    def _reset(self):
        """Clear the current operation."""
        self._operation = None


class MovementOption(StrEnum):
    """Available movement options based on current trajectory position.

    Attributes:
        CAN_MOVE_FORWARD: Robot can move forward (not at end of trajectory).
        CAN_MOVE_BACKWARD: Robot can move backward (not at start of trajectory).
    """

    CAN_MOVE_FORWARD = auto()
    CAN_MOVE_BACKWARD = auto()


# Signals emitted during motion events for external observers
motion_started = signal("motion_started")
motion_stopped = signal("motion_stopped")


class MotionEventType(StrEnum):
    """Types of motion events emitted by the cursor.

    Attributes:
        STARTED: Motion has begun or is continuing.
        STOPPED: Motion has stopped.
    """

    STARTED = auto()
    STOPPED = auto()


class MotionEvent(pydantic.BaseModel):
    """Event data emitted when motion state changes.

    Attributes:
        type: Whether motion started or stopped.
        current_location: Current position on the trajectory.
        current_action: The action at the current location (None if no actions available).
        current_action_source: Exact source span to highlight for the current action.
        target_location: The intended destination location.
        target_action: The action at the target location (None if no actions available).
        target_action_source: Exact source span to highlight for the target action.
    """

    type: MotionEventType
    current_location: float
    current_action: Action | None
    current_action_source: SourceLocation | None = None
    target_location: float
    target_action: Action | None
    target_action_source: SourceLocation | None = None


ExecuteTrajectoryRequestCommand = Union[
    api.models.InitializeMovementRequest,
    api.models.StartMovementRequest,
    api.models.PauseMovementRequest,
    api.models.PlaybackSpeedRequest,
]


@dataclass(frozen=True)
class Intent:
    """Captures the user's movement intent without immediately constructing API commands.

    Public methods like ``forward()`` and ``pause()`` write an Intent to a single
    slot.  The ``_request_loop`` reads the slot, builds the concrete API commands
    via :meth:`to_commands`, and yields them.  If two intents arrive before the
    loop wakes, only the latest one is sent — no stale commands ever reach the
    server.
    """

    operation_type: OperationType
    target_location: float | None = None
    playback_speed_in_percent: int | None = None
    start_on_io: api.models.StartOnIO | None = None
    pause_on_io: api.models.PauseOnIO | None = None
    set_outputs: list[api.models.SetIO] | None = None

    def to_commands(self) -> list[ExecuteTrajectoryRequestCommand]:
        """Build the concrete API commands for this intent."""
        commands: list[ExecuteTrajectoryRequestCommand] = []
        if self.playback_speed_in_percent is not None:
            commands.append(
                api.models.PlaybackSpeedRequest(
                    playback_speed_in_percent=self.playback_speed_in_percent
                )
            )
        if self.operation_type is OperationType.PAUSE:
            commands.append(api.models.PauseMovementRequest())
        else:
            direction = (
                api.models.Direction.DIRECTION_FORWARD
                if self.operation_type in (OperationType.FORWARD, OperationType.FORWARD_TO)
                else api.models.Direction.DIRECTION_BACKWARD
            )
            commands.append(
                api.models.StartMovementRequest(
                    direction=direction,
                    target_location=self.target_location,
                    start_on_io=self.start_on_io,
                    pause_on_io=self.pause_on_io,
                    # The server treats every start as an override of the attached
                    # overlay, so the full list must travel with each one; omitting
                    # it on a resume silently clears the remaining outputs.
                    set_outputs=self.set_outputs,
                )
            )
        return commands


@dataclass(frozen=True)
class _QueueSentinel:
    """Marker type used only as a sentinel for queue termination."""


# The single sentinel value used to signal queue termination
_QUEUE_SENTINEL = _QueueSentinel()


def action_index_for_location(location: float, num_actions: int) -> int:
    """Map a trajectory location to a zero-based action index.

    Action ``i`` occupies the segment ``[i, i + 1]``. An integer location ``N``
    is the boundary where action ``N - 1`` *ends*, so it is attributed to the
    action that just finished rather than to the next one. This keeps the action
    that was executed highlighted when the cursor snaps to an action boundary.
    The start of the trajectory (location ``0.0``) maps to the first action, and
    at or beyond the end the index is clamped to the last action.

    Args:
        location: The current trajectory location.
        num_actions: The number of actions in the trajectory (must be >= 1).

    Returns:
        The zero-based index of the action covering ``location``.
    """
    index = ceil(location) - 1
    if index < 0:
        # at the very start of the trajectory the current action is the first one
        return 0
    if index >= num_actions:
        # at the end of the trajectory the current action remains the last one
        return num_actions - 1
    return index


class TrajectoryCursor:
    """Interactive controller for navigating along a planned robot trajectory.

    The TrajectoryCursor provides bidirectional control over trajectory execution,
    allowing forward/backward movement, pausing, and stepping through individual
    actions. It manages the communication with the motion controller via async
    streams and emits events for UI integration.

    The cursor uses a location-based coordinate system where integer values
    represent action boundaries:
        - Location 0.0 = start of first action
        - Location 1.0 = start of second action (end of first)
        - Location N.0 = end of trajectory (for N actions)

    Attributes:
        motion_id: Unique identifier for this motion execution.
        joint_trajectory: The planned joint-space trajectory.
        actions: The sequence of motion actions in the trajectory.

    Example:
        ```python
        cursor = TrajectoryCursor(
            motion_id=motion_id,
            motion_group_state_stream=state_stream,
            joint_trajectory=trajectory,
            actions=actions,
        )

        # Move forward through the trajectory
        await cursor.forward()

        # Or step through action by action
        while cursor.get_movement_options() & {MovementOption.CAN_MOVE_FORWARD}:
            result = await cursor.forward_to_next_action()
            print(f"Completed action at location {result.final_location}")
        ```
    """

    def __init__(
        self,
        motion_id: str,
        motion_group_state_stream: MotionGroupStateSource,
        joint_trajectory: api.models.JointTrajectory | None = None,
        actions: list[Action] | None = None,
        initial_location: float = 0.0,
        detach_on_standstill: bool = False,
        set_outputs: list[api.models.SetIO] | None = None,
        start_on_io: api.models.StartOnIO | None = None,
        pause_on_io: api.models.PauseOnIO | None = None,
        emit_motion_events: bool = True,
        max_queued_states: int = _DEFAULT_MAX_QUEUED_STATES,
        ignore_controller_limits: bool = False,
    ):
        """Initialize a trajectory cursor.

        Args:
            motion_id: Unique identifier for this motion execution.
            motion_group_state_stream: Async stream of motion group state updates,
                or a zero-argument factory returning one.
            joint_trajectory: The planned joint-space trajectory to execute. Optional;
                when omitted the cursor cannot answer location-bounded questions
                (``end_location``, ``get_movement_options``, ``forward_to_next_action``)
                and raises if they are used.
            actions: Actions that make up the trajectory. May contain a mix of
                motion and non-motion actions (e.g. ``WriteAction``); only the
                motion actions drive the cursor's location-to-action mapping.
                Optional for location-based navigation. An empty list is treated
                the same as ``None`` — no action metadata.
            initial_location: Starting position on the trajectory (usually 0.0).
            detach_on_standstill: If True, automatically detach when robot stops.
            set_outputs: IO overlay attached to *every* ``StartMovementRequest`` this
                cursor emits. The server treats each start as an override of the
                previously attached overlay, so a resume that omitted these would
                silently clear them for the rest of the trajectory — hence they are
                held here rather than passed per call.
            start_on_io: Default IO gate applied to every emitted start (same
                override reasoning as ``set_outputs``). Per-call arguments win.
            pause_on_io: Default IO pause condition applied to every emitted start.
            emit_motion_events: When False, no ``motion_started`` signals are emitted.
                Used by one-shot execution, which has no UI to feed.
            max_queued_states: Upper bound on states buffered for ``__aiter__``.
                Oldest states are dropped past this bound so that a cursor whose
                iterator is never consumed cannot grow without limit.
            ignore_controller_limits: Skip the controller's own limit check when
                initializing the movement.
        """
        self.motion_id = motion_id
        self.joint_trajectory = joint_trajectory

        # The planner only assigns trajectory location units to motion actions
        # (see nova.actions.container.CombinedActions.to_motion_command), so the
        # cursor's location-to-action mapping must also be motion-only.
        # Non-motion actions (e.g. WriteAction) are kept on ``_raw_actions`` in
        # their original order so future work can emit events or use them as
        # execution-step boundaries without losing positional information.
        # TODO: surface non-motion actions through the cursor's event API.
        # An empty ``actions`` list carries no mapping information, so it is
        # normalised to None rather than asserting a zero-length trajectory.
        self._raw_actions: tuple[Action, ...] | None = tuple(actions) if actions else None
        # Delegate motion detection to CombinedActions.motions instead of
        # re-implementing it here.
        raw_combined = (
            CombinedActions(items=self._raw_actions) if self._raw_actions is not None else None  # ty: ignore[invalid-argument-type]
        )
        # A list with no motion actions (e.g. only io_write) carries no
        # location-to-action mapping either, so it is normalised to None just
        # like an empty list — otherwise the end-location check below would
        # demand a zero-length trajectory.
        motions = tuple(raw_combined.motions) if raw_combined is not None else ()
        self.actions = CombinedActions(items=motions) if motions else None

        if self.actions is not None and joint_trajectory is not None:
            expected_end_location = len(self.actions)
            actual_end_location = joint_trajectory.locations[-1]
            if abs(actual_end_location - expected_end_location) > 0.01:
                raise ValueError(
                    f"Trajectory end location ({actual_end_location}) does not match "
                    f"number of motion actions ({expected_end_location}). "
                    f"Expected location to be approximately {expected_end_location}.0"
                )
        self._pending_intent: Intent | None = None
        self._intent_event = asyncio.Event()
        self._stop_event = asyncio.Event()
        # Created by cntrl() when an intent is already queued at protocol start
        # (the one-shot adapter shape). While present and unset, the state
        # monitor holds after its first state so a fast state stream cannot be
        # interpreted — or torn down on EOF — before the queued command was
        # dispatched. See _request_loop/_motion_group_state_monitor.
        self._first_dispatch_gate: asyncio.Event | None = None
        self._in_queue: asyncio.Queue[api.models.MotionGroupState | _QueueSentinel] = (
            asyncio.Queue()
        )
        self._max_queued_states = max_queued_states
        self._motion_group_state_stream: AsyncIterator[api.models.MotionGroupState] = (
            _resolve_state_stream(motion_group_state_stream)
        )

        self._set_outputs = set_outputs
        self._start_on_io = start_on_io
        self._pause_on_io = pause_on_io
        self._emit_motion_events = emit_motion_events

        self._ignore_controller_limits = ignore_controller_limits

        self._current_location = initial_location
        # TODO maybe None instead until we have a target?
        self._target_location = self._current_location
        self._detach_on_standstill = detach_on_standstill

        self._state_machine = TrajectoryExecutionMachine()
        self._operation_handler = OperationHandler()

        self._initialize_task = asyncio.create_task(self.ainitialize())

    def _build_intent(
        self,
        operation_type: OperationType,
        *,
        target_location: float | None = None,
        playback_speed_in_percent: int | None = None,
        start_on_io: api.models.StartOnIO | None = None,
        pause_on_io: api.models.PauseOnIO | None = None,
    ) -> Intent:
        """Build an Intent, applying the cursor-level IO defaults.

        ``set_outputs`` is always taken from the cursor: the server treats every
        ``StartMovementRequest`` as an override of the attached overlay, so it must
        travel with each one.
        """
        return Intent(
            operation_type=operation_type,
            target_location=target_location,
            playback_speed_in_percent=playback_speed_in_percent,
            start_on_io=start_on_io if start_on_io is not None else self._start_on_io,
            pause_on_io=pause_on_io if pause_on_io is not None else self._pause_on_io,
            set_outputs=self._set_outputs,
        )

    async def ainitialize(self):
        """Async initialization that emits the initial motion event."""
        await self._send_motion_started()

    async def _send_motion_started(self, *, forward: bool | None = None) -> None:
        """Emit a ``motion_started`` signal unless event emission is disabled."""
        if not self._emit_motion_events:
            return
        await motion_started.send_async(self, event=self._get_motion_event(forward=forward))

    @property
    def end_location(self) -> float:
        """The location value at the end of the trajectory.

        Raises:
            ValueError: If the cursor was created without a ``joint_trajectory``.
        """
        if getattr(self, "joint_trajectory", None) is None:
            raise ValueError(
                "This TrajectoryCursor was created without a joint_trajectory, so the "
                "end of the trajectory is unknown. Pass joint_trajectory= to use "
                "location-bounded operations."
            )
        assert self.joint_trajectory is not None
        return self.joint_trajectory.locations[-1]

    @property
    def current_location(self) -> float:
        """The cursor's current location on the trajectory."""
        return self._current_location

    @property
    def current_action_start(self) -> float:
        """Location where the current action begins (floor of current location)."""
        return float(floor(self._current_location))

    @property
    def current_action_end(self) -> float:
        """Location where the current action ends (ceil of current location)."""
        return float(ceil(self._current_location))

    @property
    def next_action_start(self) -> float:
        """Location where the next action begins."""
        return self.current_action_end

    @property
    def previous_action_start(self) -> float:
        """Location where the previous action begins."""
        return self.current_action_start - 1.0

    @property
    def current_action_index(self) -> int | None:
        """Zero-based index of the action at the current location, or None if no actions."""
        if not self.actions:  # None or empty
            return None
        return action_index_for_location(self._current_location, len(self.actions))

    @property
    def current_action(self) -> Action | None:
        """The action at the current trajectory location, or None if no actions."""
        index = self.current_action_index
        if index is None:
            return None
        assert self.actions is not None
        return self.actions[index]

    @property
    def next_action(self) -> Action | None:
        """The action after the current one, or None if at end or no actions."""
        index = self.current_action_index
        if index is None:
            return None
        next_action_index = index + 1
        assert self.actions is not None
        if next_action_index >= len(self.actions):
            return None
        return self.actions[next_action_index]

    @property
    def previous_action(self) -> Action | None:
        """The action before the current one, or None if at start or no actions."""
        index = self.current_action_index
        if index is None:
            return None
        previous_action_index = index - 1
        if previous_action_index < 0:
            return None
        assert self.actions is not None
        return self.actions[previous_action_index]

    def get_movement_options(self) -> set[MovementOption]:
        """Get the set of currently available movement options.

        Returns:
            Set containing CAN_MOVE_FORWARD if not at end, CAN_MOVE_BACKWARD if not at start.

        Raises:
            ValueError: If the cursor was created without a ``joint_trajectory``,
                since the end of the trajectory is then unknown.
        """
        options: dict[MovementOption, bool] = {
            MovementOption.CAN_MOVE_FORWARD: self._current_location < self.end_location,
            MovementOption.CAN_MOVE_BACKWARD: self._current_location > 0.0,
        }
        return {option for option, available in options.items() if available}

    def _set_intent(self, intent: Intent) -> None:
        """Record the latest user intent and wake the request loop.

        Any previously pending intent that hasn't been consumed yet is silently
        overwritten — only the most recent intent matters.

        Thread-safety: must be called from the cursor's event-loop thread. The
        intent slot + event are read and cleared by ``_request_loop`` without
        a lock; this is safe only because asyncio is single-threaded and there
        is no ``await`` between the read and the ``_intent_event.clear()``.
        Calling this from another thread can lose intents.
        """
        self._pending_intent = intent
        self._intent_event.set()

    def forward(
        self,
        target_location: float | None = None,
        playback_speed_in_percent: int | None = None,
        start_on_io: api.models.StartOnIO | None = None,
        pause_on_io: api.models.PauseOnIO | None = None,
    ) -> asyncio.Future[OperationResult]:
        """Move forward along the trajectory.

        Starts or continues forward movement towards the end of the trajectory,
        or to a specific target location if provided.

        Args:
            target_location: Optional location to stop at. If None, moves to end.
            playback_speed_in_percent: Optional speed override (1-100).
            start_on_io: Optional IO condition to wait for before movement starts.
            pause_on_io: Optional IO condition that pauses movement while it is
                in progress.

        Returns:
            Future that resolves with OperationResult when movement stops.
            If the cursor has already been detached, returns a failed future.
        """
        if self._stop_event.is_set():
            future: asyncio.Future[OperationResult] = asyncio.Future()
            future.set_exception(
                RuntimeError("Cannot move: TrajectoryCursor has already been detached")
            )
            return future

        future = self._start_operation(
            OperationType.FORWARD, expected_response_type=api.models.StartMovementResponse
        )

        if target_location is not None:
            self._target_location = target_location

        self._set_intent(
            self._build_intent(
                OperationType.FORWARD,
                target_location=target_location,
                playback_speed_in_percent=playback_speed_in_percent,
                start_on_io=start_on_io,
                pause_on_io=pause_on_io,
            )
        )
        return future

    def backward(
        self,
        target_location: float | None = None,
        playback_speed_in_percent: int | None = None,
        start_on_io: api.models.StartOnIO | None = None,
        pause_on_io: api.models.PauseOnIO | None = None,
    ) -> asyncio.Future[OperationResult]:
        """Move backward along the trajectory.

        Starts or continues backward movement towards the start of the trajectory,
        or to a specific target location if provided.

        Args:
            target_location: Optional location to stop at. If None, moves to start.
            playback_speed_in_percent: Optional speed override (1-100).
            start_on_io: Optional IO condition to wait for before movement starts.
            pause_on_io: Optional IO condition that pauses movement while it is
                in progress.

        Returns:
            Future that resolves with OperationResult when movement stops.
            If the cursor has already been detached, returns a failed future.
        """
        if self._stop_event.is_set():
            future: asyncio.Future[OperationResult] = asyncio.Future()
            future.set_exception(
                RuntimeError("Cannot move: TrajectoryCursor has already been detached")
            )
            return future

        future = self._start_operation(
            OperationType.BACKWARD, expected_response_type=api.models.StartMovementResponse
        )

        if target_location is not None:
            self._target_location = target_location

        self._set_intent(
            self._build_intent(
                OperationType.BACKWARD,
                target_location=target_location,
                playback_speed_in_percent=playback_speed_in_percent,
                start_on_io=start_on_io,
                pause_on_io=pause_on_io,
            )
        )
        return future

    def forward_to(
        self,
        location: float,
        playback_speed_in_percent: int | None = None,
        start_on_io: api.models.StartOnIO | None = None,
        pause_on_io: api.models.PauseOnIO | None = None,
    ) -> asyncio.Future[OperationResult]:
        """Move forward to a specific location on the trajectory.

        Args:
            location: Target location to move to (must be >= current location).
            playback_speed_in_percent: Optional speed override (1-100).
            start_on_io: Optional IO condition to wait for before movement starts.
            pause_on_io: Optional IO condition that pauses movement while it is
                in progress.

        Returns:
            Future that resolves with OperationResult when target is reached.
            If location is before current position, returns a failed future.
        """
        if location < self._current_location:
            future: asyncio.Future[OperationResult] = asyncio.Future()
            future.set_exception(
                ValueError("Cannot move forward to a location before the current location")
            )
            return future
        self._target_location = location
        return self.forward(
            target_location=location,
            playback_speed_in_percent=playback_speed_in_percent,
            start_on_io=start_on_io,
            pause_on_io=pause_on_io,
        )

    def backward_to(
        self,
        location: float,
        playback_speed_in_percent: int | None = None,
        start_on_io: api.models.StartOnIO | None = None,
        pause_on_io: api.models.PauseOnIO | None = None,
    ) -> asyncio.Future[OperationResult]:
        """Move backward to a specific location on the trajectory.

        Args:
            location: Target location to move to (must be <= current location).
            playback_speed_in_percent: Optional speed override (1-100).
            start_on_io: Optional IO condition to wait for before movement starts.
            pause_on_io: Optional IO condition that pauses movement while it is
                in progress.

        Returns:
            Future that resolves with OperationResult when target is reached.
            If location is after current position, returns a failed future.
        """
        if location > self._current_location:
            future: asyncio.Future[OperationResult] = asyncio.Future()
            future.set_exception(
                ValueError("Cannot move backward to a location after the current location")
            )
            return future
        self._target_location = location
        return self.backward(
            location,
            playback_speed_in_percent=playback_speed_in_percent,
            start_on_io=start_on_io,
            pause_on_io=pause_on_io,
        )

    def forward_to_next_action(
        self,
        playback_speed_in_percent: int | None = None,
        start_on_io: api.models.StartOnIO | None = None,
        pause_on_io: api.models.PauseOnIO | None = None,
    ) -> asyncio.Future[OperationResult]:
        """Move forward to the start of the next action (or next integer location if no actions).

        Useful for stepping through a trajectory one action at a time.
        If already at an action boundary, moves to the next action.
        If at the end of the trajectory, returns immediately with current location.

        Args:
            playback_speed_in_percent: Optional speed override (1-100).
            start_on_io: Optional IO condition to wait for before movement starts.
                Ignored if already at the end of the trajectory (no movement is
                commanded in that case).
            pause_on_io: Optional IO condition that pauses movement while it is
                in progress. Same caveat as ``start_on_io`` applies at the end
                of the trajectory.

        Returns:
            Future that resolves with OperationResult when next action start is reached.
        """
        target_location = self.next_action_start
        if self._current_location == target_location:
            target_location += 1.0
        if target_location > self.end_location:
            # End of trajectory reached - return immediately
            future: asyncio.Future[OperationResult] = asyncio.Future()
            future.set_result(
                OperationResult(
                    final_location=self._current_location,
                    operation_type=OperationType.FORWARD_TO_NEXT_ACTION,
                )
            )
            return future
        return self.forward_to(
            target_location,
            playback_speed_in_percent=playback_speed_in_percent,
            start_on_io=start_on_io,
            pause_on_io=pause_on_io,
        )

    def backward_to_previous_action(
        self,
        playback_speed_in_percent: int | None = None,
        start_on_io: api.models.StartOnIO | None = None,
        pause_on_io: api.models.PauseOnIO | None = None,
    ) -> asyncio.Future[OperationResult]:
        """Move backward to the start of the previous action (or previous integer location if no actions).

        Useful for stepping backward through a trajectory one action at a time.
        If within an action, moves to the start of that action first.
        If at the start of the trajectory, returns immediately with current location.

        Args:
            playback_speed_in_percent: Optional speed override (1-100).
            start_on_io: Optional IO condition to wait for before movement starts.
                Ignored if already at the start of the trajectory (no movement is
                commanded in that case).
            pause_on_io: Optional IO condition that pauses movement while it is
                in progress. Same caveat as ``start_on_io`` applies at the start
                of the trajectory.

        Returns:
            Future that resolves with OperationResult when previous action start is reached.
        """
        target_location = (
            self.previous_action_start
            if self._current_location - self.previous_action_start <= 1.0
            else self.current_action_start
        )
        if target_location >= 0:
            return self.backward_to(
                target_location,
                playback_speed_in_percent=playback_speed_in_percent,
                start_on_io=start_on_io,
                pause_on_io=pause_on_io,
            )
        else:
            # At start of trajectory - return immediately
            future: asyncio.Future[OperationResult] = asyncio.Future()
            future.set_result(
                OperationResult(
                    final_location=self._current_location,
                    operation_type=OperationType.BACKWARD_TO_PREVIOUS_ACTION,
                )
            )
            return future

    def pause(self) -> asyncio.Future[OperationResult] | None:
        """Pause the current movement operation.

        Sends a pause command to stop the robot at its current position.
        Has no effect if no operation is currently in progress.

        Returns:
            Future that resolves when the robot has stopped, or None if no operation is active.
        """
        if not self._is_operation_in_progress():
            return None

        future = self._start_operation(
            OperationType.PAUSE, expected_response_type=api.models.PauseMovementResponse
        )
        self._set_intent(
            Intent(
                operation_type=OperationType.PAUSE
                # A pause carries no overlay: PauseMovementRequest has no such field.
            )
        )
        return future

    def detach(self):
        """Detach from the trajectory, stopping control but not necessarily movement.

        This signals the cursor to stop processing commands and state updates.
        Any in-progress operation future is cancelled so awaiters get CancelledError
        instead of hanging indefinitely.
        Note: This does not guarantee the robot will stop moving immediately.
        """
        if self._operation_handler.in_progress():
            op = self._operation_handler.current_operation
            if op is not None and not op.future.done():
                op.future.cancel()
        self._stop_event.set()
        self._intent_event.set()  # wake _request_loop so it sees the stop
        self._signal_first_dispatch()  # a parked state monitor must not outlive a stop

    def _signal_first_dispatch(self) -> None:
        """Release a state monitor parked on the first-dispatch gate, if any."""
        if self._first_dispatch_gate is not None:
            self._first_dispatch_gate.set()

    def _start_operation(
        self,
        operation_type: OperationType,
        *,
        expected_response_type: ExpectedResponseType,
        target_location: Optional[float] = None,
    ) -> asyncio.Future[OperationResult]:
        """Start a new operation, returning a Future that will be resolved when the operation completes."""
        return self._operation_handler.start(
            operation_type,
            start_location=self._current_location,
            expected_response_type=expected_response_type,
            target_location=target_location,
        )

    def _complete_operation(self, error: Optional[Exception] = None):
        """Complete the current operation with the given status."""
        self._operation_handler.complete(final_location=self._current_location, error=error)

    def _is_operation_in_progress(self) -> bool:
        return self._operation_handler.in_progress()

    async def cntrl(
        self, response_stream: ExecuteTrajectoryResponseStream
    ) -> ExecuteTrajectoryRequestStream:
        """Main control loop that manages bidirectional communication with the motion controller.

        This async generator handles the protocol for trajectory execution:
        1. Initializes movement with the motion controller
        2. Spawns background tasks for state monitoring and response processing
        3. Yields movement commands from the command queue
        4. Cleans up on completion or error

        Args:
            response_stream: Async iterator of responses from the motion controller.

        Yields:
            Movement request commands to send to the motion controller.

        Raises:
            RuntimeError: If state monitor fails to start within timeout.
        """
        await self._initialize_task

        self._response_stream = response_stream
        async for request in init_movement_gen(
            self.motion_id,
            response_stream,
            self._current_location,
            ignore_controller_limits=self._ignore_controller_limits,
        ):
            yield request

        # An intent queued before the protocol started (a cursor commanded ahead
        # of cntrl, e.g. by the move_forward adapter) must reach the wire before
        # trajectory progress is interpreted: scheduling could otherwise let the
        # state monitor consume a fast stream — and tear the cursor down on its
        # end — before _request_loop has had its first turn, failing the
        # operation without ever sending the command.
        if self._pending_intent is not None:
            self._first_dispatch_gate = asyncio.Event()

        motion_group_state_monitor_ready_event = asyncio.Event()
        response_consumer_ready_event = asyncio.Event()
        try:
            async with asyncio.TaskGroup() as tg:
                motion_group_state_monitor_task = tg.create_task(
                    self._motion_group_state_monitor(
                        ready_event=motion_group_state_monitor_ready_event
                    ),
                    name="motion_group_state_monitor",
                )
                response_consumer_task = tg.create_task(
                    self._response_consumer(ready_event=response_consumer_ready_event),
                    name="response_consumer",
                )
                motion_event_updater_task = tg.create_task(
                    self._motion_event_updater(), name="motion_event_updater"
                )
                # The timeout handling here is very defensive programming to avoid silent hangs
                # in case the connection to the API is lost or similar issues occur.
                # It might be overkill but is useful during development and debugging.
                try:
                    await asyncio.wait_for(
                        motion_group_state_monitor_ready_event.wait(),
                        timeout=_STREAM_STARTUP_TIMEOUT,
                    )
                except asyncio.TimeoutError:
                    logger.error(
                        "TrajectoryCursor motion group state monitor failed to start in time"
                    )
                    motion_group_state_monitor_task.cancel()
                    raise RuntimeError("State monitor failed to start in time")

                await response_consumer_ready_event.wait()

                async for request in self._request_loop():
                    yield request

                motion_event_updater_task.cancel()
                response_consumer_task.cancel()
                motion_group_state_monitor_task.cancel()
        except BaseExceptionGroup as eg:
            logger.exception(eg)
            # A TaskGroup wraps everything, but callers of the movement-controller
            # protocol expect the underlying error (e.g. ErrorDuringMovement), not an
            # exception group. Unwrap when the group carries exactly one exception.
            if len(eg.exceptions) == 1 and isinstance(eg.exceptions[0], Exception):
                raise eg.exceptions[0] from eg
            raise
        except asyncio.CancelledError:
            logger.debug("TrajectoryCursor cntrl was cancelled during cleanup of internal tasks")
            raise

    async def _request_loop(self) -> ExecuteTrajectoryRequestStream:
        while True:
            # Wait for either a new intent or a stop signal.
            await self._intent_event.wait()

            # A stop always wins over a queued intent: once the cursor is stopping,
            # commanding movement would move the robot after the caller's operation
            # has already been cancelled (and, on teardown, with nothing left to
            # monitor it). The dropped intent is not silently lost — its operation
            # is never marked commanded, so it is failed rather than reported as a
            # successful traversal.
            if self._stop_event.is_set():
                self._signal_first_dispatch()
                break

            # Consume the pending intent atomically.
            intent = self._pending_intent
            self._pending_intent = None
            self._intent_event.clear()

            if intent is None:
                self._signal_first_dispatch()
                continue

            commands = intent.to_commands()
            for command in commands:
                # Re-check the stop on every command, not just once per intent.
                # `yield` suspends until the consumer pulls again — a network send —
                # so a detach can land between two commands of the same intent.
                # `to_commands()` is multi-command whenever a playback speed is set,
                # and without this the trailing movement command would still reach
                # the controller after the caller's operation was cancelled.
                if self._stop_event.is_set():
                    return

                logger.debug(f"Processing command: {command}")

                # A stop that lands while this intent is mid-dispatch (e.g. between
                # a PlaybackSpeedRequest and its StartMovementRequest) must suppress
                # the remaining commands for the same reason a queued intent is
                # dropped above: nothing is left to monitor the movement. A speed
                # setting without its movement command is harmless; the reverse is
                # not.
                if self._stop_event.is_set():
                    self._signal_first_dispatch()
                    return

                if isinstance(
                    command, (api.models.StartMovementRequest, api.models.PauseMovementRequest)
                ):
                    # Record the command before handing it over: `yield` suspends
                    # until the consumer pulls again, and completion must be gated on
                    # having commanded the operation rather than on ack timing, which
                    # can lose a race against a fast state stream.
                    self._operation_handler.set_commanded()
                    # Release the state monitor only for the movement command itself,
                    # not for a leading PlaybackSpeedRequest: setting the event only
                    # schedules its waiter, and this task keeps running until the
                    # yield below has handed the command to the consumer — so the
                    # monitor can only resume once the movement command is actually
                    # on its way out.
                    self._signal_first_dispatch()

                yield command

                if isinstance(command, api.models.StartMovementRequest):
                    match command.direction:
                        case api.models.Direction.DIRECTION_FORWARD:
                            await self._send_motion_started(forward=True)
                        case api.models.Direction.DIRECTION_BACKWARD:
                            await self._send_motion_started(forward=False)

    async def _motion_group_state_monitor(self, ready_event: asyncio.Event):
        """Monitor motion group state and update operation status accordingly.

        Uses a :class:`TrajectoryExecutionMachine` to track trajectory lifecycle
        and determine when operations complete.

        Args:
            ready_event: Event to signal when the monitor is ready to receive states.
        """
        logger.debug("Starting state monitor for trajectory cursor")
        try:
            async for motion_group_state in self._held_at_first_dispatch(
                self._motion_group_state_stream
            ):
                ready_event.set()

                # Ensure the state machine is in executing state when an operation
                # is active.  This replaces the old manual kick in _start_operation()
                # and correctly handles resuming after paused/ended states.
                if (
                    self._operation_handler.in_progress()
                    and not self._state_machine.is_executing
                    and not self._state_machine.is_ending
                    and not self._state_machine.is_pausing
                ):
                    self._state_machine.send("start")

                # Tee every state to consumers of __aiter__ regardless of whether an
                # operation is active: observers (guards, overlays, UIs) need states
                # from before movement starts, not only once it is under way.
                result = self._state_machine.process_motion_state(motion_group_state)
                if result.has_execute:
                    self._enqueue_state(motion_group_state)
                    if result.location is not None:
                        self._current_location = result.location

                current_op = self._operation_handler.current_operation
                if current_op is None or current_op.future.done():
                    continue

                if result.skip and not _frame_shows_motion(motion_group_state):
                    continue

                if _frame_shows_motion(motion_group_state):
                    self._operation_handler.set_running()  # idempotent

                if self._state_machine.is_ended or self._state_machine.is_paused:
                    # Only an operation the controller actually acknowledged can be
                    # completed by a terminal state. Without this guard the cursor
                    # reports a successful traversal for movement it never commanded
                    # (e.g. when the start command was never sent).
                    if not self._operation_handler.is_commanded():
                        logger.debug(
                            "Terminal trajectory state observed while the current operation "
                            "was never commanded — not completing it."
                        )
                        continue
                    # A paused state can only conclude a pause operation, or a
                    # movement operation that was seen running. Level-based
                    # execute state (robotics/wbr!2262) publishes PAUSED_BY_USER
                    # persistently between initialize and motion start; those
                    # frames must not resolve a movement that never moved.
                    if self._state_machine.is_paused and not (
                        self._operation_handler.may_complete_as_paused()
                    ):
                        logger.debug(
                            "Paused trajectory state observed before the current operation "
                            "showed any motion — not completing it."
                        )
                        continue
                    self._complete_operation()
                    if self._detach_on_standstill and self._state_machine.is_ended:
                        logger.debug("Detaching on standstill")
                        break

        except asyncio.CancelledError:
            logger.debug("TrajectoryCursor motion group state monitor was cancelled")
            raise
        finally:
            # Fail, rather than silently abandon, an operation that can no longer
            # complete because the state stream is gone.
            if self._operation_handler.in_progress():
                self._complete_operation(
                    error=ErrorDuringMovement(
                        "Motion group state stream ended before the movement completed"
                    )
                )
            # stop the request loop
            self.detach()
            # stop the cursor iterator (TODO is this the right place?)
            self._in_queue.put_nowait(_QUEUE_SENTINEL)

    async def _held_at_first_dispatch(
        self, stream: AsyncIterator[api.models.MotionGroupState]
    ) -> AsyncIterator[api.models.MotionGroupState]:
        """Pass states through, holding between states until the first dispatch.

        With an intent queued before ``cntrl`` started (``_first_dispatch_gate``
        set up by ``cntrl``), the monitor may process the first state — that is
        the liveness handshake ``ready_event`` needs — but must not pull further
        states, or reach the stream's end, before ``_request_loop`` has sent the
        queued command: trajectory progress would be attributed to (or teardown
        would fail) an operation that never went out. Once the gate opens the
        wrapper is transparent. Without a gate (interactive use) it passes
        everything straight through.
        """
        async for motion_group_state in stream:
            yield motion_group_state
            if self._first_dispatch_gate is not None:
                await self._first_dispatch_gate.wait()

    def _enqueue_state(self, motion_group_state: api.models.MotionGroupState) -> None:
        """Buffer a state for ``__aiter__``, dropping the oldest past the bound."""
        if self._in_queue.qsize() >= self._max_queued_states:
            try:
                self._in_queue.get_nowait()
                self._in_queue.task_done()
            except asyncio.QueueEmpty:  # pragma: no cover - racy drain
                pass
        self._in_queue.put_nowait(motion_group_state)

    async def _response_consumer(self, ready_event: asyncio.Event):
        """Process responses from the motion controller and update operation state.

        Handles response messages including movement confirmations, errors, and
        playback speed acknowledgments.

        Args:
            ready_event: Event to signal when the consumer is ready.
        """
        logger.debug("Starting response consumer for trajectory cursor")
        ready_event.set()
        try:
            async for response in self._response_stream:
                logger.debug(f"Received response: {response}")

                # If no operation is in progress, log and skip
                if not self._is_operation_in_progress():
                    logger.debug(
                        f"Response received with no operation in progress: {type(response).__name__} — skipping"
                    )
                    continue

                current_op = self._operation_handler.current_operation
                assert current_op is not None

                match response:
                    case api.models.PlaybackSpeedResponse():
                        pass  # no-op for now
                    case api.models.MovementErrorResponse():
                        error = ErrorDuringMovement(
                            f"Error occurred during trajectory execution: {response.message}"
                        )
                        # Fail the operation with the controller's own message
                        # *before* raising. Raising cancels the state monitor via
                        # the TaskGroup, and its `finally` would otherwise complete
                        # the operation first with a generic "stream ended" error,
                        # leaving a caller awaiting forward()/pause() without the
                        # actual reason. complete() is a no-op once the future is
                        # resolved, so this wins the race by running first.
                        self._complete_operation(error=error)
                        raise error
                    case api.models.StartMovementResponse() | api.models.PauseMovementResponse():
                        # No per-command correlation exists on the wire, but
                        # mis-attribution is harmless here:
                        #   - the type filter rejects a stale Start ack while
                        #     the current op is a PAUSE (and vice versa);
                        #   - `set_commanded()` is idempotent, so the stale
                        #     ack and the real ack collapse to a single
                        #     INITIAL→COMMANDED transition on the current op;
                        #   - the actual RUNNING/COMPLETED lifecycle is driven
                        #     by the motion-state monitor, not by these acks.
                        # The server's 1:1 FIFO guarantee (verified by the
                        # cursor API behavior tests) ensures the current op's
                        # own ack always arrives.
                        if isinstance(response, current_op.expected_response_type):
                            self._operation_handler.set_commanded()
                    case _:
                        raise RuntimeError(
                            f"Unexpected response in trajectory cursor response consumer: {type(response)}, "
                            f"expected {current_op.expected_response_type.__name__}"
                        )
        except asyncio.CancelledError:
            logger.debug("TrajectoryCursor response consumer was cancelled")
            raise

    async def _motion_event_updater(self, interval=0.2):
        """Periodically emit motion events during active movement.

        Args:
            interval: Time in seconds between event emissions (default 0.2s).
        """
        if not self._emit_motion_events:
            return
        while True:
            current_op = self._operation_handler.current_operation
            op_type = current_op.operation_type if current_op else None
            match op_type:
                case OperationType.FORWARD | OperationType.FORWARD_TO:
                    await self._send_motion_started(forward=True)
                case OperationType.BACKWARD | OperationType.BACKWARD_TO:
                    await self._send_motion_started(forward=False)
                case _:
                    pass
            await asyncio.sleep(interval)

    def _action_at_location(self, location: float) -> Action | None:
        """The action covering ``location`` using the cursor's boundary attribution."""
        if not self.actions:  # None or empty
            return None
        index = action_index_for_location(location, len(self.actions))
        return self.actions[index]

    def _get_motion_event(self, *, forward: bool | None = None) -> MotionEvent:
        """Create a MotionEvent for the current cursor state.

        The highlighted (``current_*``) action is the *last visited* action: the
        action at the integer boundary the cursor most recently reached in its
        direction of travel. The ``target_*`` action is the action at the
        boundary it is heading toward. For example, moving forward at location
        ``1.5`` highlights the action at location ``1.0`` and targets the action
        ending at location ``2.0``; moving backward at ``1.5`` highlights the
        action at ``2.0`` and targets ``1.0``.

        Args:
            forward: Direction of travel. ``True`` for forward, ``False`` for
                backward, ``None`` for a standstill/initial event (highlight and
                target the action at the current location).
        """
        if forward is None:
            source_location = self._current_location
            target_location = self._current_location
        elif forward:
            source_location = self.current_action_start
            target_location = self.current_action_start + 1.0
        else:
            source_location = self.current_action_end
            target_location = self.current_action_end - 1.0

        current_action = self._action_at_location(source_location)
        target_action = self._action_at_location(target_location)
        return MotionEvent(
            type=MotionEventType.STARTED,
            current_location=self._current_location,
            current_action=current_action,
            current_action_source=(
                current_action.source_location if current_action is not None else None
            ),
            target_location=target_location,
            target_action=target_action,
            target_action_source=(
                target_action.source_location if target_action is not None else None
            ),
        )

    def __aiter__(self) -> AsyncIterator[api.models.MotionGroupState]:
        """Return self as an async iterator for motion group states."""
        return self

    async def __anext__(self) -> api.models.MotionGroupState:
        """Yield the next motion group state from the internal queue.

        Raises:
            StopAsyncIteration: When the cursor has been detached.
        """
        value = await self._in_queue.get()
        self._in_queue.task_done()
        if isinstance(value, _QueueSentinel):
            raise StopAsyncIteration
        return value


async def init_movement_gen(
    motion_id, response_stream, initial_location, ignore_controller_limits: bool = False
) -> ExecuteTrajectoryRequestStream:
    """Initialize movement on a trajectory with the motion controller.

    This async generator handles the initialization handshake:
    1. Sends an InitializeMovementRequest with the trajectory ID and start location
    2. Waits for and validates the InitializeMovementResponse

    Args:
        motion_id: Unique identifier for the trajectory to execute.
        response_stream: Async iterator of responses from the motion controller.
        initial_location: Starting position on the trajectory.
        ignore_controller_limits: Skip the controller's own limit check.

    Yields:
        The initialization request to send to the motion controller.

    Raises:
        InitMovementFailed: If the motion controller rejects the initialization.
    """
    trajectory_id = api.models.TrajectoryId(id=motion_id)
    init_request = api.models.InitializeMovementRequest(
        trajectory=trajectory_id,
        initial_location=initial_location,
        ignore_controller_limits=ignore_controller_limits,
    )
    yield init_request

    execute_trajectory_response = await anext(response_stream)
    initialize_movement_response = execute_trajectory_response
    assert isinstance(initialize_movement_response, api.models.InitializeMovementResponse)
    # TODO this should actually check for None but currently the API seems to return an empty string instead
    # create issue with the API to fix this
    if initialize_movement_response.message or initialize_movement_response.add_trajectory_error:
        raise InitMovementFailed(initialize_movement_response)
