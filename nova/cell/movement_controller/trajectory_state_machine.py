"""State machine for trajectory execution lifecycle.

Provides a reusable :class:`TrajectoryExecutionMachine` that encapsulates the
state handling logic shared across movement controllers (``move_forward``,
``TrajectoryCursor``, …).

The machine processes :class:`~nova.api.models.MotionGroupState` updates and
determines trajectory execution state transitions — including forward/backward
movement, pauses and trajectory completion — in a single, testable place.

State diagram::

    ┌──────┐  start   ┌───────────┐
    │ idle │─────────→│ executing │←───────────────────┐
    └──────┘          └─────┬─────┘                    │
                            │                          │
               ┌────────────┼────────────┐             │
               │            │            │          resume
            ended+ss     ended       paused+ss         │
               │         (no ss)        │              │
               ▼            │           ▼              │
         ┌───────────┐      │     ┌─────────┐          │
         │  ended    │      │     │ paused  │──────────┘
         └───────────┘      │     └─────────┘
               ▲            ▼          ▲
               │      ┌──────────┐     │
               └──ss──│  ending  │     │
                      └──────────┘     │
               ▲                       │
               │      ┌──────────┐     │
               └──ss──│ pausing  │─ss──┘
                      └──────────┘

    ss = standstill

    Any state may transition to ``error`` via :meth:`fail`.

Example::

    machine = TrajectoryExecutionMachine()
    machine.send("start")

    async for state in motion_group_states:
        result = machine.process_motion_state(state)

        if result.location is not None:
            update_location(result.location)

        if machine.is_ended:
            break
        if machine.is_paused:
            handle_pause()
"""

from __future__ import annotations

import logging
from dataclasses import dataclass

from statemachine import State, StateMachine

from nova import api

logger = logging.getLogger(__name__)


# ---------------------------------------------------------------------------
# Result type returned by process_motion_state
# ---------------------------------------------------------------------------


@dataclass(frozen=True)
class StateUpdate:
    """Result of processing a single :class:`~nova.api.models.MotionGroupState`.

    Attributes:
        location: Updated trajectory location (``None`` when no
            :class:`~nova.api.models.TrajectoryDetails` were present).
        has_execute: ``True`` when the ``execute`` field was set on the
            incoming :class:`~nova.api.models.MotionGroupState`.
        state_changed: ``True`` when the machine transitioned to a
            different state during this processing step.
        previous_state_id: Identifier of the state *before* this step.
        current_state_id: Identifier of the state *after* this step.
    """

    location: float | None = None
    has_execute: bool = False
    state_changed: bool = False
    previous_state_id: str = ""
    current_state_id: str = ""

    @property
    def skip(self) -> bool:
        """Convenience — ``True`` when the state carried no useful information."""
        return not self.has_execute and not self.state_changed


# ---------------------------------------------------------------------------
# State machine
# ---------------------------------------------------------------------------

# A frozen sentinel used only to carry the standstill flag into the
# ``cond=`` helpers.  We store it on the model before firing any
# transition so that the condition guards can read it.
_UNSET: float | None = None


class TrajectoryExecutionMachine(StateMachine):
    """Finite-state machine for a single trajectory execution lifecycle.

    **States**

    ============  =============================================================
    ``idle``       No trajectory active — waiting for :meth:`start`.
    ``executing``  Robot is moving (``TrajectoryRunning``).
    ``ending``     ``TrajectoryEnded`` received but robot not yet at standstill.
    ``pausing``    ``TrajectoryPausedByUser`` received, not yet at standstill.
    ``paused``     Robot paused and at standstill — may :meth:`start` again.
    ``ended``      Trajectory finished **and** robot at standstill.
    ``error``      Unrecoverable error — terminal state.
    ============  =============================================================

    **Transitions fired externally**

    * ``start`` — begin (or resume) execution.
    * ``fail``  — signal an error from any non-terminal state.

    All trajectory-state transitions are triggered internally by
    :meth:`process_motion_state`.
    """

    # -- States ---------------------------------------------------------------

    idle = State(initial=True)
    executing = State()
    ending = State()
    pausing = State()
    paused = State()
    ended = State()
    error = State(final=True)

    # -- External commands ----------------------------------------------------

    start = idle.to(executing) | paused.to(executing) | ended.to(executing)

    fail = (
        idle.to(error)
        | executing.to(error)
        | ending.to(error)
        | pausing.to(error)
        | paused.to(error)
    )

    # -- Internal transitions (triggered by process_motion_state) -------------

    _keep_executing = executing.to(executing, internal=True)

    _end_immediately = executing.to(ended)
    _begin_ending = executing.to(ending)

    _pause_immediately = executing.to(paused)
    _begin_pausing = executing.to(pausing)

    _end_after_standstill = ending.to(ended)
    _keep_ending = ending.to(ending, internal=True)

    _pause_after_standstill = pausing.to(paused)
    _keep_pausing = pausing.to(pausing, internal=True)

    # -- Instance state -------------------------------------------------------

    def __init__(self) -> None:
        self.location: float | None = None
        super().__init__()

    # -- Public API -----------------------------------------------------------

    def process_motion_state(self, state: api.models.MotionGroupState) -> StateUpdate:
        """Feed a :class:`~nova.api.models.MotionGroupState` into the machine.

        This is the **main entry point** for movement controllers.  It
        inspects the incoming state, fires the appropriate internal
        transition and returns a :class:`StateUpdate` describing what
        happened.

        Args:
            state: The latest motion-group state from the API stream.

        Returns:
            A :class:`StateUpdate` with location, execute presence and
            transition information.
        """
        previous_state_id: str = self.current_state.id  # type: ignore[union-attr]
        has_execute = state.execute is not None
        location: float | None = None

        if not has_execute:
            # No execute info — skip.  The API guarantees that once execute is
            # set it will remain present in subsequent states, so a bare
            # standstill (without execute) is not a reliable completion signal.
            return StateUpdate(
                has_execute=False,
                state_changed=False,
                previous_state_id=previous_state_id,
                current_state_id=self.current_state.id,  # type: ignore[union-attr]
            )

        # Execute *is* present ------------------------------------------------
        assert state.execute is not None  # mypy
        if isinstance(state.execute.details, api.models.TrajectoryDetails):
            location = state.execute.details.location.root
            self.location = location
            trajectory_state = state.execute.details.state

            if self.current_state == self.executing:
                self._handle_executing(trajectory_state, standstill=state.standstill)

            elif self.current_state == self.ending:
                if state.standstill:
                    self._end_after_standstill()
                else:
                    self._keep_ending()

            elif self.current_state == self.pausing:
                if state.standstill:
                    self._pause_after_standstill()
                else:
                    self._keep_pausing()

        return StateUpdate(
            location=location,
            has_execute=True,
            state_changed=self.current_state.id != previous_state_id,
            previous_state_id=previous_state_id,
            current_state_id=self.current_state.id,  # type: ignore[union-attr]
        )

    # -- Convenience properties -----------------------------------------------

    @property
    def is_idle(self) -> bool:
        return self.current_state == self.idle

    @property
    def is_executing(self) -> bool:
        return self.current_state == self.executing

    @property
    def is_ending(self) -> bool:
        return self.current_state == self.ending

    @property
    def is_pausing(self) -> bool:
        return self.current_state == self.pausing

    @property
    def is_paused(self) -> bool:
        return self.current_state == self.paused

    @property
    def is_ended(self) -> bool:
        return self.current_state == self.ended

    @property
    def is_error(self) -> bool:
        return self.current_state == self.error

    @property
    def is_terminal(self) -> bool:
        """``True`` when in a final state (ended or error)."""
        return self.current_state in (self.ended, self.error)

    @property
    def is_waiting_for_standstill(self) -> bool:
        """``True`` when trajectory ended or paused but robot still decelerating."""
        return self.current_state in (self.ending, self.pausing)

    # -- Logging callbacks (python-statemachine hooks) ------------------------

    def on_enter_executing(self) -> None:
        logger.debug("Trajectory state machine → executing")

    def on_enter_ending(self) -> None:
        logger.debug("Trajectory state machine → ending (waiting for standstill)")

    def on_enter_pausing(self) -> None:
        logger.debug("Trajectory state machine → pausing (waiting for standstill)")

    def on_enter_paused(self) -> None:
        logger.debug("Trajectory state machine → paused")

    def on_enter_ended(self) -> None:
        logger.debug("Trajectory state machine → ended")

    def on_enter_error(self) -> None:
        logger.debug("Trajectory state machine → error")

    # -- Private helpers ------------------------------------------------------

    def _handle_executing(
        self,
        trajectory_state: (
            api.models.TrajectoryRunning
            | api.models.TrajectoryEnded
            | api.models.TrajectoryPausedByUser
            | api.models.TrajectoryPausedOnIO
            | None
        ),
        *,
        standstill: bool,
    ) -> None:
        """Determine the right transition while in ``executing`` state."""
        match trajectory_state:
            case api.models.TrajectoryEnded() | api.models.TrajectoryPausedOnIO():
                if standstill:
                    self._end_immediately()
                else:
                    self._begin_ending()

            case api.models.TrajectoryPausedByUser():
                if standstill:
                    self._pause_immediately()
                else:
                    self._begin_pausing()

            case api.models.TrajectoryRunning():
                self._keep_executing()

            case _:
                # Unknown / None — stay executing
                self._keep_executing()
