"""State machine for trajectory execution lifecycle.

Provides a reusable :class:`TrajectoryExecutionMachine` that encapsulates the
state handling logic shared across movement controllers (``move_forward``,
``TrajectoryCursor``, …).

The machine processes :class:`~nova.api.models.MotionGroupState` updates and
determines trajectory execution state transitions — including forward/backward
movement, pauses and trajectory completion — in a single, testable place.

State diagram (``ss`` = standstill)::

    idle ──start──→ armed ──RUNNING──→ executing
                      │                   │
                      │ parked PAUSED_BY_USER, stale terminal: stay
                      │
    executing ──ended+ss──→ ended        executing ──paused+ss──→ paused
    executing ──ended─────→ ending ──ss──→ ended
    executing ──paused────→ pausing ──ss──→ paused

    ending / pausing ──RUNNING──→ executing      (never settled)
    pausing ──ended──→ ending | ended            (pause overran the end)

    paused(IO)   ──RUNNING──→ executing          (observed resume)
    paused(USER) ──RUNNING──→ error              (no start was issued)
    ended        ──RUNNING──→ error

    paused / ended ──start──→ armed

    Any state may transition to ``error`` via :meth:`fail`.

Three regimes decide how a frame is read:

* **armed** — a start was issued but the robot has not moved yet. The
  controller publishes the *parked* shape (``PAUSED_BY_USER`` at standstill,
  level-based since robotics/wbr!2262) from initialization until motion begins,
  and re-publishes the terminal state a resume was started out of
  (``PAUSED_ON_IO``, ``END_OF_TRAJECTORY``) for a few cycles. Neither is a
  pause or a completion; ``armed`` waits for ``RUNNING``. Only a pause this
  machine was told about (:meth:`request_pause`), a parked frame after the
  robot was seen leaving standstill, or a fresh IO pause are real pauses here.
* **transient** (``ending`` / ``pausing``) — a terminal discriminator was seen
  while the robot was still moving; standstill concludes it. A ``RUNNING``
  frame in a transient state means the controller never settled (a stale
  pause frame at motion start, or a re-armed execution) and returns the
  machine to ``executing``: waiting for standstill would either hang or
  conclude the wrong thing at the next standstill flicker.
* **rest** (``paused`` / ``ended``) — the operation is resolved and standstill
  was confirmed. ``RUNNING`` here without a start from this machine is a
  contradiction: for a user pause or a finished trajectory the machine goes to
  ``error`` with :attr:`failure_reason` set (nobody but this cursor may resume
  or restart), for an IO pause it follows the wire (a controller that resumes
  an IO pause by itself is conceivable, ADR 002).

The consumer guarantees that :meth:`arm` (``start``) is sent *before* the frame
that follows a new movement command is processed; "still at rest when RUNNING
arrives" therefore means "no start was issued".

The ``ss`` edges out of ``ending`` and ``pausing`` fire on any standstill
frame, **with or without an ``execute`` block**: RAE publishes the execute
state level-based, but controllers older than wbr!2262 drop the block the
instant the robot settles, leaving bare standstill frames as the only
completion signal. A bare standstill never concludes anything from ``armed``
or ``executing`` — without a terminal discriminator there is nothing to
conclude. See this package's README for the full wire behaviour.

Example::

    machine = TrajectoryExecutionMachine()
    machine.arm()

    async for state in motion_group_states:
        result = machine.process_motion_state(state)

        if result.location is not None:
            update_location(result.location)

        if machine.is_error:
            raise RuntimeError(machine.failure_reason)
        if machine.is_ended:
            break
        if machine.is_paused:
            handle_pause()
"""

from __future__ import annotations

import logging
from dataclasses import dataclass
from enum import Enum, auto

from statemachine import State, StateMachine

from nova import api

logger = logging.getLogger(__name__)

_TERMINAL_STATES = (
    api.models.TrajectoryEnded,
    api.models.TrajectoryPausedByUser,
    api.models.TrajectoryPausedOnIO,
)


class PauseReason(Enum):
    """Why the controller holds an execution in ``pausing``/``paused``.

    ``USER``: a ``PauseMovementRequest`` (or a stopped execution — the wire has no
    separate kind for it). ``IO``: the ``pause_on_io`` condition attached to the
    start became true. Both are resumed with a new ``StartMovementRequest``; the
    controller never resumes an IO pause by itself, even after the condition
    clears (measured 2026-09-03, see
    docs/architecture/incoming/pause-on-signal-evaluation.md).
    """

    USER = auto()
    IO = auto()


TrajectoryState = (
    api.models.TrajectoryRunning
    | api.models.TrajectoryEnded
    | api.models.TrajectoryPausedByUser
    | api.models.TrajectoryPausedOnIO
    | api.models.TrajectoryWaitForIO
    | None
)


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


class TrajectoryExecutionMachine(StateMachine):
    """Finite-state machine for a single trajectory execution lifecycle.

    **States**

    ============  =============================================================
    ``idle``       No trajectory active — waiting for :meth:`arm`.
    ``armed``      Start issued, robot not yet moving. The controller's parked
                   ``PAUSED_BY_USER`` frames and the re-published terminal
                   state of the stop a resume leaves (same kind, same
                   location) are expected here and change nothing.
    ``executing``  Robot is moving (``TrajectoryRunning``).
    ``ending``     ``TrajectoryEnded`` received but robot not yet at standstill.
    ``pausing``    ``TrajectoryPausedByUser`` or ``TrajectoryPausedOnIO``
                   received, not yet at standstill.
    ``paused``     Robot paused and at standstill — may :meth:`arm` again.
                   :attr:`pause_reason` tells a user pause from an IO pause.
    ``ended``      Trajectory finished **and** robot at standstill.
    ``error``      A frame contradicted the state the machine was in (see
                   :attr:`failure_reason`), or :meth:`fail` was called.
    ============  =============================================================

    **External commands**

    * :meth:`arm` (event ``start``) — begin (or resume) execution. A resume
      ignores frames repeating the terminal state it leaves until a different
      frame arrives, so they are not mistaken for a new completion.
    * :meth:`request_pause` — this machine's owner sent a pause request; the
      next parked-looking frame is a real pause even before the robot moved.
    * ``fail`` — signal an error from any non-terminal state.

    All trajectory-state transitions are triggered internally by
    :meth:`process_motion_state`. The owner must send ``start`` for a new
    movement command *before* processing the frames that follow it; the
    rest-state rules rely on it.
    """

    # -- States ---------------------------------------------------------------

    idle = State(initial=True)
    armed = State()
    executing = State()
    ending = State()
    pausing = State()
    paused = State()
    ended = State()
    error = State(final=True)

    # -- External commands ----------------------------------------------------

    start = idle.to(armed) | paused.to(armed) | ended.to(armed)

    fail = (
        idle.to(error)
        | armed.to(error)
        | executing.to(error)
        | ending.to(error)
        | pausing.to(error)
        | paused.to(error)
        | ended.to(error)
    )

    # -- Internal transitions (triggered by process_motion_state) -------------

    _keep_armed = armed.to(armed, internal=True)
    _begin_executing = armed.to(executing)
    _end_while_armed = armed.to(ended)
    _begin_ending_while_armed = armed.to(ending)
    _pause_while_armed = armed.to(paused)
    _begin_pausing_while_armed = armed.to(pausing)

    _keep_executing = executing.to(executing, internal=True)

    _end_immediately = executing.to(ended)
    _begin_ending = executing.to(ending)

    _pause_immediately = executing.to(paused)
    _begin_pausing = executing.to(pausing)

    _end_after_standstill = ending.to(ended)
    _keep_ending = ending.to(ending, internal=True)
    _resume_from_ending = ending.to(executing)

    _pause_after_standstill = pausing.to(paused)
    _keep_pausing = pausing.to(pausing, internal=True)
    _resume_from_pausing = pausing.to(executing)
    _end_from_pausing = pausing.to(ended)
    _begin_ending_from_pausing = pausing.to(ending)

    # An IO pause the controller resumed without a start from this machine
    # (another client, or a controller that clears it by itself): follow the wire.
    _resume_observed = paused.to(executing)

    # -- Instance state -------------------------------------------------------

    def __init__(self) -> None:
        self.location: float | None = None
        self.pause_reason: PauseReason | None = None
        self.failure_reason: str | None = None
        self.failed_frame: api.models.MotionGroupState | None = None
        self._moved = False
        self._pause_requested = False
        # (state kind, location) of the last terminal frame seen, and the one a
        # resume must ignore while the controller still re-publishes it.
        self._last_terminal: tuple[type, float] | None = None
        self._stale_terminal: tuple[type, float] | None = None
        super().__init__()

    def _active_configuration_id(self) -> str:
        """String id for the active configuration (uses :attr:`StateChart.configuration`)."""
        cfg = self.configuration
        if not cfg:
            return ""
        if len(cfg) == 1:
            return next(iter(cfg)).id
        return ",".join(s.id for s in cfg)

    # -- Public API -----------------------------------------------------------

    def arm(self, *, pause_requested: bool = False) -> None:
        """Begin or resume execution (sends ``start``).

        A start out of ``ended``/``paused`` ignores frames that repeat the terminal
        state it leaves — same kind at the same location — until any different
        frame arrives: level-based controllers keep re-publishing the previous
        stop until they take up the new command, and those frames belong to the
        old operation. A genuine new terminal state is always preceded by a
        different frame (``WAIT_FOR_IO`` or ``RUNNING``), which lifts the filter —
        including a start issued at the very end of the trajectory.

        Args:
            pause_requested: The operation being armed for is itself a pause (the
                owner paused before the machine ever left rest), so the parked
                frame concludes it — see :meth:`request_pause`.
        """
        self.send("start")
        self._pause_requested = pause_requested

    def request_pause(self) -> None:
        """Record that the owner sent a pause request.

        A ``PAUSED_BY_USER`` frame at standstill is then a real pause even while
        ``armed``, where it would otherwise be read as the parked pre-motion shape.
        """
        self._pause_requested = True

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
        previous_state_id: str = self._active_configuration_id()
        has_execute = state.execute is not None
        location: float | None = None

        if not has_execute:
            # No execute details on this frame. Current controllers drop the
            # trajectory `execute` block the instant the robot settles
            # (robotics/wbr MotionPointGenerator removes the provider on
            # END_OF_TRAJECTORY/USER_PAUSED, not only on STOPPED — slated to
            # change with wbr!2262), so a bare standstill can be the only
            # completion signal we ever receive. When we are already waiting
            # for standstill (`ending` / `pausing`), honour it: the
            # discriminator was already seen on the transition into that
            # state. Otherwise there is nothing to conclude from the frame.
            if state.standstill:
                if self.current_state == self.ending:
                    self._end_after_standstill()
                elif self.current_state == self.pausing:
                    self._pause_after_standstill()
            current_id = self._active_configuration_id()
            return StateUpdate(
                has_execute=False,
                state_changed=current_id != previous_state_id,
                previous_state_id=previous_state_id,
                current_state_id=current_id,
            )

        # Execute *is* present ------------------------------------------------
        assert state.execute is not None  # mypy
        if isinstance(state.execute.details, api.models.TrajectoryDetails):
            location = state.execute.details.location
            self.location = location
            trajectory_state = state.execute.details.state

            terminal = (
                (type(trajectory_state), location)
                if isinstance(trajectory_state, _TERMINAL_STATES)
                else None
            )
            if self._stale_terminal is not None:
                # A pause the owner requested is concluded by the very frame a
                # resume would otherwise ignore.
                requested_pause = self._pause_requested and isinstance(
                    trajectory_state, api.models.TrajectoryPausedByUser
                )
                if terminal == self._stale_terminal and not requested_pause:
                    if self.current_state == self.armed:
                        self._keep_armed()
                    current_id = self._active_configuration_id()
                    return StateUpdate(
                        location=location,
                        has_execute=True,
                        state_changed=current_id != previous_state_id,
                        previous_state_id=previous_state_id,
                        current_state_id=current_id,
                    )
                # Anything else proves the controller has moved on from the
                # terminal state the current start was issued out of.
                self._stale_terminal = None
            if terminal is not None:
                self._last_terminal = terminal

            if self.current_state == self.armed:
                self._handle_armed(trajectory_state, state)
            elif self.current_state == self.executing:
                self._handle_executing(trajectory_state, standstill=state.standstill)
            elif self.current_state == self.ending:
                self._handle_ending(trajectory_state, standstill=state.standstill)
            elif self.current_state == self.pausing:
                self._handle_pausing(trajectory_state, standstill=state.standstill)
            elif self.current_state in (self.paused, self.ended):
                self._handle_at_rest(trajectory_state, state)

        current_id = self._active_configuration_id()
        return StateUpdate(
            location=location,
            has_execute=True,
            state_changed=current_id != previous_state_id,
            previous_state_id=previous_state_id,
            current_state_id=current_id,
        )

    # -- Convenience properties -----------------------------------------------

    @property
    def is_idle(self) -> bool:
        return self.current_state == self.idle

    @property
    def is_armed(self) -> bool:
        return self.current_state == self.armed

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
    def is_paused_on_io(self) -> bool:
        """``True`` while the controller holds an IO pause (``pausing`` or ``paused``)."""
        return (
            self.current_state in (self.pausing, self.paused)
            and self.pause_reason is PauseReason.IO
        )

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

    def on_start(self, source: State) -> None:
        # Level-based publishing (robotics/wbr!2262) keeps re-publishing the
        # terminal state of the previous stop until the controller has taken up
        # the new command. Resuming from `ended`/`paused` therefore first sees
        # the old END_OF_TRAJECTORY / PAUSED_* frames again; concluding the new
        # operation from them would report it finished at its start. They are
        # told apart from a genuine new terminal state by identity: same kind at
        # the same location as the state we are leaving.
        self._stale_terminal = self._last_terminal if source in (self.ended, self.paused) else None

    def on_enter_armed(self) -> None:
        self._moved = False
        self._pause_requested = False
        self.pause_reason = None
        logger.debug("Trajectory state machine → armed (waiting for motion)")

    def on_enter_executing(self) -> None:
        self.pause_reason = None
        logger.debug("Trajectory state machine → executing")

    def on_enter_ending(self) -> None:
        logger.debug("Trajectory state machine → ending (waiting for standstill)")

    def on_enter_pausing(self) -> None:
        logger.debug("Trajectory state machine → pausing (waiting for standstill)")

    def on_enter_paused(self) -> None:
        logger.debug("Trajectory state machine → paused (%s)", self.pause_reason)

    def on_enter_ended(self) -> None:
        logger.debug("Trajectory state machine → ended")

    def on_enter_error(self) -> None:
        logger.error("Trajectory state machine → error: %s", self.failure_reason)

    # -- Private helpers ------------------------------------------------------

    def _handle_armed(
        self, trajectory_state: TrajectoryState, state: api.models.MotionGroupState
    ) -> None:
        """Wait for motion; the parked shape is a no-op (stale terminals never get here)."""
        standstill = state.standstill
        match trajectory_state:
            case api.models.TrajectoryRunning():
                self._begin_executing()

            case api.models.TrajectoryEnded():
                if standstill:
                    self._end_while_armed()
                else:
                    self._begin_ending_while_armed()

            case api.models.TrajectoryPausedOnIO():
                # No parked look-alike exists for PAUSED_ON_IO: the controller only
                # reports it after a start armed with pause_on_io, also when the
                # condition already held at the start and the robot never moved.
                self.pause_reason = PauseReason.IO
                if standstill:
                    self._pause_while_armed()
                else:
                    self._begin_pausing_while_armed()

            case api.models.TrajectoryPausedByUser():
                if not standstill:
                    # The robot is leaving standstill; the discriminator flips to
                    # RUNNING one cycle later (measured 2026-09-16).
                    self._moved = True
                    self._keep_armed()
                elif self._moved or self._pause_requested:
                    self.pause_reason = PauseReason.USER
                    self._pause_while_armed()
                else:
                    # The parked pre-motion shape (robotics/wbr!2262).
                    self._keep_armed()

            case _:
                # WAIT_FOR_IO (start_on_io holding the robot) / unknown: not moving yet.
                self._keep_armed()

    def _handle_executing(self, trajectory_state: TrajectoryState, *, standstill: bool) -> None:
        """Determine the right transition while in ``executing`` state."""
        match trajectory_state:
            case api.models.TrajectoryEnded():
                if standstill:
                    self._end_immediately()
                else:
                    self._begin_ending()

            case api.models.TrajectoryPausedByUser() | api.models.TrajectoryPausedOnIO():
                # An IO pause is a suspended execution, not completion: the
                # controller holds it (level-based, re-published every step) until
                # a new start arrives. Treating it as ``ended`` made execute()
                # return mid-trajectory (docs/architecture/adr/002-io-pause-is-resumable.md).
                self.pause_reason = self._reason_of(trajectory_state)
                if standstill:
                    self._pause_immediately()
                else:
                    self._begin_pausing()

            case _:
                # RUNNING / WAIT_FOR_IO / unknown — stay executing. A standstill
                # flag on a RUNNING frame is jitter, not a completion.
                self._keep_executing()

    def _handle_ending(self, trajectory_state: TrajectoryState, *, standstill: bool) -> None:
        match trajectory_state:
            case api.models.TrajectoryRunning():
                logger.warning(
                    "Controller reports RUNNING at location %s while the trajectory was ending; "
                    "resuming execution tracking",
                    self.location,
                )
                self._resume_from_ending()
            case _:
                if standstill:
                    self._end_after_standstill()
                else:
                    self._keep_ending()

    def _handle_pausing(self, trajectory_state: TrajectoryState, *, standstill: bool) -> None:
        match trajectory_state:
            case api.models.TrajectoryRunning():
                # The pause never settled: a stale PAUSED_BY_USER at motion start,
                # or the controller re-armed. Waiting for standstill here would
                # hang, or conclude a "pause" at the next standstill flicker.
                self._resume_from_pausing()
            case api.models.TrajectoryEnded():
                if standstill:
                    self._end_from_pausing()
                else:
                    self._begin_ending_from_pausing()
            case api.models.TrajectoryPausedByUser() | api.models.TrajectoryPausedOnIO():
                self.pause_reason = self._reason_of(trajectory_state)
                if standstill:
                    self._pause_after_standstill()
                else:
                    self._keep_pausing()
            case _:
                if standstill:
                    self._pause_after_standstill()
                else:
                    self._keep_pausing()

    def _handle_at_rest(
        self, trajectory_state: TrajectoryState, state: api.models.MotionGroupState
    ) -> None:
        """``paused`` / ``ended``: the owner sends ``start`` before any frame of a new movement."""
        if self.current_state == self.paused:
            match trajectory_state:
                case api.models.TrajectoryRunning():
                    if self.pause_reason is PauseReason.IO:
                        self._resume_observed()
                    else:
                        self._fail_on_frame(state, trajectory_state)
                case api.models.TrajectoryPausedByUser() | api.models.TrajectoryPausedOnIO():
                    # A user pause turns into an IO pause when the condition
                    # (re)starts being evaluated while the robot stands, e.g.
                    # after the bus-IO service came back (measured).
                    self.pause_reason = self._reason_of(trajectory_state)
                case api.models.TrajectoryEnded():
                    logger.warning(
                        "Controller reports END_OF_TRAJECTORY at location %s while paused",
                        self.location,
                    )
        else:
            match trajectory_state:
                case api.models.TrajectoryRunning():
                    self._fail_on_frame(state, trajectory_state)
                case api.models.TrajectoryPausedOnIO():
                    logger.warning(
                        "Controller reports PAUSED_ON_IO at location %s after the trajectory ended",
                        self.location,
                    )

    def _fail_on_frame(
        self, state: api.models.MotionGroupState, trajectory_state: TrajectoryState
    ) -> None:
        self.failure_reason = (
            f"controller reports {type(trajectory_state).__name__} at location {self.location} "
            f"(standstill={state.standstill}) while the execution is "
            f"'{self._active_configuration_id()}' "
            "and no start was issued"
        )
        self.failed_frame = state
        self.fail()

    @staticmethod
    def _reason_of(trajectory_state: TrajectoryState) -> PauseReason:
        return (
            PauseReason.IO
            if isinstance(trajectory_state, api.models.TrajectoryPausedOnIO)
            else PauseReason.USER
        )
