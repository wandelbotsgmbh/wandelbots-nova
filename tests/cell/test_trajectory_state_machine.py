"""Tests for the TrajectoryExecutionMachine state machine.

These tests verify the state machine models the trajectory execution lifecycle
correctly: the armed phase between a start and the first motion, forward and
backward movement, user and IO pauses, standstill detection, multi-phase
completion (TrajectoryEnded followed by standstill), and the contradictions
that must fail instead of drifting into a state that can never complete.
"""

import json
from datetime import datetime, timezone
from pathlib import Path

import pytest

from nova import api
from nova.cell.movement_controller.trajectory_state_machine import (
    PauseReason,
    TrajectoryExecutionMachine,
)

# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------


def _make_motion_group_state(
    standstill: bool, execute: api.models.Execute | None = None
) -> api.models.MotionGroupState:
    """Create a MotionGroupState with the given standstill and execute fields."""
    return api.models.MotionGroupState(
        timestamp=datetime.now(timezone.utc),
        sequence_number=1,
        description_revision=0,
        motion_group="mg-0",
        controller="ctrl-0",
        joint_position=[0.0] * 6,
        joint_limit_reached=api.models.MotionGroupStateJointLimitReached(limit_reached=[False] * 6),
        standstill=standstill,
        execute=execute,
    )


def _make_execute(
    trajectory_state: (
        api.models.TrajectoryRunning
        | api.models.TrajectoryEnded
        | api.models.TrajectoryPausedByUser
        | api.models.TrajectoryPausedOnIO
        | api.models.TrajectoryWaitForIO
    ),
    location: float = 1.0,
) -> api.models.Execute:
    """Create an Execute with TrajectoryDetails."""
    return api.models.Execute(
        joint_position=[0.0] * 6,
        details=api.models.TrajectoryDetails(
            trajectory="traj-123", location=location, state=trajectory_state
        ),
    )


def _running(location: float = 1.0, *, standstill: bool = False) -> api.models.MotionGroupState:
    return _make_motion_group_state(
        standstill, _make_execute(api.models.TrajectoryRunning(time_to_end=1000), location)
    )


def _ended(location: float = 2.0, *, standstill: bool = True) -> api.models.MotionGroupState:
    return _make_motion_group_state(
        standstill, _make_execute(api.models.TrajectoryEnded(), location)
    )


def _paused_by_user(
    location: float = 1.0, *, standstill: bool = True
) -> api.models.MotionGroupState:
    return _make_motion_group_state(
        standstill, _make_execute(api.models.TrajectoryPausedByUser(), location)
    )


def _paused_on_io(location: float = 1.0, *, standstill: bool = True) -> api.models.MotionGroupState:
    return _make_motion_group_state(
        standstill, _make_execute(api.models.TrajectoryPausedOnIO(), location)
    )


def _wait_for_io(location: float = 0.0) -> api.models.MotionGroupState:
    return _make_motion_group_state(True, _make_execute(api.models.TrajectoryWaitForIO(), location))


def _bare(*, standstill: bool = True) -> api.models.MotionGroupState:
    return _make_motion_group_state(standstill)


def _armed() -> TrajectoryExecutionMachine:
    machine = TrajectoryExecutionMachine()
    machine.arm()
    assert machine.is_armed
    return machine


def _executing() -> TrajectoryExecutionMachine:
    """A machine that has seen its first RUNNING frame."""
    machine = _armed()
    machine.process_motion_state(_running(0.5))
    assert machine.is_executing
    return machine


# ---------------------------------------------------------------------------
# State machine lifecycle tests
# ---------------------------------------------------------------------------


class TestStateMachineLifecycle:
    """Basic lifecycle: idle → start → armed → executing → completed."""

    def test_initial_state_is_idle(self):
        machine = TrajectoryExecutionMachine()
        assert machine.is_idle

    def test_start_transitions_to_armed(self):
        machine = TrajectoryExecutionMachine()
        machine.send("start")
        assert machine.is_armed
        assert not machine.is_executing

    def test_first_running_frame_transitions_to_executing(self):
        machine = _armed()
        result = machine.process_motion_state(_running(0.0))
        assert machine.is_executing
        assert result.state_changed
        assert result.previous_state_id == "armed"
        assert result.current_state_id == "executing"

    def test_ended_is_terminal(self):
        machine = _executing()
        machine.process_motion_state(_ended())
        assert machine.is_ended
        assert machine.is_terminal

    def test_error_is_terminal(self):
        machine = TrajectoryExecutionMachine()
        machine.send("fail")
        assert machine.is_error
        assert machine.is_terminal


# ---------------------------------------------------------------------------
# Armed: start issued, robot not yet moving
# ---------------------------------------------------------------------------


class TestArmed:
    """Between a start and the first motion the controller publishes the parked
    ``PAUSED_BY_USER`` at standstill (level-based, robotics/wbr!2262). That frame,
    and the re-published terminal state of a resume, must not conclude anything."""

    def test_parked_frames_keep_the_machine_armed(self):
        machine = _armed()
        for _ in range(3):
            result = machine.process_motion_state(_paused_by_user(0.0))
            assert machine.is_armed
            assert not result.state_changed
            assert result.has_execute
        assert machine.pause_reason is None

    def test_standstill_dropping_before_the_discriminator_is_motion_starting(self):
        """Measured 2026-09-16: `standstill` flips to False one control cycle
        before the execute state flips from PAUSED_BY_USER to RUNNING."""
        machine = _armed()
        machine.process_motion_state(_paused_by_user(0.0))
        machine.process_motion_state(_paused_by_user(0.0, standstill=False))
        machine.process_motion_state(_paused_by_user(0.0, standstill=False))
        assert machine.is_armed
        machine.process_motion_state(_running(0.0))
        assert machine.is_executing

    def test_parked_frame_after_the_robot_moved_is_a_real_pause(self):
        """An external pause landing inside the start lag: the robot left
        standstill and is parked again without a RUNNING frame in between."""
        machine = _armed()
        machine.process_motion_state(_paused_by_user(0.0, standstill=False))
        result = machine.process_motion_state(_paused_by_user(0.01))
        assert machine.is_paused
        assert machine.pause_reason is PauseReason.USER
        assert result.state_changed

    def test_requested_pause_concludes_on_the_parked_frame(self):
        """A pause the owner asked for before the robot moved is the same frame as
        the parked shape; request_pause() tells them apart."""
        machine = _armed()
        machine.process_motion_state(_paused_by_user(0.0))
        assert machine.is_armed
        machine.request_pause()
        machine.process_motion_state(_paused_by_user(0.0))
        assert machine.is_paused
        assert machine.pause_reason is PauseReason.USER

    def test_wait_for_io_keeps_the_machine_armed(self):
        machine = _armed()
        machine.process_motion_state(_wait_for_io())
        assert machine.is_armed
        machine.process_motion_state(_running(0.0))
        assert machine.is_executing

    def test_bare_standstill_keeps_the_machine_armed(self):
        machine = _armed()
        result = machine.process_motion_state(_bare())
        assert machine.is_armed
        assert result.skip

    def test_ended_at_standstill_completes_without_motion(self):
        """A zero-length or one-cycle trajectory may never show ¬standstill;
        END is then the only completion signal and must count."""
        machine = _armed()
        machine.process_motion_state(_ended(0.0))
        assert machine.is_ended

    def test_ended_while_moving_goes_to_ending(self):
        machine = _armed()
        machine.process_motion_state(_ended(0.5, standstill=False))
        assert machine.is_ending
        machine.process_motion_state(_bare())
        assert machine.is_ended

    def test_io_pause_before_any_motion_is_a_real_pause(self):
        """No parked look-alike exists for PAUSED_ON_IO: the controller reports it
        only after a start armed with pause_on_io, also when the condition already
        held at the start (measured)."""
        machine = _armed()
        machine.process_motion_state(_paused_by_user(0.0))
        machine.process_motion_state(_paused_on_io(0.0))
        assert machine.is_paused
        assert machine.pause_reason is PauseReason.IO
        assert machine.is_paused_on_io

    def test_io_pause_while_moving_goes_to_pausing(self):
        machine = _armed()
        machine.process_motion_state(_paused_on_io(0.1, standstill=False))
        assert machine.is_pausing
        assert machine.is_paused_on_io

    def test_stale_io_pause_after_a_resume_is_ignored(self):
        """After a resume the controller re-publishes PAUSED_ON_IO for a few cycles
        before RUNNING appears (measured); those frames must not re-pause."""
        machine = _executing()
        machine.process_motion_state(_paused_on_io(1.0))
        assert machine.is_paused_on_io

        machine.arm(stale_terminal=api.models.TrajectoryPausedOnIO)
        for _ in range(3):
            result = machine.process_motion_state(_paused_on_io(1.0))
            assert machine.is_armed
            assert not result.state_changed
        machine.process_motion_state(_running(1.5))
        assert machine.is_executing

    def test_stale_end_after_an_intermediate_stop_is_ignored(self):
        machine = _executing()
        machine.process_motion_state(_ended(1.0))
        assert machine.is_ended

        machine.arm(stale_terminal=api.models.TrajectoryEnded)
        machine.process_motion_state(_ended(1.0))
        machine.process_motion_state(_ended(1.0))
        assert machine.is_armed
        machine.process_motion_state(_running(1.5))
        assert machine.is_executing
        machine.process_motion_state(_ended(3.0))
        assert machine.is_ended

    def test_any_other_discriminator_clears_the_stale_marker(self):
        machine = _executing()
        machine.process_motion_state(_ended(1.0))
        machine.arm(stale_terminal=api.models.TrajectoryEnded)
        machine.process_motion_state(_ended(1.0))
        assert machine.is_armed
        # The controller moved on (here: back to the parked shape); a later END
        # is genuine again.
        machine.process_motion_state(_paused_by_user(1.0))
        assert machine.is_armed
        machine.process_motion_state(_ended(1.0))
        assert machine.is_ended

    def test_arm_resets_the_pause_request_and_motion_memory(self):
        machine = _armed()
        machine.request_pause()
        machine.process_motion_state(_paused_by_user(0.0, standstill=False))
        machine.process_motion_state(_paused_by_user(0.0))
        assert machine.is_paused

        machine.arm()
        assert machine.is_armed
        assert machine.pause_reason is None
        machine.process_motion_state(_paused_by_user(0.0))
        assert machine.is_armed, "a resume must not inherit the previous pause request"


# ---------------------------------------------------------------------------
# TrajectoryEnded handling
# ---------------------------------------------------------------------------


class TestTrajectoryEnded:
    """TrajectoryEnded with and without standstill."""

    def test_ended_with_standstill_completes_immediately(self):
        machine = _executing()
        result = machine.process_motion_state(_ended())
        assert machine.is_ended
        assert result.state_changed
        assert result.has_execute

    def test_ended_without_standstill_goes_to_ending(self):
        machine = _executing()
        result = machine.process_motion_state(_ended(standstill=False))
        assert machine.is_ending
        assert machine.is_waiting_for_standstill
        assert result.state_changed

    def test_ending_then_standstill_ends(self):
        """Two-phase ending: TrajectoryEnded(no standstill) → standstill → ended."""
        machine = _executing()
        machine.process_motion_state(_ended(standstill=False))
        assert machine.is_ending
        result = machine.process_motion_state(_ended(standstill=True))
        assert machine.is_ended
        assert result.state_changed

    def test_ending_then_bare_standstill_ends(self):
        """A bare standstill frame (no execute) completes ending → ended.

        Current controllers drop the trajectory ``execute`` block the instant
        the robot settles (robotics/wbr MotionPointGenerator; changes with
        wbr!2262), so after ``TrajectoryEnded`` was observed, a bare standstill
        can be the only completion signal that ever arrives.
        """
        machine = _executing()
        machine.process_motion_state(_ended(standstill=False))
        assert machine.is_ending
        result = machine.process_motion_state(_bare())
        assert machine.is_ended
        assert result.state_changed
        assert not result.skip

    def test_pausing_then_bare_standstill_pauses(self):
        """A bare standstill frame (no execute) completes pausing → paused."""
        machine = _executing()
        machine.process_motion_state(_paused_by_user(standstill=False))
        assert machine.is_pausing
        result = machine.process_motion_state(_bare())
        assert machine.is_paused
        assert result.state_changed

    def test_bare_standstill_in_executing_does_not_end(self):
        """Without a terminal discriminator, bare standstill concludes nothing:
        the machine must not fabricate a completion from `executing`."""
        machine = _executing()
        result = machine.process_motion_state(_bare())
        assert machine.is_executing
        assert result.skip

    def test_ending_stays_in_ending_without_standstill(self):
        machine = _executing()
        machine.process_motion_state(_ended(standstill=False))
        machine.process_motion_state(_ended(standstill=False))
        assert machine.is_ending  # still waiting


# ---------------------------------------------------------------------------
# TrajectoryPausedByUser handling
# ---------------------------------------------------------------------------


class TestTrajectoryPaused:
    """TrajectoryPausedByUser with and without standstill, once the robot moved."""

    def test_paused_with_standstill_completes_to_paused(self):
        machine = _executing()
        result = machine.process_motion_state(_paused_by_user())
        assert machine.is_paused
        assert machine.pause_reason is PauseReason.USER
        assert not machine.is_paused_on_io
        assert result.state_changed

    def test_paused_without_standstill_goes_to_pausing(self):
        machine = _executing()
        result = machine.process_motion_state(_paused_by_user(standstill=False))
        assert machine.is_pausing
        assert machine.is_waiting_for_standstill
        assert result.state_changed

    def test_pausing_then_standstill_goes_to_paused(self):
        machine = _executing()
        machine.process_motion_state(_paused_by_user(standstill=False))
        assert machine.is_pausing
        machine.process_motion_state(_paused_by_user(standstill=True))
        assert machine.is_paused


# ---------------------------------------------------------------------------
# Transient states follow the discriminator
# ---------------------------------------------------------------------------


class TestTransientStatesFollowTheDiscriminator:
    """In ``pausing``/``ending`` the robot is still moving. A RUNNING frame there is
    not a contradiction but proof that the pause/end never settled; waiting for
    standstill would hang, or conclude the wrong thing at the next standstill
    flicker (the 2026-09-16 hang)."""

    def test_running_while_pausing_returns_to_executing(self):
        machine = _executing()
        machine.process_motion_state(_paused_by_user(1.0, standstill=False))
        assert machine.is_pausing
        result = machine.process_motion_state(_running(1.1))
        assert machine.is_executing
        assert result.state_changed
        assert machine.pause_reason is None

    def test_standstill_jitter_on_running_frames_does_not_pause(self):
        """The failing capture: RUNNING with standstill=True for eleven frames
        while the location keeps advancing."""
        machine = _executing()
        for location in (0.0065, 0.0077, 0.0091, 0.0105):
            machine.process_motion_state(_running(location, standstill=True))
        assert machine.is_executing

    def test_running_while_ending_returns_to_executing(self):
        machine = _executing()
        machine.process_motion_state(_ended(2.0, standstill=False))
        assert machine.is_ending
        machine.process_motion_state(_running(2.1))
        assert machine.is_executing

    def test_ended_while_pausing_completes_the_trajectory(self):
        """A pause requested just before the end: the trajectory finishes first."""
        machine = _executing()
        machine.process_motion_state(_paused_by_user(1.9, standstill=False))
        assert machine.is_pausing
        machine.process_motion_state(_ended(2.0, standstill=True))
        assert machine.is_ended

    def test_ended_while_pausing_without_standstill_goes_to_ending(self):
        machine = _executing()
        machine.process_motion_state(_paused_by_user(1.9, standstill=False))
        machine.process_motion_state(_ended(2.0, standstill=False))
        assert machine.is_ending
        machine.process_motion_state(_bare())
        assert machine.is_ended


# ---------------------------------------------------------------------------
# Rest states enforce the discriminator
# ---------------------------------------------------------------------------


class TestRestStatesEnforceTheDiscriminator:
    """In ``paused``/``ended`` the owner sends ``start`` before any frame of a new
    movement is processed. RUNNING arriving at rest therefore means the robot moves
    without this machine's owner having started it."""

    def test_running_while_paused_by_user_is_an_error(self):
        machine = _executing()
        machine.process_motion_state(_paused_by_user(1.0))
        assert machine.is_paused

        frame = _running(1.5)
        result = machine.process_motion_state(frame)

        assert machine.is_error
        assert machine.is_terminal
        assert result.state_changed
        assert machine.failed_frame is frame
        assert machine.failure_reason is not None
        assert "TrajectoryRunning" in machine.failure_reason
        assert "paused" in machine.failure_reason
        assert "1.5" in machine.failure_reason

    def test_running_while_ended_is_an_error(self):
        machine = _executing()
        machine.process_motion_state(_ended(2.0))
        assert machine.is_ended

        machine.process_motion_state(_running(2.5))

        assert machine.is_error
        assert machine.failure_reason is not None
        assert "ended" in machine.failure_reason

    def test_running_while_paused_on_io_is_an_observed_resume(self):
        """ADR 002: a controller that clears an IO pause by itself is conceivable;
        the machine follows the wire instead of failing."""
        machine = _executing()
        machine.process_motion_state(_paused_on_io(1.0))
        assert machine.is_paused_on_io

        result = machine.process_motion_state(_running(1.5))

        assert machine.is_executing
        assert machine.pause_reason is None
        assert result.state_changed
        assert result.location == 1.5

    def test_tolerated_frames_at_rest_change_nothing(self):
        machine = _executing()
        machine.process_motion_state(_paused_by_user(1.0))
        for frame in (_paused_by_user(1.0), _wait_for_io(1.0), _ended(1.0), _bare()):
            result = machine.process_motion_state(frame)
            assert machine.is_paused
            assert not result.state_changed

        machine = _executing()
        machine.process_motion_state(_ended(2.0))
        for frame in (_ended(2.0), _paused_by_user(2.0), _paused_on_io(2.0), _wait_for_io(2.0)):
            result = machine.process_motion_state(frame)
            assert machine.is_ended
            assert not result.state_changed

    def test_start_after_a_pause_makes_running_legitimate_again(self):
        machine = _executing()
        machine.process_motion_state(_paused_by_user(1.0))
        machine.arm()
        machine.process_motion_state(_running(1.5))
        assert machine.is_executing


# ---------------------------------------------------------------------------
# IO pauses (ADR 002)
# ---------------------------------------------------------------------------


class TestPausedOnIO:
    """``TrajectoryPausedOnIO`` is a pause, not completion (ADR 002).

    Measured 2026-09-03 (docs/architecture/incoming/pause-on-signal-evaluation.md):
    the controller holds the pause level-based and only resumes on a new start.
    """

    def test_paused_on_io_with_standstill_goes_to_paused_with_io_reason(self):
        machine = _executing()
        result = machine.process_motion_state(_paused_on_io())
        assert machine.is_paused
        assert not machine.is_ended
        assert machine.pause_reason is PauseReason.IO
        assert machine.is_paused_on_io
        assert result.state_changed

    def test_paused_on_io_without_standstill_goes_to_pausing_then_paused(self):
        machine = _executing()
        machine.process_motion_state(_paused_on_io(standstill=False))
        assert machine.is_pausing
        assert machine.is_paused_on_io
        # The decelerating frames keep the discriminator; a bare standstill concludes.
        machine.process_motion_state(_bare())
        assert machine.is_paused
        assert machine.is_paused_on_io

    def test_start_from_io_pause_clears_the_reason(self):
        machine = _executing()
        machine.process_motion_state(_paused_on_io())
        machine.arm()
        assert machine.is_armed
        assert machine.pause_reason is None
        assert not machine.is_paused_on_io

    def test_io_pause_then_end_completes_the_lifecycle(self):
        machine = _executing()
        machine.process_motion_state(_paused_on_io())
        machine.arm(stale_terminal=api.models.TrajectoryPausedOnIO)
        machine.process_motion_state(_paused_on_io())
        machine.process_motion_state(_running(1.5))
        machine.process_motion_state(_ended(3.0))
        assert machine.is_ended

    def test_pause_reason_follows_the_wire_while_paused(self):
        """A user pause that turns into an IO pause while standing (the bus-IO
        service came back and the condition holds) is an IO pause from then on."""
        machine = _executing()
        machine.process_motion_state(_paused_by_user())
        assert machine.pause_reason is PauseReason.USER
        machine.process_motion_state(_paused_on_io())
        assert machine.is_paused
        assert machine.pause_reason is PauseReason.IO
        assert machine.is_paused_on_io

    def test_pause_reason_follows_the_wire_while_pausing(self):
        machine = _executing()
        machine.process_motion_state(_paused_by_user(standstill=False))
        assert machine.pause_reason is PauseReason.USER
        machine.process_motion_state(_paused_on_io(standstill=False))
        assert machine.is_pausing
        assert machine.pause_reason is PauseReason.IO


# ---------------------------------------------------------------------------
# Resume from paused / completed
# ---------------------------------------------------------------------------


class TestResumeFromPaused:
    """start() transitions from paused or completed back to armed."""

    def test_start_from_paused(self):
        machine = _executing()
        machine.process_motion_state(_paused_by_user())
        assert machine.is_paused
        machine.send("start")
        assert machine.is_armed

    def test_start_from_ended(self):
        machine = _executing()
        machine.process_motion_state(_ended())
        assert machine.is_ended
        machine.send("start")
        assert machine.is_armed


# ---------------------------------------------------------------------------
# TrajectoryRunning (staying in executing)
# ---------------------------------------------------------------------------


class TestTrajectoryRunning:
    """TrajectoryRunning keeps the machine in executing."""

    def test_running_stays_in_executing(self):
        machine = _executing()
        result = machine.process_motion_state(_running(1.0))
        assert machine.is_executing
        assert not result.state_changed

    def test_running_with_standstill_stays_in_executing(self):
        """TrajectoryRunning + standstill should NOT complete."""
        machine = _executing()
        machine.process_motion_state(_running(1.0, standstill=True))
        assert machine.is_executing


# ---------------------------------------------------------------------------
# Skip / no-execute handling
# ---------------------------------------------------------------------------


class TestNoExecute:
    """States without execute are skipped when idle, armed or executing."""

    def test_no_execute_in_idle_is_skip(self):
        machine = TrajectoryExecutionMachine()
        result = machine.process_motion_state(_bare())
        assert result.skip
        assert machine.is_idle

    def test_no_execute_in_executing_is_skip(self):
        machine = _executing()
        result = machine.process_motion_state(_bare())
        assert result.skip
        assert machine.is_executing

    def test_non_trajectory_details_no_completion(self):
        """Execute with non-TrajectoryDetails should not trigger transitions."""
        machine = _executing()
        execute = api.models.Execute(joint_position=[0.0] * 6, details=None)
        result = machine.process_motion_state(_make_motion_group_state(True, execute))
        assert machine.is_executing
        assert result.has_execute
        assert result.location is None


# ---------------------------------------------------------------------------
# Location tracking
# ---------------------------------------------------------------------------


class TestLocationTracking:
    """The machine tracks the latest trajectory location."""

    def test_location_updated_from_trajectory_details(self):
        machine = _executing()
        result = machine.process_motion_state(_running(2.5))
        assert result.location == 2.5
        assert machine.location == 2.5

    def test_location_none_without_execute(self):
        machine = _executing()
        result = machine.process_motion_state(_bare(standstill=False))
        assert result.location is None

    def test_location_preserved_across_states(self):
        machine = _executing()
        machine.process_motion_state(_running(1.0))
        assert machine.location == 1.0
        machine.process_motion_state(_running(2.0))
        assert machine.location == 2.0


# ---------------------------------------------------------------------------
# Error transitions
# ---------------------------------------------------------------------------


class TestErrorTransitions:
    """fail() transitions to error from any non-error state."""

    @pytest.mark.parametrize(
        "setup",
        [
            pytest.param(lambda m: None, id="from_idle"),
            pytest.param(lambda m: m.arm(), id="from_armed"),
            pytest.param(
                lambda m: (m.arm(), m.process_motion_state(_running(0.5))), id="from_executing"
            ),
            pytest.param(
                lambda m: (
                    m.arm(),
                    m.process_motion_state(_running(0.5)),
                    m.process_motion_state(_paused_by_user()),
                ),
                id="from_paused",
            ),
            pytest.param(
                lambda m: (
                    m.arm(),
                    m.process_motion_state(_running(0.5)),
                    m.process_motion_state(_ended()),
                ),
                id="from_ended",
            ),
        ],
    )
    def test_fail_from_various_states(self, setup):
        machine = TrajectoryExecutionMachine()
        setup(machine)
        machine.send("fail")
        assert machine.is_error
        assert machine.is_terminal


# ---------------------------------------------------------------------------
# Full execution sequence
# ---------------------------------------------------------------------------


class TestFullSequence:
    """End-to-end sequences mirroring real controller flows."""

    def test_forward_to_completion(self):
        """Simulate a full forward execution: parked → running → ended → standstill."""
        machine = _armed()
        machine.process_motion_state(_paused_by_user(0.0))
        assert machine.is_armed

        for loc in [0.0, 0.5, 1.0, 1.5]:
            result = machine.process_motion_state(_running(loc))
            assert machine.is_executing
            assert result.location == loc

        machine.process_motion_state(_ended(2.0, standstill=False))
        assert machine.is_ending
        machine.process_motion_state(_ended(2.0, standstill=True))
        assert machine.is_ended
        assert machine.location == 2.0

    def test_forward_pause_resume_complete(self):
        """forward → pause → resume forward → complete."""
        machine = _armed()
        machine.process_motion_state(_running(0.5))
        assert machine.is_executing

        machine.process_motion_state(_paused_by_user(0.8))
        assert machine.is_paused

        # Resume: the controller re-publishes the parked shape until motion begins.
        machine.arm()
        assert machine.is_armed
        machine.process_motion_state(_paused_by_user(0.8))
        assert machine.is_armed
        machine.process_motion_state(_running(1.2))
        assert machine.is_executing

        machine.process_motion_state(_ended(2.0))
        assert machine.is_ended

    def test_forward_pause_backward_complete(self):
        """forward → pause → backward → complete (at start)."""
        machine = _armed()
        machine.process_motion_state(_running(1.0))
        machine.process_motion_state(_paused_by_user(1.0))
        assert machine.is_paused

        machine.arm()
        machine.process_motion_state(_running(0.5))
        assert machine.is_executing

        machine.process_motion_state(_ended(0.0))
        assert machine.is_ended
        assert machine.location == 0.0


class TestObservedStartLag:
    """Replay of the 2026-09-16 capture (trajectory 21a04c31, frames F007–F055)
    that hung ``execute()``: 22 parked frames, two frames with ``standstill``
    already False but the discriminator still PAUSED_BY_USER, RUNNING while the
    location ramps up, then eleven RUNNING frames with ``standstill=True``
    (jitter) before the motion runs to the end.

    The machine previously went ``executing → pausing → paused`` on the second
    and fourth of those shapes and then ignored the rest of the trajectory.
    """

    def test_start_lag_and_standstill_jitter_reach_ended_without_pausing(self):
        frames = (
            [_paused_by_user(0.0)] * 22
            + [_paused_by_user(0.0, standstill=False)] * 2
            + [_running(loc) for loc in (0.0, 1.4e-5, 2.8e-5, 1e-4, 2.6e-4, 5.2e-4, 9e-4)]
            + [_running(loc) for loc in (0.0014, 0.002, 0.0027, 0.0035, 0.0044, 0.0054)]
            + [_running(loc, standstill=True) for loc in (0.0065, 0.0077, 0.0091, 0.0105)]
            + [_running(loc, standstill=True) for loc in (0.012, 0.0137, 0.0155, 0.0173)]
            + [_running(loc, standstill=True) for loc in (0.0193, 0.0214, 0.0235)]
            + [_running(loc) for loc in (0.0258, 1.0, 2.09, 3.03, 4.62, 5.81, 6.83)]
            + [_ended(7.0, standstill=False)]
            + [_ended(7.0, standstill=True)] * 2
        )
        machine = _armed()
        visited = ["armed"]
        for frame in frames:
            result = machine.process_motion_state(frame)
            if result.state_changed:
                visited.append(result.current_state_id)

        assert visited == ["armed", "executing", "ending", "ended"]
        assert machine.is_ended
        assert machine.location == 7.0


# ---------------------------------------------------------------------------
# StateUpdate result tests
# ---------------------------------------------------------------------------


class TestStateUpdateResult:
    def test_state_update_properties(self):
        machine = _executing()
        result = machine.process_motion_state(_running(1.5))
        assert result.location == 1.5
        assert result.has_execute is True
        assert result.state_changed is False
        assert not result.skip
        assert result.current_state_id == "executing"

    def test_skip_on_no_execute_no_transition(self):
        machine = _executing()
        result = machine.process_motion_state(_bare(standstill=False))
        assert result.skip


# ---------------------------------------------------------------------------
# Replay of a real captured stream
# ---------------------------------------------------------------------------


class TestRecordedStreamReplay:
    def test_50ms_capture_edge_then_bare_standstill_reaches_ended(self):
        """Replay of a real 50 ms state-stream capture (virtual UR10e).

        The recorded shape is the measured production failure mode of the
        throttled stream: ``TrajectoryEnded`` arrives while the robot is still
        decelerating (standstill False), the controller then drops the
        ``execute`` block at settle, and only bare standstill frames follow.
        The machine previously discarded those frames and hung in ``ending``
        forever; it must complete to ``ended``.
        """
        fixture = json.loads(
            (
                Path(__file__).parent / "fixtures" / "frames_50ms_edge_then_bare_standstill.json"
            ).read_text()
        )
        machine = _armed()
        for raw in fixture["frames"]:
            machine.process_motion_state(api.models.MotionGroupState.model_validate(raw))
        assert machine.is_ended

    def test_step_capture_pause_then_bare_standstill_reaches_paused(self):
        """Replay of a real step-rate pause capture (the robotics/wbr!2322
        scenario on a current controller): PAUSED_BY_USER is published almost
        exclusively while still decelerating; standstill coincides with it for
        only the last couple of steps before the ``execute`` block drops.

        Replayed twice: the full capture, and a thinned variant with the
        PAUSED_BY_USER@standstill frames removed — which is exactly what a
        throttled stream delivers (frames are dropped, and the pause-at-
        standstill window is one to two steps wide). Both must reach
        ``paused``; the thinned variant previously hung in ``pausing``.
        """
        fixture = json.loads(
            (
                Path(__file__).parent / "fixtures" / "frames_step_pause_then_bare_standstill.json"
            ).read_text()
        )
        frames = [api.models.MotionGroupState.model_validate(raw) for raw in fixture["frames"]]

        machine = _armed()
        for state in frames:
            machine.process_motion_state(state)
        assert machine.is_paused

        def paused_at_standstill(state: api.models.MotionGroupState) -> bool:
            return (
                state.standstill
                and state.execute is not None
                and isinstance(state.execute.details, api.models.TrajectoryDetails)
                and isinstance(state.execute.details.state, api.models.TrajectoryPausedByUser)
            )

        thinned = [s for s in frames if not paused_at_standstill(s)]
        assert len(thinned) < len(frames), "fixture must contain the dropped-frame window"
        machine = _armed()
        for state in thinned:
            machine.process_motion_state(state)
        assert machine.is_paused
