"""Tests for the TrajectoryExecutionMachine state machine.

These tests verify the state machine models the trajectory execution lifecycle
correctly, including forward/backward movement, pauses, standstill detection
and multi-phase completion (TrajectoryEnded followed by standstill).
"""

from datetime import datetime, timezone

import pytest

from nova import api
from nova.cell.movement_controller.trajectory_state_machine import (
    StateUpdate,
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
        motion_group="mg-0",
        controller="ctrl-0",
        joint_position=api.models.Joints(root=[0.0] * 6),
        joint_limit_reached=api.models.MotionGroupStateJointLimitReached(
            limit_reached=[False] * 6
        ),
        standstill=standstill,
        execute=execute,
    )


def _make_execute(
    trajectory_state: (
        api.models.TrajectoryRunning
        | api.models.TrajectoryEnded
        | api.models.TrajectoryPausedByUser
    ),
    location: float = 1.0,
) -> api.models.Execute:
    """Create an Execute with TrajectoryDetails."""
    return api.models.Execute(
        joint_position=[0.0] * 6,
        details=api.models.TrajectoryDetails(
            trajectory="traj-123",
            location=api.models.Location(root=location),
            state=trajectory_state,
        ),
    )


# ---------------------------------------------------------------------------
# State machine lifecycle tests
# ---------------------------------------------------------------------------


class TestStateMachineLifecycle:
    """Basic lifecycle: idle → start → executing → completed."""

    def test_initial_state_is_idle(self):
        machine = TrajectoryExecutionMachine()
        assert machine.is_idle

    def test_start_transitions_to_executing(self):
        machine = TrajectoryExecutionMachine()
        machine.send("start")
        assert machine.is_executing

    def test_completed_is_terminal(self):
        machine = TrajectoryExecutionMachine()
        machine.send("start")
        state = _make_motion_group_state(
            standstill=True, execute=_make_execute(api.models.TrajectoryEnded())
        )
        machine.process_motion_state(state)
        assert machine.is_completed
        assert machine.is_terminal

    def test_error_is_terminal(self):
        machine = TrajectoryExecutionMachine()
        machine.send("fail")
        assert machine.is_error
        assert machine.is_terminal


# ---------------------------------------------------------------------------
# TrajectoryEnded handling
# ---------------------------------------------------------------------------


class TestTrajectoryEnded:
    """TrajectoryEnded with and without standstill."""

    def test_ended_with_standstill_completes_immediately(self):
        machine = TrajectoryExecutionMachine()
        machine.send("start")

        state = _make_motion_group_state(
            standstill=True, execute=_make_execute(api.models.TrajectoryEnded())
        )
        result = machine.process_motion_state(state)

        assert machine.is_completed
        assert result.state_changed
        assert result.has_execute

    def test_ended_without_standstill_goes_to_ending(self):
        machine = TrajectoryExecutionMachine()
        machine.send("start")

        state = _make_motion_group_state(
            standstill=False, execute=_make_execute(api.models.TrajectoryEnded())
        )
        result = machine.process_motion_state(state)

        assert machine.is_ending
        assert machine.is_waiting_for_standstill
        assert result.state_changed

    def test_ending_then_standstill_completes(self):
        """Two-phase completion: TrajectoryEnded(no standstill) → standstill → completed."""
        machine = TrajectoryExecutionMachine()
        machine.send("start")

        # Phase 1: TrajectoryEnded without standstill
        state1 = _make_motion_group_state(
            standstill=False, execute=_make_execute(api.models.TrajectoryEnded())
        )
        machine.process_motion_state(state1)
        assert machine.is_ending

        # Phase 2: Standstill (with execute still present, as API keeps it)
        state2 = _make_motion_group_state(
            standstill=True, execute=_make_execute(api.models.TrajectoryEnded())
        )
        result = machine.process_motion_state(state2)
        assert machine.is_completed
        assert result.state_changed

    def test_ending_then_bare_standstill_does_not_complete(self):
        """Standalone standstill without execute does NOT complete from ending state.

        The API guarantees execute persists once set, so bare standstill is
        unreliable for determining completion.
        """
        machine = TrajectoryExecutionMachine()
        machine.send("start")

        state1 = _make_motion_group_state(
            standstill=False, execute=_make_execute(api.models.TrajectoryEnded())
        )
        machine.process_motion_state(state1)
        assert machine.is_ending

        # Bare standstill (no execute) — should NOT complete
        state2 = _make_motion_group_state(standstill=True)
        machine.process_motion_state(state2)
        assert machine.is_ending  # still waiting

    def test_ending_stays_in_ending_without_standstill(self):
        machine = TrajectoryExecutionMachine()
        machine.send("start")

        state1 = _make_motion_group_state(
            standstill=False, execute=_make_execute(api.models.TrajectoryEnded())
        )
        machine.process_motion_state(state1)
        assert machine.is_ending

        # Another state without standstill
        state2 = _make_motion_group_state(
            standstill=False, execute=_make_execute(api.models.TrajectoryEnded())
        )
        machine.process_motion_state(state2)
        assert machine.is_ending  # still waiting


# ---------------------------------------------------------------------------
# TrajectoryPausedByUser handling
# ---------------------------------------------------------------------------


class TestTrajectoryPaused:
    """TrajectoryPausedByUser with and without standstill."""

    def test_paused_with_standstill_completes_to_paused(self):
        machine = TrajectoryExecutionMachine()
        machine.send("start")

        state = _make_motion_group_state(
            standstill=True, execute=_make_execute(api.models.TrajectoryPausedByUser())
        )
        result = machine.process_motion_state(state)

        assert machine.is_paused
        assert result.state_changed

    def test_paused_without_standstill_goes_to_pausing(self):
        machine = TrajectoryExecutionMachine()
        machine.send("start")

        state = _make_motion_group_state(
            standstill=False,
            execute=_make_execute(api.models.TrajectoryPausedByUser()),
        )
        result = machine.process_motion_state(state)

        assert machine.is_pausing
        assert machine.is_waiting_for_standstill
        assert result.state_changed

    def test_pausing_then_standstill_goes_to_paused(self):
        machine = TrajectoryExecutionMachine()
        machine.send("start")

        state1 = _make_motion_group_state(
            standstill=False,
            execute=_make_execute(api.models.TrajectoryPausedByUser()),
        )
        machine.process_motion_state(state1)
        assert machine.is_pausing

        state2 = _make_motion_group_state(
            standstill=True,
            execute=_make_execute(api.models.TrajectoryPausedByUser()),
        )
        machine.process_motion_state(state2)
        assert machine.is_paused


# ---------------------------------------------------------------------------
# Resume from paused / completed
# ---------------------------------------------------------------------------


class TestResumeFromPaused:
    """start() transitions from paused or completed back to executing."""

    def test_start_from_paused(self):
        machine = TrajectoryExecutionMachine()
        machine.send("start")

        # Go to paused
        state = _make_motion_group_state(
            standstill=True, execute=_make_execute(api.models.TrajectoryPausedByUser())
        )
        machine.process_motion_state(state)
        assert machine.is_paused

        # Resume
        machine.send("start")
        assert machine.is_executing

    def test_start_from_completed(self):
        machine = TrajectoryExecutionMachine()
        machine.send("start")

        state = _make_motion_group_state(
            standstill=True, execute=_make_execute(api.models.TrajectoryEnded())
        )
        machine.process_motion_state(state)
        assert machine.is_completed

        # Restart
        machine.send("start")
        assert machine.is_executing


# ---------------------------------------------------------------------------
# TrajectoryRunning (staying in executing)
# ---------------------------------------------------------------------------


class TestTrajectoryRunning:
    """TrajectoryRunning keeps the machine in executing."""

    def test_running_stays_in_executing(self):
        machine = TrajectoryExecutionMachine()
        machine.send("start")

        state = _make_motion_group_state(
            standstill=False,
            execute=_make_execute(api.models.TrajectoryRunning(time_to_end=5000)),
        )
        result = machine.process_motion_state(state)

        assert machine.is_executing
        assert not result.state_changed

    def test_running_with_standstill_stays_in_executing(self):
        """TrajectoryRunning + standstill should NOT complete."""
        machine = TrajectoryExecutionMachine()
        machine.send("start")

        state = _make_motion_group_state(
            standstill=True,
            execute=_make_execute(api.models.TrajectoryRunning(time_to_end=5000)),
        )
        machine.process_motion_state(state)
        assert machine.is_executing


# ---------------------------------------------------------------------------
# Skip / no-execute handling
# ---------------------------------------------------------------------------


class TestNoExecute:
    """States without execute are skipped when idle or executing."""

    def test_no_execute_in_idle_is_skip(self):
        machine = TrajectoryExecutionMachine()
        state = _make_motion_group_state(standstill=True)
        result = machine.process_motion_state(state)
        assert result.skip
        assert machine.is_idle

    def test_no_execute_in_executing_is_skip(self):
        machine = TrajectoryExecutionMachine()
        machine.send("start")
        state = _make_motion_group_state(standstill=True)
        result = machine.process_motion_state(state)
        assert result.skip
        assert machine.is_executing

    def test_non_trajectory_details_no_completion(self):
        """Execute with non-TrajectoryDetails should not trigger transitions."""
        machine = TrajectoryExecutionMachine()
        machine.send("start")

        execute = api.models.Execute(joint_position=[0.0] * 6, details=None)
        state = _make_motion_group_state(standstill=True, execute=execute)
        result = machine.process_motion_state(state)
        assert machine.is_executing
        assert result.has_execute
        assert result.location is None


# ---------------------------------------------------------------------------
# Location tracking
# ---------------------------------------------------------------------------


class TestLocationTracking:
    """The machine tracks the latest trajectory location."""

    def test_location_updated_from_trajectory_details(self):
        machine = TrajectoryExecutionMachine()
        machine.send("start")

        state = _make_motion_group_state(
            standstill=False,
            execute=_make_execute(
                api.models.TrajectoryRunning(time_to_end=5000), location=2.5
            ),
        )
        result = machine.process_motion_state(state)

        assert result.location == 2.5
        assert machine.location == 2.5

    def test_location_none_without_execute(self):
        machine = TrajectoryExecutionMachine()
        machine.send("start")
        state = _make_motion_group_state(standstill=False)
        result = machine.process_motion_state(state)
        assert result.location is None

    def test_location_preserved_across_states(self):
        machine = TrajectoryExecutionMachine()
        machine.send("start")

        state1 = _make_motion_group_state(
            standstill=False,
            execute=_make_execute(
                api.models.TrajectoryRunning(time_to_end=5000), location=1.0
            ),
        )
        machine.process_motion_state(state1)
        assert machine.location == 1.0

        state2 = _make_motion_group_state(
            standstill=False,
            execute=_make_execute(
                api.models.TrajectoryRunning(time_to_end=3000), location=2.0
            ),
        )
        machine.process_motion_state(state2)
        assert machine.location == 2.0


# ---------------------------------------------------------------------------
# Error transitions
# ---------------------------------------------------------------------------


class TestErrorTransitions:
    """fail() transitions to error from any non-terminal state."""

    @pytest.mark.parametrize(
        "setup",
        [
            pytest.param(lambda m: None, id="from_idle"),
            pytest.param(lambda m: m.send("start"), id="from_executing"),
            pytest.param(
                lambda m: (
                    m.send("start"),
                    m.process_motion_state(
                        _make_motion_group_state(
                            standstill=True,
                            execute=_make_execute(api.models.TrajectoryPausedByUser()),
                        )
                    ),
                ),
                id="from_paused",
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
        """Simulate a full forward execution: running → ended → standstill."""
        machine = TrajectoryExecutionMachine()
        machine.send("start")

        # Running states
        for loc in [0.0, 0.5, 1.0, 1.5]:
            state = _make_motion_group_state(
                standstill=False,
                execute=_make_execute(
                    api.models.TrajectoryRunning(time_to_end=int((2.0 - loc) * 1000)),
                    location=loc,
                ),
            )
            result = machine.process_motion_state(state)
            assert machine.is_executing
            assert result.location == loc

        # TrajectoryEnded without standstill
        state = _make_motion_group_state(
            standstill=False,
            execute=_make_execute(api.models.TrajectoryEnded(), location=2.0),
        )
        machine.process_motion_state(state)
        assert machine.is_ending

        # Standstill
        state = _make_motion_group_state(
            standstill=True,
            execute=_make_execute(api.models.TrajectoryEnded(), location=2.0),
        )
        machine.process_motion_state(state)
        assert machine.is_completed
        assert machine.location == 2.0

    def test_forward_pause_resume_complete(self):
        """forward → pause → resume forward → complete."""
        machine = TrajectoryExecutionMachine()
        machine.send("start")

        # Running
        state = _make_motion_group_state(
            standstill=False,
            execute=_make_execute(
                api.models.TrajectoryRunning(time_to_end=5000), location=0.5
            ),
        )
        machine.process_motion_state(state)
        assert machine.is_executing

        # Paused by user + standstill
        state = _make_motion_group_state(
            standstill=True,
            execute=_make_execute(api.models.TrajectoryPausedByUser(), location=0.8),
        )
        machine.process_motion_state(state)
        assert machine.is_paused

        # Resume
        machine.send("start")
        assert machine.is_executing

        # Running again
        state = _make_motion_group_state(
            standstill=False,
            execute=_make_execute(
                api.models.TrajectoryRunning(time_to_end=2000), location=1.2
            ),
        )
        machine.process_motion_state(state)
        assert machine.is_executing

        # Ended + standstill
        state = _make_motion_group_state(
            standstill=True,
            execute=_make_execute(api.models.TrajectoryEnded(), location=2.0),
        )
        machine.process_motion_state(state)
        assert machine.is_completed

    def test_forward_pause_backward_complete(self):
        """forward → pause → backward → complete (at start)."""
        machine = TrajectoryExecutionMachine()
        machine.send("start")

        # Running forward
        state = _make_motion_group_state(
            standstill=False,
            execute=_make_execute(
                api.models.TrajectoryRunning(time_to_end=5000), location=1.0
            ),
        )
        machine.process_motion_state(state)

        # Paused
        state = _make_motion_group_state(
            standstill=True,
            execute=_make_execute(api.models.TrajectoryPausedByUser(), location=1.0),
        )
        machine.process_motion_state(state)
        assert machine.is_paused

        # Start backward
        machine.send("start")
        assert machine.is_executing

        # Running backward
        state = _make_motion_group_state(
            standstill=False,
            execute=_make_execute(
                api.models.TrajectoryRunning(time_to_end=3000), location=0.5
            ),
        )
        machine.process_motion_state(state)
        assert machine.is_executing

        # Reached start, ended
        state = _make_motion_group_state(
            standstill=True,
            execute=_make_execute(api.models.TrajectoryEnded(), location=0.0),
        )
        machine.process_motion_state(state)
        assert machine.is_completed
        assert machine.location == 0.0


# ---------------------------------------------------------------------------
# StateUpdate result tests
# ---------------------------------------------------------------------------


class TestStateUpdateResult:
    def test_state_update_properties(self):
        machine = TrajectoryExecutionMachine()
        machine.send("start")

        state = _make_motion_group_state(
            standstill=False,
            execute=_make_execute(
                api.models.TrajectoryRunning(time_to_end=5000), location=1.5
            ),
        )
        result = machine.process_motion_state(state)

        assert result.location == 1.5
        assert result.has_execute is True
        assert result.state_changed is False
        assert not result.skip
        assert result.current_state_id == "executing"

    def test_skip_on_no_execute_no_transition(self):
        machine = TrajectoryExecutionMachine()
        machine.send("start")
        state = _make_motion_group_state(standstill=False)
        result = machine.process_motion_state(state)
        assert result.skip
