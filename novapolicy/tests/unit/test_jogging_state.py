"""Tests for JoggingStateTracker — standstill/collision detection."""

from __future__ import annotations

from unittest.mock import MagicMock

import pytest

from novapolicy.jogging import JoggingStateTracker
from novapolicy.jogging.session import _BLOCKING_BRAKES
from novapolicy.types import MotionError


def _state(kind: str = "RUNNING", **kwargs) -> MagicMock:
    """Create a mock motion group state with jogging execution details."""
    jog_state = MagicMock()
    jog_state.kind = kind
    for k, v in kwargs.items():
        setattr(jog_state, k, v)
    details = MagicMock()
    details.state = jog_state
    execute = MagicMock()
    execute.details = details
    state = MagicMock()
    state.execute = execute
    return state


def test_running_state_no_error():
    t = JoggingStateTracker("0@ur10e", confirm_ticks=3)
    for _ in range(10):
        t.update_from_state(_state("RUNNING"))
        t.check()  # Should not raise


def test_collision_raises_after_confirm():
    t = JoggingStateTracker("0@ur10e", confirm_ticks=3)

    # First 2 ticks — confirmed but not yet at threshold
    for _ in range(2):
        t.update_from_state(_state("BRAKING_NEAR_COLLISION"))
        t.check()  # Should not raise yet

    # 3rd tick → should raise
    t.update_from_state(_state("BRAKING_NEAR_COLLISION"))
    with pytest.raises(MotionError, match="collision"):
        t.check()


def test_joint_limit_raises():
    t = JoggingStateTracker("0@ur10e", confirm_ticks=3)
    for _ in range(2):
        t.update_from_state(_state("BRAKING_NEAR_JOINT_LIMIT", joint_indices=[2, 4]))
        t.check()  # Should not raise yet (count < 3)
    t.update_from_state(_state("BRAKING_NEAR_JOINT_LIMIT", joint_indices=[2, 4]))
    with pytest.raises(MotionError, match="joint_limit"):
        t.check()


def test_singularity_raises():
    t = JoggingStateTracker("0@ur10e", confirm_ticks=3)
    for _ in range(2):
        t.update_from_state(_state("BRAKING_NEAR_SINGULARITY"))
        t.check()
    t.update_from_state(_state("BRAKING_NEAR_SINGULARITY"))
    with pytest.raises(MotionError, match="singularity"):
        t.check()


def test_workspace_boundary_raises():
    t = JoggingStateTracker("0@ur10e", confirm_ticks=3)
    for _ in range(2):
        t.update_from_state(_state("BRAKING_NEAR_WORKSPACE_BOUNDARY"))
        t.check()
    t.update_from_state(_state("BRAKING_NEAR_WORKSPACE_BOUNDARY"))
    with pytest.raises(MotionError, match="workspace_boundary"):
        t.check()


def test_transient_braking_resets():
    """A brief braking state followed by RUNNING doesn't accumulate."""
    t = JoggingStateTracker("0@ur10e", confirm_ticks=5)

    # 2 braking ticks
    for _ in range(2):
        t.update_from_state(_state("BRAKING_NEAR_COLLISION"))
        t.check()

    # Back to running — resets counter
    t.update_from_state(_state("RUNNING"))
    t.check()

    # 2 more braking ticks — still below threshold
    for _ in range(2):
        t.update_from_state(_state("BRAKING_NEAR_COLLISION"))
        t.check()  # Should not raise (only 2, not 5)


def test_no_execute_details_is_safe():
    """State without execute details (e.g., monitoring mode) is fine."""
    t = JoggingStateTracker("0@ur10e", confirm_ticks=2)
    state = MagicMock()
    state.execute = None
    for _ in range(10):
        t.update_from_state(state)
        t.check()


def test_unknown_state_type_does_not_trigger():
    """Only _BLOCKING_BRAKES trigger — other kinds are ignored."""
    t = JoggingStateTracker("0@ur10e", confirm_ticks=1)
    t.update_from_state(_state("PAUSED_SOME_OTHER_REASON"))
    t.check()  # Should not raise


def test_paused_by_user_is_recoverable_and_never_raises():
    """PAUSED_BY_USER (waypoint buffer exhausted) is recoverable, not a fault.

    It means "the buffer emptied, send chunks faster" — the robot resumes once a
    new chunk arrives. It must NOT be in _BLOCKING_BRAKES and must never raise,
    no matter how many consecutive ticks report it. This pins that contract so a
    well-meaning edit that "completes" the table by adding PAUSED_BY_USER to the
    blocking set would fail here.
    """
    assert "PAUSED_BY_USER" not in _BLOCKING_BRAKES
    t = JoggingStateTracker("0@ur10e", confirm_ticks=2)
    for _ in range(20):
        t.update_from_state(_state("PAUSED_BY_USER"))
        t.check()  # never raises


def test_stopped_by_user_is_terminal_but_not_motion_error():
    """STOPPED_BY_USER is reported by the stream but must not look like braking."""
    assert "STOPPED_BY_USER" not in _BLOCKING_BRAKES
    t = JoggingStateTracker("0@ur10e", confirm_ticks=2)
    for _ in range(20):
        t.update_from_state(_state("STOPPED_BY_USER"))
        t.check()  # never raises


# ===========================================================================
# Stateful property machine — the debounce contract over arbitrary tick
# sequences. A reference counter mirrors the tracker: it must raise iff
# `confirm_ticks` *consecutive* blocking ticks have been checked, and any
# non-blocking tick resets the streak.
# ===========================================================================

from hypothesis import (  # noqa: E402
    settings as _settings,
    strategies as st,
)
from hypothesis.stateful import (  # noqa: E402
    RuleBasedStateMachine,
    initialize,
    invariant,
    rule,
)

# Kinds that must NOT count toward a trip: RUNNING, user pause/stop, no execute
# details, and a state type that isn't one of the blocking brakes.
_CLEARING_KINDS = st.sampled_from(
    ["RUNNING", "PAUSED_BY_USER", "STOPPED_BY_USER", "PAUSED_SOME_OTHER_REASON", None]
)


def _tick_state(kind):
    """A jogging state whose pause kind is `kind` (None => no execute details)."""
    if kind is None:
        s = MagicMock()
        s.execute = None
        return s
    return _state(kind)


class JoggingDebounceMachine(RuleBasedStateMachine):
    """Drive a tracker with random blocking/clearing ticks; the model is a
    consecutive-blocking counter that predicts exactly when it must trip."""

    @initialize(ticks=st.integers(min_value=1, max_value=6))
    def start(self, ticks):
        self.confirm_ticks = ticks
        self.tracker = JoggingStateTracker("0@ur10e", confirm_ticks=ticks)
        self.streak = 0  # consecutive blocking ticks that have been checked

    @rule(kind=st.sampled_from(sorted(_BLOCKING_BRAKES)))
    def blocking_tick(self, kind):
        self.tracker.update_from_state(_tick_state(kind))
        self.streak += 1
        if self.streak >= self.confirm_ticks:
            # The model says we are at/over threshold -> must raise.
            with pytest.raises(MotionError):
                self.tracker.check()
        else:
            self.tracker.check()  # below threshold -> must not raise

    @rule(kind=_CLEARING_KINDS)
    def clearing_tick(self, kind):
        self.tracker.update_from_state(_tick_state(kind))
        self.streak = 0  # any non-blocking state resets the streak
        self.tracker.check()  # never raises

    @invariant()
    def streak_never_negative(self):
        assert self.streak >= 0


TestJoggingDebounce = JoggingDebounceMachine.TestCase
TestJoggingDebounce.settings = _settings(max_examples=200, stateful_step_count=40, deadline=None)
