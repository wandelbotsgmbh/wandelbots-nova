"""Tests for JoggingStateTracker — standstill/collision detection."""

from __future__ import annotations

from unittest.mock import MagicMock

import pytest

from novapolicy.jogging import JoggingStateTracker
from novapolicy.jogging.session import _BLOCKING_PAUSES
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


def test_collision_raises_after_confirm():
    t = JoggingStateTracker("0@ur10e", confirm_ticks=3)

    # First 2 ticks — confirmed but not yet at threshold
    for _ in range(2):
        t.update_from_state(_state("PAUSED_NEAR_COLLISION"))
        t.check()  # Should not raise yet

    # 3rd tick → should raise
    t.update_from_state(_state("PAUSED_NEAR_COLLISION"))
    with pytest.raises(MotionError, match="collision"):
        t.check()


def test_joint_limit_raises():
    t = JoggingStateTracker("0@ur10e", confirm_ticks=3)
    for _ in range(2):
        t.update_from_state(_state("PAUSED_NEAR_JOINT_LIMIT", joint_indices=[2, 4]))
        t.check()  # Should not raise yet (count < 3)
    t.update_from_state(_state("PAUSED_NEAR_JOINT_LIMIT", joint_indices=[2, 4]))
    with pytest.raises(MotionError, match="joint_limit"):
        t.check()


def test_singularity_raises():
    t = JoggingStateTracker("0@ur10e", confirm_ticks=3)
    for _ in range(2):
        t.update_from_state(_state("PAUSED_NEAR_SINGULARITY"))
        t.check()
    t.update_from_state(_state("PAUSED_NEAR_SINGULARITY"))
    with pytest.raises(MotionError, match="singularity"):
        t.check()


def test_paused_by_user_is_recoverable_and_never_raises():
    """PAUSED_BY_USER (waypoint buffer exhausted) is recoverable, not a fault.

    jogging.md lists PAUSED_BY_USER alongside the fatal pause states, but it
    means "the buffer emptied, send chunks faster" — the robot resumes once a
    new chunk arrives. It must NOT be in _BLOCKING_PAUSES and must never raise,
    no matter how many consecutive ticks report it. This pins that contract so a
    well-meaning edit that "completes" the table by adding PAUSED_BY_USER to the
    blocking set would fail here.
    """
    assert "PAUSED_BY_USER" not in _BLOCKING_PAUSES
    t = JoggingStateTracker("0@ur10e", confirm_ticks=2)
    for _ in range(20):
        t.update_from_state(_state("PAUSED_BY_USER"))
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

# Kinds that must NOT count toward a trip: RUNNING, no execute details, and a
# pause type that isn't one of the blocking pauses.
_CLEARING_KINDS = st.sampled_from(["RUNNING", "PAUSED_SOME_OTHER_REASON", None])


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

    @rule(kind=st.sampled_from(sorted(_BLOCKING_PAUSES)))
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
