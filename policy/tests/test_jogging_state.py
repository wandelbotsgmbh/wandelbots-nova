"""Tests for JoggingStateTracker — standstill/collision detection."""

from __future__ import annotations

from unittest.mock import MagicMock

import pytest

from policy.pidjogging import JoggingStateTracker
from policy.types import MotionError


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


def test_transient_pause_resets():
    """A brief pause followed by RUNNING doesn't accumulate."""
    t = JoggingStateTracker("0@ur10e", confirm_ticks=5)

    # 2 ticks paused
    for _ in range(2):
        t.update_from_state(_state("PAUSED_NEAR_COLLISION"))
        t.check()

    # Back to running — resets counter
    t.update_from_state(_state("RUNNING"))
    t.check()

    # 2 more ticks paused — still below threshold
    for _ in range(2):
        t.update_from_state(_state("PAUSED_NEAR_COLLISION"))
        t.check()  # Should not raise (only 2, not 5)


def test_no_execute_details_is_safe():
    """State without execute details (e.g., monitoring mode) is fine."""
    t = JoggingStateTracker("0@ur10e", confirm_ticks=2)
    state = MagicMock()
    state.execute = None
    for _ in range(10):
        t.update_from_state(state)
        t.check()


def test_unknown_pause_type_does_not_trigger():
    """Only _BLOCKING_PAUSES trigger — other kinds are ignored."""
    t = JoggingStateTracker("0@ur10e", confirm_ticks=1)
    t.update_from_state(_state("PAUSED_SOME_OTHER_REASON"))
    t.check()  # Should not raise
