"""Tests for StandstillDetector — has the robot actually stopped moving?

NOVA reports no "the commanded waypoints ran out" state: since 26.6
``PAUSED_BY_USER`` follows a Pause/Stop *request* only, and the SDK never sends
one, so a drained queue reports ``RUNNING`` indefinitely. Standstill therefore
has to be measured from the joint positions, and this is that measurement's
contract.
"""

from __future__ import annotations

from hypothesis import given, settings, strategies as st

from novapolicy.jogging.session import (
    _STANDSTILL_EPS_RAD,
    _STANDSTILL_HOLD_MS,
    StandstillDetector,
)


def _still(detector: StandstillDetector, *, until_ms: int, step_ms: int = 10) -> None:
    """Feed identical samples up to and including ``until_ms``."""
    for ms in range(0, until_ms + 1, step_ms):
        detector.update(ms, [0.0] * 6)


# ---------------------------------------------------------------------------
# The hold window
# ---------------------------------------------------------------------------


def test_no_state_yet_is_not_a_standstill():
    """Before the first sample nothing is known, and unknown must not read as stopped."""
    detector = StandstillDetector()
    assert detector.is_settled is False
    assert detector.standstill_ms == 0.0


def test_the_first_sample_alone_is_not_a_standstill():
    """One sample dates the reference; it says nothing about how long it has held."""
    detector = StandstillDetector(hold_ms=60)
    detector.update(1_000, [0.1] * 6)
    assert detector.is_settled is False


def test_it_settles_once_the_joints_have_held_for_the_hold_window():
    detector = StandstillDetector(hold_ms=60)
    _still(detector, until_ms=50)
    assert detector.is_settled is False
    _still(detector, until_ms=60)
    assert detector.is_settled is True
    assert detector.standstill_ms == 60.0


def test_motion_resets_the_hold():
    detector = StandstillDetector(hold_ms=60)
    _still(detector, until_ms=100)
    assert detector.is_settled is True

    detector.update(110, [0.5] * 6)  # moved
    assert detector.is_settled is False
    assert detector.standstill_ms == 0.0

    detector.update(170, [0.5] * 6)  # still again, for the full window
    assert detector.is_settled is True


# ---------------------------------------------------------------------------
# Noise vs. crawl — why the reference is held rather than differenced
# ---------------------------------------------------------------------------


def test_encoder_noise_under_the_epsilon_never_counts_as_motion():
    """Jitter inside the band must not keep resetting the hold.

    The reference stays put while every joint is inside the band, so bounded
    noise can never accumulate out of it however long the session runs.
    """
    detector = StandstillDetector(hold_ms=60)
    noise = _STANDSTILL_EPS_RAD / 2
    for i in range(200):
        detector.update(i * 10, [noise if i % 2 else -noise] * 6)
    assert detector.is_settled is True


def test_a_crawl_slower_than_the_epsilon_per_sample_still_counts_as_motion():
    """The case a sample-to-sample difference would miss entirely.

    Each step moves less than one epsilon, so differencing consecutive samples
    reads standstill forever. Measuring against a held reference lets the drift
    accumulate until it leaves the band.
    """
    detector = StandstillDetector(hold_ms=60)
    creep = _STANDSTILL_EPS_RAD / 4
    joints = [0.0] * 6
    settled_at_any_point = False
    for i in range(1, 400):
        joints = [j + creep for j in joints]
        detector.update(i * 10, list(joints))
        settled_at_any_point = settled_at_any_point or detector.is_settled
    assert settled_at_any_point is False


def test_one_moving_joint_is_enough_to_count_as_motion():
    detector = StandstillDetector(hold_ms=60)
    _still(detector, until_ms=100)
    assert detector.is_settled is True
    detector.update(110, [0.0, 0.0, 0.0, 0.0, 0.0, 0.5])
    assert detector.is_settled is False


# ---------------------------------------------------------------------------
# Server time, not sample counts
# ---------------------------------------------------------------------------


def test_the_hold_is_measured_in_server_time_not_in_samples():
    """Two samples spanning the window settle; many packed into it do not.

    State packets arrive in bursts on some deployments, so a sample count would
    call a burst-delivered standstill far too early.
    """
    burst = StandstillDetector(hold_ms=60)
    for i in range(20):  # twenty samples, but only 19ms of server time
        burst.update(1_000 + i, [0.0] * 6)
    assert burst.is_settled is False

    sparse = StandstillDetector(hold_ms=60)
    sparse.update(1_000, [0.0] * 6)
    sparse.update(1_100, [0.0] * 6)  # two samples, 100ms apart
    assert sparse.is_settled is True


def test_a_changed_joint_count_re_anchors_instead_of_comparing():
    """A differently-shaped sample cannot be compared, so it counts as motion."""
    detector = StandstillDetector(hold_ms=60)
    _still(detector, until_ms=100)
    assert detector.is_settled is True
    detector.update(110, [0.0] * 7)
    assert detector.is_settled is False


# ---------------------------------------------------------------------------
# Property: a settled report always means the full window was quiet
# ---------------------------------------------------------------------------


@given(
    hold_ms=st.integers(min_value=10, max_value=200),
    quiet_ms=st.integers(min_value=0, max_value=400),
)
@settings(max_examples=200, deadline=None)
def test_settled_iff_the_quiet_span_covers_the_hold(hold_ms, quiet_ms):
    """The report is exactly "quiet for at least hold_ms", never an approximation."""
    detector = StandstillDetector(hold_ms=hold_ms)
    detector.update(0, [0.3] * 6)  # anchor, counts as motion
    detector.update(quiet_ms, [0.3] * 6)
    assert detector.standstill_ms == float(quiet_ms)
    assert detector.is_settled is (quiet_ms >= hold_ms)


def test_the_default_hold_is_the_module_constant():
    """The default must track the documented constant, not a copy of its value."""
    detector = StandstillDetector()
    detector.update(0, [0.0] * 6)
    detector.update(_STANDSTILL_HOLD_MS, [0.0] * 6)
    assert detector.is_settled is True
