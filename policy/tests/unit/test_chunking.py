"""Tests for the pure action-chunk transforms in :mod:`policy.chunking`.

These are stateless functions the executor leans on for receding-horizon
trimming, relative→absolute conversion, chunk timing, and where each chunk
gets placed on the jogging timeline. They have no I/O, so they are tested
directly with plain data.
"""

from __future__ import annotations

from types import SimpleNamespace

from hypothesis import given, settings, strategies as st
import pytest

from policy.chunking import (
    apply_relative_mode,
    chunk_duration_s,
    placement_start_ms,
    trim_chunk,
)
from policy.types import ActionChunk

# Strategies for generated chunks --------------------------------------------

_J = st.floats(min_value=-10.0, max_value=10.0, allow_nan=False, allow_infinity=False)
_STEP = st.lists(_J, min_size=6, max_size=6)
_STEPS = st.lists(_STEP, min_size=0, max_size=20)

# ---------------------------------------------------------------------------
# chunk_duration_s — how long a chunk takes to play out
# ---------------------------------------------------------------------------


def test_duration_is_steps_times_dt():
    """A 4-step chunk at 50 ms/step lasts 0.2 s."""
    chunk = ActionChunk(joints={"0@ur5e": [[0.0] * 6] * 4}, dt_ms=50.0)
    assert chunk_duration_s(chunk) == 0.2


def test_duration_uses_the_longest_arm():
    """With uneven arms, duration follows the one with the most steps."""
    chunk = ActionChunk(
        joints={"0@ur5e": [[0.0] * 6] * 2, "0@ur10e": [[0.0] * 6] * 5},
        dt_ms=10.0,
    )
    assert chunk_duration_s(chunk) == 0.05


def test_duration_zero_when_dt_unset():
    """A single-step chunk (dt_ms=0) has no meaningful duration."""
    chunk = ActionChunk(joints={"0@ur5e": [[0.0] * 6]})
    assert chunk_duration_s(chunk) == 0.0


# ---------------------------------------------------------------------------
# trim_chunk — receding horizon: keep only the first n steps
# ---------------------------------------------------------------------------


def test_trim_keeps_first_n_steps():
    """Trimming a 16-step chunk to 8 keeps the first 8 steps per arm."""
    chunk = ActionChunk(joints={"0@ur5e": [[float(i)] * 6 for i in range(16)]}, dt_ms=33.0)
    trimmed = trim_chunk(chunk, 8)
    assert len(trimmed.joints["0@ur5e"]) == 8
    assert trimmed.joints["0@ur5e"][0][0] == 0.0
    assert trimmed.joints["0@ur5e"][-1][0] == 7.0
    # Metadata is preserved.
    assert trimmed.dt_ms == 33.0


def test_trim_zero_means_execute_all():
    """n <= 0 is 'no trimming' — the whole chunk runs."""
    chunk = ActionChunk(joints={"0@ur5e": [[0.0] * 6] * 12}, dt_ms=33.0)
    assert trim_chunk(chunk, 0) is chunk


def test_trim_also_trims_tcp_steps():
    """TCP targets are trimmed the same way as joint targets."""
    chunk = ActionChunk(tcp={"0@ur5e": [[float(i)] * 6 for i in range(10)]}, dt_ms=33.0)
    trimmed = trim_chunk(chunk, 3)
    assert len(trimmed.tcp["0@ur5e"]) == 3


def test_trim_shorter_than_n_is_left_alone(caplog):
    """If the policy returned fewer steps than requested, keep them and warn."""
    chunk = ActionChunk(joints={"0@ur5e": [[0.0] * 6] * 4}, dt_ms=33.0)
    trimmed = trim_chunk(chunk, 8)
    assert len(trimmed.joints["0@ur5e"]) == 4
    assert "Policy returned 4 steps" in caplog.text


# ---------------------------------------------------------------------------
# apply_relative_mode — convert delta targets to absolute positions
# ---------------------------------------------------------------------------


def test_relative_joint_deltas_accumulate_from_current_state():
    """Relative joint steps are offsets that add up on top of the robot's pose."""
    state = SimpleNamespace(joints=[1.0, 2.0, 3.0])
    # Two steps of +0.5 each on joint 0 → 1.5 then 2.0 (exact in float).
    chunk = ActionChunk(joints={"0@ur5e": [[0.5, 0.0, 0.0], [0.5, 0.0, 0.0]]})
    out = apply_relative_mode(chunk, {"0@ur5e": state}, relative_mgs=["0@ur5e"])
    assert out.joints["0@ur5e"][0] == [1.5, 2.0, 3.0]
    assert out.joints["0@ur5e"][1] == [2.0, 2.0, 3.0]


def test_no_relative_groups_is_a_passthrough():
    """With no relative motion groups, the chunk is returned untouched."""
    chunk = ActionChunk(joints={"0@ur5e": [[0.1, 0.0, 0.0]]})
    assert apply_relative_mode(chunk, {}, relative_mgs=[]) is chunk


def test_relative_skips_groups_without_state():
    """A relative group with no current state is left as-is (no crash)."""
    chunk = ActionChunk(joints={"0@ur5e": [[0.1, 0.0, 0.0]]})
    out = apply_relative_mode(chunk, {}, relative_mgs=["0@ur5e"])
    assert out.joints["0@ur5e"] == [[0.1, 0.0, 0.0]]


def test_relative_tcp_offsets_from_current_pose():
    """Relative TCP steps add to the robot's current [position + orientation]."""
    pose = SimpleNamespace(position=[10.0, 20.0, 30.0], orientation=[0.0, 0.0, 0.0])
    state = SimpleNamespace(joints=[], pose=pose)
    chunk = ActionChunk(tcp={"0@ur5e": [[1.0, 2.0, 3.0, 0.0, 0.0, 0.0]]})
    out = apply_relative_mode(chunk, {"0@ur5e": state}, relative_mgs=["0@ur5e"])
    assert out.tcp["0@ur5e"][0] == [11.0, 22.0, 33.0, 0.0, 0.0, 0.0]


# ---------------------------------------------------------------------------
# placement_start_ms — where a chunk lands on the session timeline
# ---------------------------------------------------------------------------


def test_explicit_start_time_always_wins():
    """A first_timestamp_ms the policy set explicitly is used verbatim."""
    chunk = ActionChunk(joints={"0@ur5e": [[0.0]]}, first_timestamp_ms=1234)
    assert (
        placement_start_ms(chunk, policy_rate_hz=20, session_elapsed_ms=999, backdate_ms=50) == 1234
    )


def test_wait_for_chunk_uses_relative_placement():
    """Sequential mode (policy_rate_hz < 0) places chunks relative ('now') = -1."""
    chunk = ActionChunk(joints={"0@ur5e": [[0.0]]})  # first_timestamp_ms defaults to -1
    assert (
        placement_start_ms(chunk, policy_rate_hz=-1, session_elapsed_ms=5000, backdate_ms=80) == -1
    )


def test_overlapping_mode_anchors_and_backdates():
    """Overlapping mode (rate >= 0) anchors at send time minus the seam backdate."""
    chunk = ActionChunk(joints={"0@ur5e": [[0.0]]})
    assert (
        placement_start_ms(chunk, policy_rate_hz=20, session_elapsed_ms=5000, backdate_ms=800)
        == 4200
    )


def test_overlapping_backdate_never_goes_negative():
    """Early in a session the backdate is clamped so placement stays >= 0."""
    chunk = ActionChunk(joints={"0@ur5e": [[0.0]]})
    assert (
        placement_start_ms(chunk, policy_rate_hz=20, session_elapsed_ms=100, backdate_ms=800) == 0
    )


# ===========================================================================
# Property-based invariants — these generalise the known-answer examples above
# over whole equivalence classes of chunks rather than single fixtures.
# ===========================================================================


# chunk_duration_s ----------------------------------------------------------


@given(n_steps=st.integers(min_value=0, max_value=64), dt_ms=st.floats(0.1, 1000.0))
@settings(max_examples=200, deadline=None)
def test_duration_is_always_steps_times_dt(n_steps, dt_ms):
    """For any positive dt, duration is exactly n_steps * dt / 1000."""
    chunk = ActionChunk(joints={"0@ur5e": [[0.0] * 6] * n_steps}, dt_ms=dt_ms)
    assert chunk_duration_s(chunk) == n_steps * dt_ms / 1000.0


@given(n_steps=st.integers(min_value=1, max_value=64), dt_ms=st.floats(-1000.0, 0.0))
@settings(max_examples=100, deadline=None)
def test_duration_is_zero_whenever_dt_is_unset(n_steps, dt_ms):
    """A non-positive dt means 'no timeline' regardless of step count."""
    chunk = ActionChunk(joints={"0@ur5e": [[0.0] * 6] * n_steps}, dt_ms=dt_ms)
    assert chunk_duration_s(chunk) == 0.0


# trim_chunk ----------------------------------------------------------------


@given(steps=_STEPS, n=st.integers(min_value=1, max_value=30))
@settings(max_examples=200, deadline=None)
def test_trim_bounds_each_arm_to_n_and_preserves_a_prefix(steps, n):
    """Trimming to n>0 yields min(len, n) steps that are an exact prefix."""
    chunk = ActionChunk(joints={"0@ur5e": steps}, dt_ms=33.0, first_timestamp_ms=7)
    trimmed = trim_chunk(chunk, n)
    out = trimmed.joints["0@ur5e"]
    assert len(out) == min(len(steps), n)
    assert out == steps[: len(out)]  # a genuine prefix, never reordered


@given(steps=_STEPS, n=st.integers(min_value=1, max_value=30))
@settings(max_examples=200, deadline=None)
def test_trim_never_mutates_chunk_metadata(steps, n):
    """Trimming touches only the step lists; dt/start/ios/seam are unchanged."""
    chunk = ActionChunk(
        joints={"0@ur5e": steps},
        ios={"0@ur5e": {"do[0]": True}},
        dt_ms=33.0,
        first_timestamp_ms=7,
        seam_backdate_steps=2,
    )
    trimmed = trim_chunk(chunk, n)
    assert trimmed.dt_ms == chunk.dt_ms
    assert trimmed.first_timestamp_ms == chunk.first_timestamp_ms
    assert trimmed.seam_backdate_steps == chunk.seam_backdate_steps
    assert trimmed.ios == chunk.ios


@given(steps=_STEPS, n=st.integers(min_value=-5, max_value=0))
@settings(max_examples=50, deadline=None)
def test_trim_with_non_positive_n_is_identity(steps, n):
    """n <= 0 means 'execute all' — the exact same chunk object is returned."""
    chunk = ActionChunk(joints={"0@ur5e": steps}, dt_ms=33.0)
    assert trim_chunk(chunk, n) is chunk


# apply_relative_mode -------------------------------------------------------


@given(
    start=st.lists(_J, min_size=6, max_size=6),
    deltas=st.lists(_STEP, min_size=1, max_size=12),
)
@settings(max_examples=200, deadline=None)
def test_relative_joint_steps_are_the_running_cumulative_sum(start, deltas):
    """Each absolute step equals the start pose plus the cumulative deltas."""
    state = SimpleNamespace(joints=list(start))
    chunk = ActionChunk(joints={"0@ur5e": deltas})
    out = apply_relative_mode(chunk, {"0@ur5e": state}, relative_mgs=["0@ur5e"]).joints["0@ur5e"]

    running = list(start)
    for produced, delta in zip(out, deltas, strict=True):
        running = [r + d for r, d in zip(running, delta, strict=True)]
        assert produced == pytest.approx(running)


@given(steps=st.lists(_STEP, min_size=1, max_size=10))
@settings(max_examples=100, deadline=None)
def test_relative_mode_with_no_groups_is_identity(steps):
    """With no relative motion groups the chunk is returned untouched."""
    chunk = ActionChunk(joints={"0@ur5e": steps})
    assert apply_relative_mode(chunk, {}, relative_mgs=[]) is chunk


# placement_start_ms --------------------------------------------------------


@given(
    explicit=st.integers(min_value=0, max_value=100_000),
    rate=st.floats(-5.0, 60.0),
    elapsed=st.integers(min_value=0, max_value=100_000),
    backdate=st.integers(min_value=0, max_value=2000),
)
@settings(max_examples=200, deadline=None)
def test_an_explicit_start_time_always_wins(explicit, rate, elapsed, backdate):
    """A first_timestamp_ms the policy set (>= 0) is used verbatim in every mode."""
    chunk = ActionChunk(joints={"0@ur5e": [[0.0] * 6]}, first_timestamp_ms=explicit)
    placed = placement_start_ms(
        chunk, policy_rate_hz=rate, session_elapsed_ms=elapsed, backdate_ms=backdate
    )
    assert placed == explicit


@given(
    rate=st.floats(-5.0, -0.001),
    elapsed=st.integers(min_value=0, max_value=100_000),
    backdate=st.integers(min_value=0, max_value=2000),
)
@settings(max_examples=100, deadline=None)
def test_wait_for_chunk_mode_always_places_relative(rate, elapsed, backdate):
    """Sequential mode (rate < 0) with no explicit start always places at -1 ('now')."""
    chunk = ActionChunk(joints={"0@ur5e": [[0.0] * 6]})  # first_timestamp_ms defaults to -1
    placed = placement_start_ms(
        chunk, policy_rate_hz=rate, session_elapsed_ms=elapsed, backdate_ms=backdate
    )
    assert placed == -1


@given(
    rate=st.floats(0.0, 60.0),
    elapsed=st.integers(min_value=0, max_value=100_000),
    backdate=st.integers(min_value=0, max_value=2000),
)
@settings(max_examples=200, deadline=None)
def test_overlapping_mode_anchors_at_send_time_minus_backdate_clamped(rate, elapsed, backdate):
    """Overlapping mode (rate >= 0) is elapsed - backdate, never negative."""
    chunk = ActionChunk(joints={"0@ur5e": [[0.0] * 6]})
    placed = placement_start_ms(
        chunk, policy_rate_hz=rate, session_elapsed_ms=elapsed, backdate_ms=backdate
    )
    assert placed == max(0, elapsed - backdate)
    assert placed >= 0
