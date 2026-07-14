"""Tests for the pure action-chunk transforms in :mod:`novapolicy.chunking`.

These are stateless functions the executor leans on for receding-horizon
trimming, relative→absolute conversion, chunk timing, and where each chunk
gets placed on the jogging timeline. They have no I/O, so they are tested
directly with plain data.
"""

from __future__ import annotations

from types import SimpleNamespace

from hypothesis import given, settings, strategies as st
import pytest

from novapolicy.chunking import (
    NOW,
    apply_relative_mode,
    chunk_duration_s,
    connect_action_chunk,
    create_bridge_chunk,
    interpolate_action_chunk_ramps,
    placement,
    trim_chunk,
)
from novapolicy.types import ActionChunk

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
# interpolate_action_chunk_ramps — allocate time for acceleration and braking
# ---------------------------------------------------------------------------


def test_endpoint_ramps_preserve_original_waypoints_and_remap_indices():
    chunk = ActionChunk(
        joints={"0@ur5e": [[0.0], [1.0], [2.0], [3.0]]},
        dt_ms=50.0,
    )

    interpolated = interpolate_action_chunk_ramps(chunk, interpolation_steps=3)

    assert [step[0] for step in interpolated.motion.joints["0@ur5e"]] == pytest.approx(
        [0.0, 1 / 9, 4 / 9, 1.0, 2.0, 23 / 9, 26 / 9, 3.0]
    )
    assert interpolated.original_step_indices["0@ur5e"] == (0, 3, 4, 7)
    assert interpolated.motion.dt_ms == 50.0


def test_single_interval_uses_smoothstep_when_accelerating_and_braking():
    chunk = ActionChunk(joints={"0@ur5e": [[0.0], [1.0]]}, dt_ms=50.0)

    interpolated = interpolate_action_chunk_ramps(chunk, interpolation_steps=4)

    assert [step[0] for step in interpolated.motion.joints["0@ur5e"]] == pytest.approx(
        [0.0, 0.15625, 0.5, 0.84375, 1.0]
    )
    assert interpolated.original_step_indices["0@ur5e"] == (0, 4)


def test_ramp_interpolation_steps_must_be_at_least_two():
    with pytest.raises(ValueError, match="at least 2"):
        interpolate_action_chunk_ramps(ActionChunk(), interpolation_steps=1)


# ---------------------------------------------------------------------------
# create_bridge_chunk — connect current state to policy step zero
# ---------------------------------------------------------------------------


def test_bridge_uses_policy_spacing_and_ends_at_first_joint_waypoint():
    state = SimpleNamespace(joints=[0.0, 0.0])
    chunk = ActionChunk(
        joints={"0@ur5e": [[3.0, 0.0], [4.0, 0.0], [5.0, 0.0]]},
        ios={"0@ur5e": {"do[0]": True}},
        dt_ms=50.0,
    )

    bridge = create_bridge_chunk(chunk, {"0@ur5e": state})

    assert bridge is not None
    assert bridge.joints["0@ur5e"] == [
        [0.0, 0.0],
        [1.0, 0.0],
        [2.0, 0.0],
        [3.0, 0.0],
    ]
    assert bridge.dt_ms == 50.0
    assert bridge.ios is None


def test_connected_chunk_has_no_duplicate_policy_boundary_or_ios():
    state = SimpleNamespace(joints=[0.0, 0.0])
    chunk = ActionChunk(
        joints={"0@ur5e": [[3.0, 0.0], [4.0, 0.0], [5.0, 0.0]]},
        ios={"0@ur5e": {"do[0]": True}},
        dt_ms=50.0,
    )

    connected = connect_action_chunk(chunk, {"0@ur5e": state})

    assert connected is not None
    assert connected.motion.joints["0@ur5e"] == [
        [0.0, 0.0],
        [1.0, 0.0],
        [2.0, 0.0],
        [3.0, 0.0],
        [4.0, 0.0],
        [5.0, 0.0],
    ]
    assert connected.policy_start_steps == {"0@ur5e": 3}
    assert connected.motion.ios is None
    assert connected.motion.dt_ms == 50.0


def test_bridge_is_omitted_when_first_waypoint_is_within_normal_spacing():
    state = SimpleNamespace(joints=[0.0, 0.0])
    chunk = ActionChunk(joints={"0@ur5e": [[0.5, 0.0], [1.5, 0.0]]}, dt_ms=50.0)

    assert create_bridge_chunk(chunk, {"0@ur5e": state}) is None


def test_always_anchored_bridge_holds_current_state_before_a_near_waypoint():
    state = SimpleNamespace(joints=[0.0, 0.0])
    chunk = ActionChunk(joints={"0@ur5e": [[0.5, 0.0], [1.5, 0.0]]}, dt_ms=50.0)

    connected = connect_action_chunk(
        chunk,
        {"0@ur5e": state},
        always_anchor=True,
    )

    assert connected is not None
    assert connected.bridge.joints["0@ur5e"] == [[0.0, 0.0], [0.5, 0.0]]
    assert connected.motion.joints["0@ur5e"] == [
        [0.0, 0.0],
        [0.5, 0.0],
        [1.5, 0.0],
    ]
    assert connected.policy_start_steps == {"0@ur5e": 1}


def test_bridge_supports_tcp_translation_without_mixing_rotation_units():
    pose = SimpleNamespace(position=[0.0, 0.0, 0.0], orientation=[0.0, 0.0, 0.0])
    state = SimpleNamespace(pose=pose)
    chunk = ActionChunk(
        tcp={
            "0@ur5e": [
                [30.0, 0.0, 0.0, 0.3, 0.0, 0.0],
                [40.0, 0.0, 0.0, 0.4, 0.0, 0.0],
            ]
        },
        dt_ms=20.0,
    )

    bridge = create_bridge_chunk(chunk, {"0@ur5e": state})

    assert bridge is not None
    expected = [
        [0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
        [10.0, 0.0, 0.0, 0.1, 0.0, 0.0],
        [20.0, 0.0, 0.0, 0.2, 0.0, 0.0],
        [30.0, 0.0, 0.0, 0.3, 0.0, 0.0],
    ]
    for actual_step, expected_step in zip(bridge.tcp["0@ur5e"], expected, strict=True):
        assert actual_step == pytest.approx(expected_step)


def test_bridge_requires_at_least_two_policy_waypoints_for_spacing():
    state = SimpleNamespace(joints=[0.0])
    chunk = ActionChunk(joints={"0@ur5e": [[10.0]]}, dt_ms=50.0)

    assert create_bridge_chunk(chunk, {"0@ur5e": state}) is None


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
# placement — how a chunk is anchored on the session timeline
# ---------------------------------------------------------------------------


def test_explicit_start_time_wins_and_is_anchored_exactly():
    """A first_timestamp_ms the policy set explicitly becomes an exact anchor."""
    chunk = ActionChunk(joints={"0@ur5e": [[0.0]]}, first_timestamp_ms=1234)
    place = placement(chunk, policy_rate_hz=20)
    assert place.anchor_ms == 1234
    assert place.anchor_offset_steps == 0


def test_wait_for_chunk_anchors_at_now_one_step_ahead():
    """Sequential mode (policy_rate_hz < 0) anchors at 'now' (NOW), one step ahead."""
    chunk = ActionChunk(joints={"0@ur5e": [[0.0]]})  # first_timestamp_ms defaults to -1
    place = placement(chunk, policy_rate_hz=-1)
    assert place.anchor_ms == NOW
    assert place.anchor_offset_steps == 1


def test_overlapping_mode_anchors_at_now_backdated_by_the_seam():
    """Overlapping mode (rate >= 0) anchors at 'now' backdated by the seam steps."""
    chunk = ActionChunk(joints={"0@ur5e": [[0.0]]}, seam_backdate_steps=8)
    place = placement(chunk, policy_rate_hz=20)
    assert place.anchor_ms == NOW
    assert place.anchor_offset_steps == -8


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


# placement -----------------------------------------------------------------


@given(
    explicit=st.integers(min_value=0, max_value=100_000),
    rate=st.floats(-5.0, 60.0),
)
@settings(max_examples=200, deadline=None)
def test_an_explicit_start_time_always_wins(explicit, rate):
    """A first_timestamp_ms the policy set (>= 0) becomes an exact anchor in every mode."""
    chunk = ActionChunk(joints={"0@ur5e": [[0.0] * 6]}, first_timestamp_ms=explicit)
    place = placement(chunk, policy_rate_hz=rate)
    assert place.anchor_ms == explicit
    assert place.anchor_offset_steps == 0


@given(rate=st.floats(-5.0, -0.001))
@settings(max_examples=100, deadline=None)
def test_wait_for_chunk_mode_always_anchors_at_now_one_step_ahead(rate):
    """Sequential mode (rate < 0) with no explicit start anchors at NOW, one step ahead."""
    chunk = ActionChunk(joints={"0@ur5e": [[0.0] * 6]})  # first_timestamp_ms defaults to -1
    place = placement(chunk, policy_rate_hz=rate)
    assert place.anchor_ms == NOW
    assert place.anchor_offset_steps == 1


@given(
    rate=st.floats(0.0, 60.0),
    seam=st.integers(min_value=0, max_value=64),
)
@settings(max_examples=200, deadline=None)
def test_overlapping_mode_anchors_at_now_backdated_by_the_seam_property(rate, seam):
    """Overlapping mode (rate >= 0) anchors at NOW, backdated by the seam steps."""
    chunk = ActionChunk(joints={"0@ur5e": [[0.0] * 6]}, seam_backdate_steps=seam)
    place = placement(chunk, policy_rate_hz=rate)
    assert place.anchor_ms == NOW
    assert place.anchor_offset_steps == -seam
