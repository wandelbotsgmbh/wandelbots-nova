"""Tests for the pure action-chunk transforms in :mod:`policy.chunking`.

These are stateless functions the executor leans on for receding-horizon
trimming, relative→absolute conversion, chunk timing, and where each chunk
gets placed on the jogging timeline. They have no I/O, so they are tested
directly with plain data.
"""

from __future__ import annotations

from types import SimpleNamespace

from policy.chunking import (
    apply_relative_mode,
    chunk_duration_s,
    placement_start_ms,
    trim_chunk,
)
from policy.types import ActionChunk

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
    """A start_time_ms the policy set explicitly is used verbatim."""
    chunk = ActionChunk(joints={"0@ur5e": [[0.0]]}, start_time_ms=1234)
    assert (
        placement_start_ms(chunk, policy_rate_hz=20, session_elapsed_ms=999, backdate_ms=50) == 1234
    )


def test_wait_for_chunk_uses_relative_placement():
    """Sequential mode (policy_rate_hz < 0) places chunks relative ('now') = -1."""
    chunk = ActionChunk(joints={"0@ur5e": [[0.0]]})  # start_time_ms defaults to -1
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
