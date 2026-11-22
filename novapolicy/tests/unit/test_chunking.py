"""Tests for the pure action-chunk transforms in :mod:`novapolicy.chunking`.

These are stateless functions the executor leans on for receding-horizon
trimming, relative→absolute conversion, chunk timing, and where each chunk
gets placed on the jogging timeline. They have no I/O, so they are tested
directly with plain data.
"""

from __future__ import annotations

import math
from types import SimpleNamespace

from hypothesis import given, settings, strategies as st
import numpy as np
import pytest
from scipy.spatial.transform import Rotation

from novapolicy.chunking import (
    TcpDelta,
    apply_relative_mode,
    bridge_steps,
    compose_tcp_deltas,
    placement,
    ramp_steps,
    smooth_steps,
    trim_chunk,
    unwrap_tcp_rotations,
)
from novapolicy.types import ActionChunk, ContinuousExecution, SequentialExecution

# Strategies for generated chunks --------------------------------------------

_J = st.floats(min_value=-10.0, max_value=10.0, allow_nan=False, allow_infinity=False)
_STEP = st.lists(_J, min_size=6, max_size=6)
_STEPS = st.lists(_STEP, min_size=0, max_size=20)

# ---------------------------------------------------------------------------
# trim_chunk — receding horizon: keep only the first n steps
# ---------------------------------------------------------------------------


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
# smooth_steps — filter mutable motion lookaheads
# ---------------------------------------------------------------------------


def test_smoothing_filters_motion_and_keeps_the_retained_prefix():
    steps = [[0.0] * 6, [4.0] * 6, [0.0] * 6]
    smoothed = smooth_steps(steps, retained_prefix_steps=1)
    assert [step[0] for step in smoothed] == pytest.approx([0.0, 1.5, 1.25])
    assert steps == [[0.0] * 6, [4.0] * 6, [0.0] * 6]  # not mutated
    assert [step[0] for step in smooth_steps(steps, passes=1)] == pytest.approx([1.0, 2.0, 1.0])


@pytest.mark.parametrize(
    ("kwargs", "message"),
    [({"passes": -1}, "passes"), ({"retained_prefix_steps": -1}, "retained_prefix_steps")],
)
def test_smoothing_rejects_negative_configuration(kwargs, message):
    with pytest.raises(ValueError, match=message):
        smooth_steps([[0.0]], **kwargs)


# ---------------------------------------------------------------------------
# ramp_steps — allocate time for acceleration and braking
# ---------------------------------------------------------------------------


def test_endpoint_ramps_preserve_original_waypoints_and_remap_indices():
    steps, indices = ramp_steps([[0.0], [1.0], [2.0], [3.0]], interpolation_steps=3)
    assert [step[0] for step in steps] == pytest.approx([
        0.0,
        1 / 9,
        4 / 9,
        1.0,
        2.0,
        23 / 9,
        26 / 9,
        3.0,
    ])
    assert indices == (0, 3, 4, 7)


def test_single_interval_uses_smoothstep_when_accelerating_and_braking():
    steps, indices = ramp_steps([[0.0], [1.0]], interpolation_steps=4)
    assert [step[0] for step in steps] == pytest.approx([0.0, 0.15625, 0.5, 0.84375, 1.0])
    assert indices == (0, 4)


def test_ramp_interpolation_steps_must_be_at_least_two():
    with pytest.raises(ValueError, match="at least 2"):
        ramp_steps([], interpolation_steps=1)


# ---------------------------------------------------------------------------
# bridge_steps — connect the measured position to the first policy step
# ---------------------------------------------------------------------------


def test_bridge_uses_policy_spacing_and_ends_at_first_joint_waypoint():
    bridge = bridge_steps([0.0, 0.0], [[3.0, 0.0], [4.0, 0.0], [5.0, 0.0]])
    assert bridge == [[0.0, 0.0], [1.0, 0.0], [2.0, 0.0], [3.0, 0.0]]


def test_bridge_is_omitted_when_first_waypoint_is_within_normal_spacing():
    assert bridge_steps([0.0, 0.0], [[0.5, 0.0], [1.5, 0.0]]) == []


def test_always_anchored_bridge_holds_current_state_before_a_near_waypoint():
    bridge = bridge_steps([0.0, 0.0], [[0.5, 0.0], [1.5, 0.0]], always_anchor=True)
    assert bridge == [[0.0, 0.0], [0.5, 0.0]]


def test_bridge_supports_tcp_translation_without_mixing_rotation_units():
    bridge = bridge_steps(
        [0.0] * 6,
        [[30.0, 0.0, 0.0, 0.3, 0.0, 0.0], [40.0, 0.0, 0.0, 0.4, 0.0, 0.0]],
        tcp=True,
    )
    expected = [
        [0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
        [10.0, 0.0, 0.0, 0.1, 0.0, 0.0],
        [20.0, 0.0, 0.0, 0.2, 0.0, 0.0],
        [30.0, 0.0, 0.0, 0.3, 0.0, 0.0],
    ]
    for actual_step, expected_step in zip(bridge, expected, strict=True):
        assert actual_step == pytest.approx(expected_step)


def _tool_down_flip() -> tuple[list[float], list[list[float]]]:
    """Tool pointing down: measured just under pi about +y, policy just over it.

    The policy steps are written the way a rotation library returns them, as
    angles under pi about -y. They differ from the measured orientation by under
    a degree, but component-wise they are about 2 pi apart.
    """
    current = [0.0, 0.0, 0.0, 0.0, math.pi - 0.002, 0.0]
    steps = [
        [1.0, 0.0, 0.0, 0.0, -(math.pi - 0.004), 0.0],
        [2.0, 0.0, 0.0, 0.0, -(math.pi - 0.006), 0.0],
        [3.0, 0.0, 0.0, 0.0, -(math.pi - 0.008), 0.0],
    ]
    return current, steps


def test_bridge_does_not_swing_the_wrist_across_a_rotation_vector_flip():
    """Regression: a sign flip near pi once bridged through the identity in 573 steps."""
    from novapolicy.jogging.clock import ServerMs
    from novapolicy.jogging.modules import Bridge
    from novapolicy.jogging.pipeline import ModuleContext
    from novapolicy.jogging.waypoints import PendingChunk

    current, policy = _tool_down_flip()
    chunk = PendingChunk(steps=policy, dt_ms=100.0, caller_step_count=3)
    context = ModuleContext(now_ms=ServerMs(0), position=current, step_ms=50.0, mode="cartesian")
    steps = Bridge(always_anchor=True).on_chunk(chunk, context).steps
    assert len(steps) < 10  # a short bridge, not the 573 steps of the incident
    for step in steps:
        # Every waypoint stays a near-half-turn about y: no pass through the identity.
        assert math.hypot(*step[3:6]) == pytest.approx(math.pi, abs=0.01)
        assert step[4] > 0  # unwrapped onto the measured side
    assert steps[-1][4] == pytest.approx(math.pi + 0.008)


def test_unwrap_keeps_equivalent_rotations_and_leaves_continuous_ones_alone():
    unwrapped = unwrap_tcp_rotations([[0.0, 0.0, 0.0, 0.0, -(math.pi - 0.1), 0.0]], [0.0, 3.1, 0.0])
    assert unwrapped[0][3:6] == pytest.approx([0.0, math.pi + 0.1, 0.0])

    steps = [[5.0, 6.0, 7.0, 0.1, 0.2, 0.3], [5.0, 6.0, 7.0, 0.11, 0.2, 0.3]]
    for actual, expected in zip(unwrap_tcp_rotations(steps, [0.1, 0.2, 0.3]), steps, strict=True):
        assert actual == pytest.approx(expected)


def test_the_identity_after_unwrapped_steps_stays_a_full_turn():
    """Steps unwrapped past pi that reach the identity become the full turn, not a jump to 0."""
    steps = [[0.0] * 3 + [0.0, 0.0, -3.0], [0.0] * 6]
    unwrapped = unwrap_tcp_rotations(steps, [0.0, 0.0, 3.2])
    assert unwrapped[0][3:6] == pytest.approx([0.0, 0.0, 2.0 * math.pi - 3.0])
    assert unwrapped[1][3:6] == pytest.approx([0.0, 0.0, 2.0 * math.pi])
    assert unwrap_tcp_rotations([[0.0] * 6], [0.0, 0.0, 0.5])[0][3:6] == [0.0, 0.0, 0.0]


def test_endpoint_ramps_do_not_interpolate_across_a_rotation_vector_flip():
    current, policy = _tool_down_flip()
    ramped, _ = ramp_steps([current, *policy], tcp=True)
    for step in ramped:
        assert math.hypot(*step[3:6]) == pytest.approx(math.pi, abs=0.01)


def test_bridge_requires_at_least_two_policy_waypoints_for_spacing():
    assert bridge_steps([0.0], [[10.0]]) == []


# ---------------------------------------------------------------------------
# apply_relative_mode — convert delta targets to absolute positions
# ---------------------------------------------------------------------------


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


# ===========================================================================
# Property-based invariants — these generalise the known-answer examples above
# over whole equivalence classes of chunks rather than single fixtures.
# ===========================================================================


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
    """Trimming touches only the step lists; dt/start/ios are unchanged."""
    chunk = ActionChunk(
        joints={"0@ur5e": steps},
        ios={"0@ur5e": {"do[0]": True}},
        dt_ms=33.0,
        first_timestamp_ms=7,
    )
    trimmed = trim_chunk(chunk, n)
    assert trimmed.dt_ms == chunk.dt_ms
    assert trimmed.first_timestamp_ms == chunk.first_timestamp_ms
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


@given(explicit=st.integers(min_value=0, max_value=100_000))
def test_an_explicit_start_time_always_wins(explicit):
    """A first_timestamp_ms the policy set (>= 0) becomes an exact anchor in every mode."""
    chunk = ActionChunk(joints={"0@ur5e": [[0.0] * 6]}, first_timestamp_ms=explicit)
    for execution in (
        SequentialExecution(),
        ContinuousExecution(),
        ContinuousExecution(rate_hz=20),
    ):
        place = placement(chunk, execution=execution)
        assert place.first_timestamp_ms == explicit
        assert place.timestamp_offset_steps == 0


def test_sequential_execution_uses_the_same_lead_rule():
    """One rule for both modes; the session dates a sequential chunk from its send."""
    chunk = ActionChunk(joints={"0@ur5e": [[0.0] * 6] * 4}, dt_ms=1000 / 30)
    for from_observation in (True, False):
        place = placement(chunk, execution=SequentialExecution(), from_observation=from_observation)
        assert place.lead
        assert place.first_timestamp_ms is None


def test_a_chunk_answering_an_observation_is_dated_by_it():
    """One step or many: a continuous chunk from the latest observation is placed by the
    session, a fixed lead after that observation."""
    for steps in ([[0.0] * 6], [[0.0] * 6] * 8):
        chunk = ActionChunk(joints={"0@ur5e": steps}, dt_ms=1000 / 30)
        place = placement(chunk, execution=ContinuousExecution(), from_observation=True)
        assert place.lead
        assert place.first_timestamp_ms is None
        assert not placement(chunk, execution=ContinuousExecution()).lead


def test_a_chunk_on_its_own_timeline_is_not_dated_by_the_observation():
    """A policy timestep or a spacing-less multi-step chunk keeps the anchor it already has."""
    base = ActionChunk(joints={"0@ur5e": [[0.0] * 6] * 8}, dt_ms=1000 / 30)
    for chunk in (
        base.model_copy(update={"action_timestep": 4}),
        base.model_copy(update={"dt_ms": 0.0}),
    ):
        place = placement(chunk, execution=ContinuousExecution(), from_observation=True)
        assert not place.lead
        assert place.timestamp_offset_steps == 0


# compose_tcp_deltas ---------------------------------------------------------


_DOWN = [-100.0, -500.0, 300.0, math.pi, 0.0, 0.0]  # tool pointing straight down


def _rot(pose: list[float]) -> Rotation:
    return Rotation.from_rotvec(pose[3:6])


@pytest.mark.parametrize("width", [7, 9])
def test_relative_tcp_offsets_of_another_width_are_refused(width):
    """A quaternion (7) or rot6d (9) offset read as a rotation vector would turn the tool."""
    with pytest.raises(ValueError, match="6 values"):
        compose_tcp_deltas(_DOWN, [[0.0] * width], TcpDelta())


def test_relative_tcp_rotations_compose_instead_of_adding():
    """A yaw offset on a tool-down pose turns the tool about base z, not about a summed vector."""
    yaw = math.radians(10.0)
    out = compose_tcp_deltas(_DOWN, [[0.0, 0.0, 0.0, 0.0, 0.0, yaw]], TcpDelta())
    expected = Rotation.from_rotvec([0.0, 0.0, yaw]) * _rot(_DOWN)
    assert (_rot(out[0]) * expected.inv()).magnitude() == pytest.approx(0.0, abs=1e-9)
    assert out[0][:3] == pytest.approx(_DOWN[:3])  # rotating about the TCP keeps it in place


def test_accumulated_tcp_deltas_chain_from_step_to_step():
    out = compose_tcp_deltas(
        _DOWN, [[1.0, 0.0, -2.0, 0.0, 0.0, 0.0]] * 3, TcpDelta(accumulate=True)
    )
    assert [s[0] for s in out] == pytest.approx([-99.0, -98.0, -97.0])
    assert [s[2] for s in out] == pytest.approx([298.0, 296.0, 294.0])
    offsets = compose_tcp_deltas(_DOWN, [[1.0, 0.0, -2.0, 0.0, 0.0, 0.0]] * 3, TcpDelta())
    assert [s[0] for s in offsets] == pytest.approx([-99.0] * 3)  # the default does not chain


def test_tool_frame_tcp_deltas_follow_the_tool_axes():
    out = compose_tcp_deltas(
        _DOWN, [[0.0, 0.0, 10.0, 0.0, 0.0, 0.0]], TcpDelta(use_tool_coordinate_system=True)
    )
    assert out[0][:3] == pytest.approx([-100.0, -500.0, 290.0])  # tool z points down


def test_tcp_deltas_stay_continuous_near_a_half_turn():
    start = [*_DOWN[:3], 0.0, -math.pi, 0.0]
    steps = [[0.0, 0.0, 0.0, 0.0, 0.0, math.radians(2.0)]] * 10
    out = compose_tcp_deltas(start, steps, TcpDelta(accumulate=True))
    rotvecs = np.asarray([s[3:] for s in out])
    assert np.abs(np.diff(rotvecs, axis=0)).max() < 0.1  # no sign flip between steps


def test_computed_action_values_survive_trimming_and_relative_targets():
    """Values for Action.computed hooks travel with the chunk through every rebuild."""
    rows = [[float(i)] for i in range(6)]
    chunk = ActionChunk(tcp={"mg": [[0.0] * 6] * 6}, values={"gripper": rows}, dt_ms=33.0)
    assert trim_chunk(chunk, 4).values == {"gripper": rows[:4]}
    pose = SimpleNamespace(position=[1.0, 2.0, 3.0], orientation=[0.0, 0.0, 0.0])
    resolved = apply_relative_mode(chunk, {"mg": SimpleNamespace(joints=[], pose=pose)}, ["mg"])
    assert resolved.values == {"gripper": rows}
