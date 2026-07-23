"""Tests for RTC helpers (pure functions, numpy-only — no ZMQ/server)."""

from __future__ import annotations

import numpy as np

from novapolicy.gr00t.rtc import (
    RTCConfig,
    RTCState,
    compute_rtc_options,
    detect_action_horizon,
    seam_backdate_steps,
)


def test_detect_action_horizon_none_when_no_array():
    assert detect_action_horizon({"x": 1.0, "y": [1, 2, 3]}) is None


def test_seam_backdate_is_executed_minus_consumed_head_in_steady_state():
    """In steady state (executed <= H) backdate = executed - (H - overlap)."""
    state = RTCState(action_horizon=16, last_executed_steps=6, last_overlap_steps=12)
    # 6 - 16 + 12 = 2
    assert seam_backdate_steps(state) == 2


def test_seam_backdate_is_capped_at_overlap_under_starvation():
    """If the robot ran past the previous chunk (executed > H), cap at overlap.

    Without the cap the anchor would slide so far into the past that the whole
    chunk is discarded as stale and the robot stalls.
    """
    state = RTCState(action_horizon=16, last_executed_steps=30, last_overlap_steps=12)
    # raw = 30 - 16 + 12 = 26, capped to overlap=12
    assert seam_backdate_steps(state) == 12


def test_seam_backdate_floors_at_zero_when_robot_is_behind_the_reused_head():
    """A negative raw value (robot not yet in the reused head) floors at 0."""
    state = RTCState(action_horizon=16, last_executed_steps=1, last_overlap_steps=4)
    # raw = 1 - 16 + 4 = -11 -> 0
    assert seam_backdate_steps(state) == 0


def test_seam_backdate_is_zero_before_horizon_is_known():
    assert seam_backdate_steps(RTCState()) == 0


def test_compute_rtc_options_does_not_mutate_the_latency_queue():
    """compute_rtc_options is pure w.r.t. latency sampling.

    The client owns the latency queue (it appends real measurements). This
    helper must only read an estimate it is handed — appending here would
    double-count and bias frozen_steps (regression: it used to push
    ``avg + offset`` back into the queue on every call).
    """
    cfg = RTCConfig()
    state = RTCState(action_horizon=16)
    state.latency_queue.extend([0.05, 0.06, 0.07])
    before = list(state.latency_queue)
    compute_rtc_options(cfg, state, inference_latency=0.1, dt_ms=33.0)
    assert list(state.latency_queue) == before


# ===========================================================================
# Property-based invariants over horizon, latency, dt, and overlap-factor ranges.
# ===========================================================================

from hypothesis import (  # noqa: E402
    given,
    settings,
    strategies as st,
)

_HORIZON = st.integers(min_value=1, max_value=64)
_LATENCY = st.floats(min_value=0.0, max_value=2.0, allow_nan=False, allow_infinity=False)
_DT = st.floats(min_value=1.0, max_value=500.0, allow_nan=False, allow_infinity=False)
_FACTOR = st.floats(min_value=0.0, max_value=1.0, allow_nan=False, allow_infinity=False)


@given(horizon=_HORIZON, latency=_LATENCY, dt_ms=_DT, factor=_FACTOR, ramp=st.floats(0.1, 10.0))
@settings(max_examples=300, deadline=None)
def test_rtc_options_are_always_clamped_to_a_valid_range(horizon, latency, dt_ms, factor, ramp):
    """overlap in [0, H], frozen in [0, overlap], and H is echoed back — always."""
    cfg = RTCConfig(max_overlap_factor=factor, ramp_rate=ramp)
    state = RTCState(action_horizon=horizon)
    opts = compute_rtc_options(cfg, state, inference_latency=latency, dt_ms=dt_ms)
    assert opts is not None
    overlap = opts["rtc_overlap_steps"]
    frozen = opts["rtc_frozen_steps"]
    assert opts["action_horizon"] == horizon
    assert 0 <= overlap <= horizon
    assert 0 <= frozen <= overlap
    # The seam-backdate state mirrors the returned overlap exactly.
    assert state.last_overlap_steps == overlap


@given(latency=_LATENCY, dt_ms=_DT)
@settings(max_examples=100, deadline=None)
def test_rtc_options_are_none_until_the_horizon_is_known(latency, dt_ms):
    """With no detected horizon there is no previous action to reuse — None."""
    cfg = RTCConfig()
    state = RTCState()  # action_horizon is None
    assert compute_rtc_options(cfg, state, inference_latency=latency, dt_ms=dt_ms) is None


@given(
    shape=st.lists(st.integers(min_value=1, max_value=8), min_size=2, max_size=5),
    dof_last=st.integers(min_value=1, max_value=16),
)
@settings(max_examples=200, deadline=None)
def test_detect_horizon_is_the_second_to_last_axis(shape, dof_last):
    """For any array with ndim >= 2 the horizon is shape[-2]."""
    shape = [*shape[:-1], dof_last]
    arr = np.zeros(tuple(shape), dtype=np.float32)
    assert detect_action_horizon({"action.x": arr}) == shape[-2]


@given(values=st.lists(st.integers(min_value=-100, max_value=100), min_size=0, max_size=8))
@settings(max_examples=100, deadline=None)
def test_detect_horizon_ignores_scalars_and_1d_arrays(values):
    """Scalars and 1-D arrays carry no temporal axis → None."""
    arr = np.array(values, dtype=np.float32)  # 1-D (or empty)
    assert detect_action_horizon({"state": arr, "scalar": np.float32(1.0)}) is None
