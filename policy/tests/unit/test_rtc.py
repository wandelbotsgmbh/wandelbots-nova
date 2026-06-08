"""Tests for RTC helpers (pure functions, numpy-only — no ZMQ/server)."""

from __future__ import annotations

import numpy as np

from policy.gr00t.rtc import RTCConfig, RTCState, compute_rtc_options, detect_action_horizon


def test_detect_action_horizon_batched():
    """GR00T arrays are (batch=1, time=T, dof); horizon is the time axis."""
    action = {"action.left_arm": np.zeros((1, 16, 6), dtype=np.float32)}
    assert detect_action_horizon(action) == 16


def test_detect_action_horizon_unbatched():
    """An un-batched (T, dof) array still reports T via shape[-2]."""
    action = {"action.arm": np.zeros((8, 6), dtype=np.float32)}
    assert detect_action_horizon(action) == 8


def test_detect_action_horizon_skips_scalars_and_1d():
    action = {"meta": np.zeros((4,), dtype=np.float32), "action.arm": np.zeros((1, 12, 6))}
    assert detect_action_horizon(action) == 12


def test_detect_action_horizon_none_when_no_array():
    assert detect_action_horizon({"x": 1.0, "y": [1, 2, 3]}) is None


def test_compute_rtc_options_none_before_horizon_known():
    cfg = RTCConfig()
    state = RTCState()  # action_horizon is None
    assert compute_rtc_options(cfg, state, inference_latency=0.1, dt_ms=33.0) is None


def test_compute_rtc_options_clamps_within_horizon():
    cfg = RTCConfig(max_overlap_factor=0.75)
    state = RTCState(action_horizon=16)
    opts = compute_rtc_options(cfg, state, inference_latency=0.1, dt_ms=33.0)
    assert opts is not None
    assert opts["action_horizon"] == 16
    assert 0 <= opts["rtc_overlap_steps"] <= 16
    assert 0 <= opts["rtc_frozen_steps"] <= opts["rtc_overlap_steps"]
    # State is updated for the executor's seam backdate.
    assert state.last_overlap_steps == opts["rtc_overlap_steps"]
