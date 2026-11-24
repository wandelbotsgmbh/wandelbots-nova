"""ServoHorizon: a spring-damper on live targets, sent as the spring's braking path (ADR 0022)."""

from __future__ import annotations

from itertools import pairwise
import math

import pytest

from novapolicy import ServoHorizon
from novapolicy.pipeline.base import ModuleContext
from novapolicy.transport.clock import ServerMs
from novapolicy.transport.waypoints import PendingChunk

START = [0.0, 0.0, 500.0, math.pi - 0.02, 0.0, 0.0]  # a tool pointing down, near the half-turn


def _context(mode: str = "cartesian") -> ModuleContext:
    return ModuleContext(now_ms=ServerMs(0), position=None, step_ms=50.0, mode=mode)


def _live(target: list[float], due_ms: float) -> PendingChunk:
    return PendingChunk(steps=[list(target)], dt_ms=0.0, first_timestamp_ms=ServerMs(round(due_ms)))


def _run(
    module: ServoHorizon, targets: list[list[float]], tick_ms: float = 10.0
) -> list[PendingChunk]:
    return [
        module.on_chunk(_live(t, 1_000 + k * tick_ms), _context()) for k, t in enumerate(targets)
    ]


def test_a_step_settles_on_the_target_without_overshoot() -> None:
    """Critically damped by default: a sudden 100 mm jump is approached, never passed."""
    target = [100.0, *START[1:]]
    out = _run(ServoHorizon(), [START] + [target] * 60)
    first = [chunk.steps[0][0] for chunk in out[1:]]
    assert max(first) <= 100.0 + 1e-6
    assert first[-1] == pytest.approx(100.0, abs=1.0)
    assert all(b >= a - 1e-9 for a, b in pairwise(first))  # moves one way only
    assert max(step[0] for step in out[1].steps) <= 100.0 + 1e-6  # the horizon brakes onto it


def test_waypoints_start_on_the_controller_cycle_and_are_evenly_spaced() -> None:
    module = ServoHorizon(step_ms=32.0, horizon_ms=320.0)
    for chunk in _run(module, [START] * 5, tick_ms=7.0):
        assert chunk.first_timestamp_ms is not None
        assert chunk.first_timestamp_ms % 8 == 0
        assert chunk.dt_ms == 32.0
        assert len(chunk.steps) == 11
        assert chunk.lead is False


def test_the_spring_keeps_to_the_limits() -> None:
    module = ServoHorizon(
        gain=2000.0, lookahead_s=0.045, linear_acceleration=2000.0, linear_speed=200.0
    )
    out = _run(module, [START] + [[500.0, *START[1:]]] * 80)
    path = [step[0] for step in out[-40].steps]  # mid-move: the horizon at 32 ms spacing
    speeds = [(b - a) / 0.032 for a, b in pairwise(path)]
    assert max(speeds) <= 200.0 + 1.0


def test_a_rotation_through_the_half_turn_goes_the_short_way() -> None:
    """Past pi the shortest rotation vector flips sign; the spring must not swing round."""
    beyond = [
        0.0,
        0.0,
        500.0,
        -(math.pi - 0.02),
        0.0,
        0.0,
    ]  # 2.3 deg further, written the short way
    out = _run(ServoHorizon(), [START] + [beyond] * 50)
    angles = [chunk.steps[0][3] for chunk in out]
    assert all(abs(b - a) < 0.01 for a, b in pairwise(angles))
    assert angles[-1] == pytest.approx(math.pi + 0.02, abs=1e-3)


def test_a_held_target_keeps_the_waypoints_still() -> None:
    out = _run(ServoHorizon(), [START] * 10)
    assert all(step == pytest.approx(START) for chunk in out for step in chunk.steps)


def test_a_chunk_of_several_steps_passes_through() -> None:
    chunk = PendingChunk(steps=[START, START], dt_ms=10.0, first_timestamp_ms=ServerMs(1_000))
    assert ServoHorizon().on_chunk(chunk, _context()) is chunk
    lead = PendingChunk(steps=[START], dt_ms=33.0, lead=True)
    assert ServoHorizon().on_chunk(lead, _context()) is lead


def test_joint_targets_use_the_angular_limits_per_joint() -> None:
    module = ServoHorizon(angular_speed=1.0)
    start, far = [0.0] * 6, [2.0] + [0.0] * 5
    out = [
        module.on_chunk(_live(t, 1_000 + k * 10), _context("joint"))
        for k, t in enumerate([start] + [far] * 60)
    ]
    path = [step[0] for step in out[30].steps]
    assert max((b - a) / 0.032 for a, b in pairwise(path)) <= 1.0 + 0.01


@pytest.mark.parametrize(
    "settings", [{"gain": 0.0}, {"lookahead_s": -0.1}, {"step_ms": 1.0}, {"horizon_ms": 10.0}]
)
def test_servo_horizon_refuses_settings_it_cannot_run(settings: dict) -> None:
    with pytest.raises(ValueError, match="ServoHorizon needs"):
        ServoHorizon(**settings)
