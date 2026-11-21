"""Tests for the opt-in pre-send stages (novapolicy.jogging.stages)."""

from __future__ import annotations

from itertools import pairwise

import pytest

from novapolicy.jogging.clock import ServerMs
from novapolicy.jogging.stages import EaseIn, StageContext

TARGET = [100.0, 0.0, 0.0, 0.0, 0.0, 0.0]
AT_REST = StageContext(now_ms=ServerMs(0), position=[0.0] * 6)


def _blend(ease: EaseIn, times_ms: range) -> list[float]:
    """Joint 0 of one fixed target, blended at each of these session times."""
    waypoints = [(ServerMs(t), list(TARGET)) for t in times_ms]
    return [v[0] for v in ease(waypoints, AT_REST)]


def test_ease_in_spans_the_window_from_where_the_robot_stood():
    ease = EaseIn(duration_s=0.5)
    blended = _blend(ease, range(1_000, 1_600, 100))
    assert blended[0] == pytest.approx(0.0)  # the first waypoint starts the window
    assert blended[5] == pytest.approx(100.0)  # 500 ms later it is the target


def test_ease_in_releases_without_a_velocity_step():
    """A straight ramp stops dead at its end; the stumble shows as one acceleration spike."""
    positions = _blend(EaseIn(duration_s=0.5), range(0, 600, 10))
    velocity = [b - a for a, b in pairwise(positions)]
    accel = [abs(b - a) for a, b in pairwise(velocity)]
    assert max(accel) < 3 * (sum(accel) / len(accel))


def test_ease_in_starts_and_ends_without_an_acceleration_step():
    """Smootherstep zeroes velocity and acceleration at both ends (smoothstep only velocity)."""
    positions = _blend(EaseIn(duration_s=0.5), range(0, 510, 10))
    velocity = [b - a for a, b in pairwise(positions)]
    accel = [abs(b - a) for a, b in pairwise(velocity)]
    peak = max(accel)
    assert accel[0] < 0.5 * peak
    assert accel[-1] < 0.5 * peak


def test_ease_in_gives_a_moment_the_same_value_whenever_it_is_sent():
    """The rolling window re-sends waypoints: each must keep its blend, so seams agree."""
    ease = EaseIn(duration_s=0.5)
    first = ease([(ServerMs(1_000), TARGET), (ServerMs(1_200), TARGET)], AT_REST)
    again = ease([(ServerMs(1_200), TARGET), (ServerMs(1_400), TARGET)], AT_REST)
    assert again[0] == first[1]


def test_ease_in_waits_for_a_measured_position():
    ease = EaseIn(duration_s=0.5)
    unknown = StageContext(now_ms=ServerMs(0), position=None)
    assert ease([(ServerMs(0), TARGET)], unknown) == [TARGET]
    assert ease([(ServerMs(0), TARGET)], AT_REST)[0][0] == pytest.approx(0.0)


def test_ease_in_needs_a_positive_duration():
    with pytest.raises(ValueError, match="positive duration_s"):
        EaseIn(duration_s=0.0)
