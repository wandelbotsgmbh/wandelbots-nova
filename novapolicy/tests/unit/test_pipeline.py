"""Tests for the send pipeline (novapolicy.jogging.pipeline): its split and EaseIn."""

from __future__ import annotations

from itertools import pairwise

import pytest

from novapolicy.jogging.clock import ServerMs
from novapolicy.jogging.live_buffer import LiveBuffer
from novapolicy.jogging.modules import DropLate
from novapolicy.jogging.pipeline import (
    ChunkModule,
    EaseIn,
    ModuleContext,
    Timestamps,
    WaypointModule,
    split_pipeline,
)

TARGET = [100.0, 0.0, 0.0, 0.0, 0.0, 0.0]
AT_REST = ModuleContext(now_ms=ServerMs(0), position=[0.0] * 6, step_ms=50.0)


def _blend(ease: EaseIn, times_ms: range) -> list[float]:
    """Joint 0 of one fixed target, blended at each of these session times."""
    waypoints = [(ServerMs(t), list(TARGET)) for t in times_ms]
    return [v[0] for v in ease.on_waypoints(waypoints, AT_REST)]


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
    first = ease.on_waypoints([(ServerMs(1_000), TARGET), (ServerMs(1_200), TARGET)], AT_REST)
    again = ease.on_waypoints([(ServerMs(1_200), TARGET), (ServerMs(1_400), TARGET)], AT_REST)
    assert again[0] == first[1]


def test_ease_in_waits_for_a_measured_position():
    ease = EaseIn(duration_s=0.5)
    unknown = ModuleContext(now_ms=ServerMs(0), position=None, step_ms=50.0)
    assert ease.on_waypoints([(ServerMs(0), TARGET)], unknown) == [TARGET]
    assert ease.on_waypoints([(ServerMs(0), TARGET)], AT_REST)[0][0] == pytest.approx(0.0)


def test_ease_in_needs_a_positive_duration():
    with pytest.raises(ValueError, match="positive duration_s"):
        EaseIn(duration_s=0.0)


# -- split_pipeline -----------------------------------------------------------


class _ChunkProbe:
    def on_chunk(self, chunk, context):
        return chunk


class _WaypointProbe:
    def on_waypoints(self, waypoints, context):
        return [v for _, v in waypoints]


def test_the_module_kinds_are_told_apart_by_their_method():
    assert isinstance(LiveBuffer(window_ms=100.0), ChunkModule)
    assert not isinstance(LiveBuffer(window_ms=100.0), WaypointModule)
    assert isinstance(EaseIn(duration_s=1.0), WaypointModule)
    assert not isinstance(EaseIn(duration_s=1.0), ChunkModule)


def test_split_pipeline_keeps_each_kind_in_order():
    first, second, buffer = _ChunkProbe(), _ChunkProbe(), LiveBuffer(window_ms=100.0)
    ease, probe, late = EaseIn(duration_s=1.0), _WaypointProbe(), DropLate()
    chunk_modules, filters, waypoint_modules = split_pipeline([
        first,
        buffer,
        second,
        Timestamps(),
        late,
        ease,
        probe,
    ])
    assert chunk_modules == [first, buffer, second]
    assert filters == [late]
    assert waypoint_modules == [ease, probe]


def test_timestamps_is_optional_and_an_empty_pipeline_has_no_modules():
    buffer, ease = LiveBuffer(window_ms=100.0), EaseIn(duration_s=1.0)
    assert split_pipeline([buffer, ease]) == ([buffer], [], [ease])
    assert split_pipeline([Timestamps()]) == ([], [], [])
    assert split_pipeline([]) == ([], [], [])


def test_a_filter_after_a_waypoint_module_or_a_chunk_module_after_a_filter_is_refused():
    with pytest.raises(ValueError, match="before the waypoint modules"):
        split_pipeline([EaseIn(duration_s=1.0), DropLate()])
    with pytest.raises(ValueError, match="before Timestamps"):
        split_pipeline([DropLate(), LiveBuffer(window_ms=100.0)])


def test_a_chunk_module_after_the_timestamps_is_refused():
    with pytest.raises(ValueError, match="before Timestamps"):
        split_pipeline([Timestamps(), LiveBuffer(window_ms=100.0)])


@pytest.mark.parametrize(
    ("pipeline", "error", "match"),
    [
        ([EaseIn(duration_s=1.0), LiveBuffer(window_ms=100.0)], ValueError, "before Timestamps"),
        ([Timestamps(), Timestamps()], ValueError, "appears twice"),
        ([object()], TypeError, "not a pipeline module"),
        ([lambda waypoints, context: waypoints], TypeError, "not a pipeline module"),  # old form
    ],
    ids=["chunk-module-after-waypoint-module", "timestamps-twice", "not-a-module", "old-stage"],
)
def test_a_misordered_or_foreign_pipeline_is_refused(pipeline, error, match):
    with pytest.raises(error, match=match):
        split_pipeline(pipeline)
