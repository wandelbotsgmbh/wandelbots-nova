"""Tests for the send pipeline (novapolicy.pipeline.base): its split and EaseIn."""

from __future__ import annotations

from itertools import pairwise
import math

import pytest

from novapolicy.pipeline.base import (
    ChunkModule,
    ModuleContext,
    Timestamps,
    WaypointModule,
    split_pipeline,
)
from novapolicy.pipeline.defaults import default_pipeline
from novapolicy.pipeline.modules.bridge import Bridge
from novapolicy.pipeline.modules.drop_late import DropLate
from novapolicy.pipeline.modules.ease_in import EaseIn
from novapolicy.pipeline.modules.endpoint_ramp import EndpointRamp
from novapolicy.pipeline.modules.hold_padding import HoldPadding
from novapolicy.pipeline.modules.joint_clamp import JointClamp
from novapolicy.pipeline.modules.live_buffer import LiveBuffer
from novapolicy.pipeline.modules.smoothing import Smoothing
from novapolicy.pipeline.modules.timeline import TimestepTimeline
from novapolicy.pipeline.placement import resample_on_grid
from novapolicy.transport.clock import ServerMs

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
    chunk_modules, filters, waypoint_modules, _timestamps = split_pipeline([
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
    assert split_pipeline([buffer, ease]) == ([buffer], [], [ease], Timestamps())
    assert split_pipeline([Timestamps()]) == ([], [], [], Timestamps())
    assert split_pipeline([]) == ([], [], [], Timestamps())


def test_the_split_carries_the_listed_timestamps():
    grid = Timestamps(grid_ms=32.0)
    assert split_pipeline([LiveBuffer(window_ms=100.0), grid, DropLate()]).timestamps is grid


@pytest.mark.parametrize("kwargs", [{"grid_ms": 0.0}, {"grid_ms": -8.0}, {"grid_ms": math.inf}])
def test_timestamps_refuses_a_grid_it_cannot_place_on(kwargs):
    with pytest.raises(ValueError, match="Timestamps needs"):
        Timestamps(**kwargs)


def test_a_path_is_sampled_on_the_grid_from_the_start_to_its_last_point():
    path = [(0.0, [0.0]), (100.0, [1.0])]
    assert resample_on_grid(path, 10.0, 32.0) == [
        (32, [pytest.approx(0.32)]),
        (64, [pytest.approx(0.64)]),
        (96, [pytest.approx(0.96)]),
    ]
    assert resample_on_grid([(40.0, [0.0]), (100.0, [1.0])], 0.0, 32.0)[0][0] == 64  # 32 is before
    assert resample_on_grid([], 0.0, 32.0) == []


def test_the_grid_starts_at_the_robot_and_continues_the_policy_path():
    grid = Timestamps(grid_ms=32.0)
    assert Timestamps().on_grid([(1_150, [1.0])], 1_000.0, [0.0]) == []  # off by default
    first = grid.on_grid([(1_150, [1.5])], 1_000.0, [0.0])
    # From the robot at "now" to the policy's target, on multiples of 32 ms from now + 20 ms.
    assert [t for t, _ in first] == [1_024, 1_056, 1_088, 1_120]
    assert first[0][1] == [pytest.approx(0.24)]
    # The next target extends the same path: the sent part is not resampled differently.
    second = grid.on_grid([(1_183, [1.83])], 1_033.0, [9.0])
    assert [t for t, _ in second] == [1_056, 1_088, 1_120, 1_152]
    assert dict(second)[1_088] == dict(first)[1_088]
    assert second[-1][1] == [pytest.approx(1.52)]
    assert grid.on_grid([], 1_066.0, None) == []  # forgets the path
    assert grid.on_grid([(1_216, [2.0])], 1_066.0, [3.0])[0][1][0] > 2.0  # restarts at the robot


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


class _OwnModule:
    """A user's chunk module: no stage, so it may stand anywhere among the chunk modules."""

    def on_chunk(self, chunk, context):
        return chunk


@pytest.mark.parametrize(
    ("pipeline", "match"),
    [
        ([Bridge(), TimestepTimeline()], "TimestepTimeline must come before Bridge"),
        ([TimestepTimeline(), Smoothing()], "Smoothing must come before TimestepTimeline"),
        ([HoldPadding(), EndpointRamp()], "EndpointRamp must come before HoldPadding"),
        ([Bridge(), LiveBuffer(window_ms=500)], "LiveBuffer must come before Bridge"),
    ],
    ids=[
        "timeline-after-bridge",
        "smoothing-after-timeline",
        "hold-before-ramp",
        "live-buffer-late",
    ],
)
def test_chunk_modules_out_of_their_stages_order_are_refused(pipeline, match):
    with pytest.raises(ValueError, match=match):
        split_pipeline(pipeline)


@pytest.mark.parametrize(
    "pipeline",
    [[Bridge(), Bridge()], [JointClamp(), Bridge(), JointClamp()], [DropLate(), DropLate()]],
    ids=["bridge", "clamp", "drop-late"],
)
def test_a_built_in_module_twice_is_refused(pipeline):
    with pytest.raises(ValueError, match="appears twice"):
        split_pipeline(pipeline)


def test_a_clamp_and_your_own_modules_may_stand_anywhere_among_the_chunk_modules():
    own = _OwnModule()
    pipeline = [
        JointClamp(),
        Smoothing(),
        own,
        TimestepTimeline(),
        Bridge(),
        _OwnModule(),
        EndpointRamp(),
        HoldPadding(),
    ]
    assert len(split_pipeline(pipeline).chunk_modules) == 8


def test_every_default_pipeline_keeps_the_stages_order():
    from novapolicy.providers.base import CallbackPolicyClient
    from novapolicy.providers.queued import QueuedPolicyClient
    from novapolicy.types import ContinuousExecution, SequentialExecution

    queue = QueuedPolicyClient(CallbackPolicyClient(lambda _obs: None))
    for pipeline in (
        default_pipeline(),
        default_pipeline(execution=SequentialExecution()),
        default_pipeline(execution=ContinuousExecution(rate_hz=10), policy=queue),
        [LiveBuffer(window_ms=500), *default_pipeline()],
    ):
        split_pipeline(pipeline)
