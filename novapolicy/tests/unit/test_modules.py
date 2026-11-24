"""The built-in send-pipeline modules on their own."""

from __future__ import annotations

from dataclasses import replace

import pytest

from novapolicy.mapping.joint_limits import JointLimits
from novapolicy.pipeline.base import ModuleContext, Timestamps, split_pipeline
from novapolicy.pipeline.defaults import default_pipeline, timestep_queue_modules
from novapolicy.pipeline.modules.bridge import Bridge
from novapolicy.pipeline.modules.drop_late import DropLate
from novapolicy.pipeline.modules.endpoint_ramp import EndpointRamp
from novapolicy.pipeline.modules.hold_padding import HoldPadding
from novapolicy.pipeline.modules.joint_clamp import JointClamp
from novapolicy.pipeline.modules.smoothing import Smoothing
from novapolicy.pipeline.modules.timeline import TimestepTimeline
from novapolicy.transport.clock import ServerMs
from novapolicy.transport.waypoints import PendingChunk
from novapolicy.types import ContinuousExecution, SequentialExecution

LIMITS = JointLimits(lower=(-1.0,) * 3, upper=(1.0,) * 3)


def _context(limits: JointLimits | None = None) -> ModuleContext:
    return ModuleContext(
        now_ms=ServerMs(0),
        position=None,
        step_ms=50.0,
        motion_group_id="0@arm",
        joint_limits=limits,
    )


def _chunk(steps: list[list[float]], *, dt_ms: float = 100.0, pad: bool = True) -> PendingChunk:
    return PendingChunk(steps=steps, dt_ms=dt_ms, caller_step_count=len(steps), extend_buffer=pad)


def test_hold_padding_holds_the_last_target_up_to_the_horizon() -> None:
    padded = HoldPadding(min_horizon_ms=500.0).on_chunk(_chunk([[0.0], [1.0]]), _context())
    assert padded.steps == [[0.0], [1.0], [1.0], [1.0], [1.0]]
    assert padded.caller_step_count == 2  # the holds are not the caller's steps


def test_hold_padding_leaves_chunks_that_asked_for_none_or_are_long_enough() -> None:
    module = HoldPadding(min_horizon_ms=500.0)
    unpadded = _chunk([[0.0]], pad=False)
    assert module.on_chunk(unpadded, _context()) is unpadded
    long = _chunk([[float(i)] for i in range(5)])
    assert module.on_chunk(long, _context()) is long


def test_hold_padding_holds_after_the_last_step_however_long_the_chunk() -> None:
    long = _chunk([[float(i)] for i in range(5)])  # already spans the 500 ms horizon
    padded = HoldPadding(min_horizon_ms=500.0, after_ms=200.0).on_chunk(long, _context())
    assert padded.steps == [*long.steps, [4.0], [4.0]]
    assert padded.caller_step_count == 5


def test_joint_clamp_holds_targets_inside_the_limits() -> None:
    clamped = JointClamp().on_chunk(_chunk([[2.0, -3.0, 0.5]]), _context(LIMITS))
    assert clamped.steps == [[1.0, -1.0, 0.5]]


def test_joint_clamp_passes_everything_without_limits() -> None:
    chunk = _chunk([[2.0, -3.0, 0.5]])
    assert JointClamp().on_chunk(chunk, _context()) is chunk


def test_the_defaults_are_valid_pipelines() -> None:
    jog = default_pipeline()
    continuous = default_pipeline(execution=ContinuousExecution())
    sequential = default_pipeline(execution=SequentialExecution())
    for pipeline in (jog, continuous, sequential):
        split_pipeline(pipeline)
        assert [type(m) for m in pipeline[-2:]] == [Timestamps, DropLate]
    assert [type(m) for m in continuous] == [JointClamp, *[type(m) for m in jog]]
    assert [type(m) for m in sequential] == [Bridge, EndpointRamp, *[type(m) for m in continuous]]


def test_a_queue_policy_gets_an_anchored_bridge_in_continuous_execution() -> None:
    class _Queue:
        default_modules = timestep_queue_modules()

    pipeline = default_pipeline(execution=ContinuousExecution(), policy=_Queue())
    assert [type(m) for m in pipeline[:3]] == [Smoothing, TimestepTimeline, Bridge]
    assert pipeline[2] == Bridge(always_anchor=True)


def test_endpoint_ramp_validates_interpolation_steps() -> None:
    with pytest.raises(ValueError, match="interpolation_steps must be at least 2"):
        EndpointRamp(interpolation_steps=1)


def test_endpoint_ramp_keeps_every_step_and_moves_io_with_its_step() -> None:
    async def io() -> None:
        pass

    chunk = PendingChunk(
        steps=[[0.0], [1.0], [2.0], [3.0]], dt_ms=50.0, caller_step_count=4, when_due=(2, io)
    )
    ramped = EndpointRamp(interpolation_steps=3).on_chunk(chunk, _context())
    assert len(ramped.steps) == 1 + 3 + 1 + 3  # eased first and last interval
    assert ramped.caller_step_count == len(ramped.steps)
    assert ramped.when_due is not None
    assert ramped.steps[ramped.when_due[0]] == [2.0]


def test_drop_late_keeps_what_is_still_reachable_and_never_retimes() -> None:
    chunk = PendingChunk(steps=[[0.0]] * 4, dt_ms=10.0, caller_step_count=4, first_timestamp_ms=0)
    waypoints = [(ServerMs(t), [float(t)]) for t in (100, 110, 120, 130)]
    context = ModuleContext(now_ms=ServerMs(85), position=None, step_ms=50.0)  # 105 is the edge
    assert DropLate(min_lead_ms=20.0).keep(waypoints, chunk, context) == waypoints[1:]


def test_drop_late_sends_no_padding_when_every_caller_step_is_late() -> None:
    """The padding behind a late chunk holds its last, stale step; sent alone, it pulled the arm
    back. Nothing goes out, so the previous chunk runs on."""
    chunk = PendingChunk(steps=[[0.0], [1.0], [1.0], [1.0]], dt_ms=100.0, caller_step_count=2)
    chunk = replace(chunk, first_timestamp_ms=ServerMs(0))
    waypoints = [(ServerMs(t), v) for t, v in zip((0, 100, 200, 300), chunk.steps, strict=True)]
    context = ModuleContext(now_ms=ServerMs(150), position=None, step_ms=50.0)
    assert DropLate().keep(waypoints, chunk, context) == []
    # One caller step still reachable: it and the padding after it go out.
    later = ModuleContext(now_ms=ServerMs(70), position=None, step_ms=50.0)
    assert DropLate().keep(waypoints, chunk, later) == waypoints[1:]


def test_drop_late_passes_a_chunk_stamped_at_the_send() -> None:
    chunk = PendingChunk(steps=[[0.0]] * 2, dt_ms=10.0, caller_step_count=2)
    waypoints = [(ServerMs(0), [0.0]), (ServerMs(10), [1.0])]
    context = ModuleContext(now_ms=ServerMs(0), position=None, step_ms=50.0)
    assert DropLate().keep(waypoints, chunk, context) == waypoints


def _at(position: list[float]) -> ModuleContext:
    return ModuleContext(now_ms=ServerMs(0), position=position, step_ms=50.0)


def test_bridge_connects_the_robot_to_the_first_step_at_the_chunks_spacing() -> None:
    chunk = _chunk([[3.0], [4.0], [5.0]], dt_ms=10.0)
    bridged = Bridge().on_chunk(chunk, _at([0.0]))
    assert bridged.steps == [[0.0], [1.0], [2.0], [3.0], [4.0], [5.0]]
    assert bridged.start_step == 3
    assert bridged.caller_step_count == 6


def test_bridge_leaves_a_chunk_on_its_own_timeline_or_one_the_robot_is_at() -> None:
    timed = PendingChunk(
        steps=[[3.0], [4.0]], dt_ms=10.0, caller_step_count=2, first_timestamp_ms=ServerMs(500)
    )
    assert Bridge().on_chunk(timed, _at([0.0])) is timed
    near = _chunk([[0.5], [1.0]], dt_ms=10.0)
    assert Bridge().on_chunk(near, _at([0.0])) is near
    # anchored, the robot's position is held before even a near first step
    assert Bridge(always_anchor=True).on_chunk(near, _at([0.0])).steps == [[0.0], [0.5], [1.0]]


def test_a_bridged_chunk_dated_by_its_observation_is_dated_at_the_send_instead() -> None:
    observed = PendingChunk(
        steps=[[3.0], [4.0]], dt_ms=10.0, caller_step_count=2, lead=True, observed_at_ms=ServerMs(7)
    )
    bridged = Bridge().on_chunk(observed, _at([0.0]))
    assert (bridged.lead, bridged.observed_at_ms, bridged.first_timestamp_ms) == (False, None, None)
    at_send = PendingChunk(steps=[[3.0], [4.0]], dt_ms=10.0, caller_step_count=2, lead=True)
    assert Bridge().on_chunk(at_send, _at([0.0])).lead is True  # sequential keeps its placement


def test_smoothing_keeps_the_steps_it_already_sent_exact() -> None:
    """A replacement starts with the queue's predecessor and frozen head: they keep the values
    sent for their timesteps; the rest is filtered."""
    module = Smoothing(passes=2)
    first = PendingChunk(
        steps=[[0.0], [4.0], [0.0], [4.0]], dt_ms=10.0, caller_step_count=4, action_timestep=0
    )
    sent = module.on_chunk(first, _context()).steps
    assert sent != first.steps
    replacement = PendingChunk(
        steps=[[9.0], [9.0], [9.0], [9.0]],  # unsmoothed values for timesteps 1..4
        dt_ms=10.0,
        caller_step_count=4,
        action_timestep=1,
        retained_prefix_steps=2,
    )
    resent = module.on_chunk(replacement, _context()).steps
    assert resent[:2] == sent[1:3]


def test_a_step_rate_comes_from_the_caller_else_the_policy_else_30_hz(caplog) -> None:
    from novapolicy.providers.base import resolve_control_freq

    assert resolve_control_freq(20, owner="p") == 20.0
    assert resolve_control_freq(None, owner="p", recorded=50) == 50.0
    assert resolve_control_freq(None, owner="p") == 30.0
    assert any("got no control_freq" in r.message for r in caplog.records)
    with pytest.raises(ValueError, match="positive"):
        resolve_control_freq(0, owner="p")


def test_a_timestep_timeline_places_later_chunks_on_the_first_ones_grid() -> None:
    """Until a chunk with timesteps was sent, chunks pass (and get bridged); after that, the
    chunk starting at timestep t is due origin + (t - t0) * dt."""
    module = TimestepTimeline()
    first = PendingChunk(steps=[[0.0]] * 3, dt_ms=10.0, caller_step_count=3, action_timestep=0)
    assert module.on_chunk(first, _context()) is first
    sent = ModuleContext(
        now_ms=ServerMs(0), position=None, step_ms=50.0, scheduled_start=(0, ServerMs(1_010))
    )
    later = PendingChunk(
        steps=[[0.0]] * 3, dt_ms=10.0, caller_step_count=3, action_timestep=4, lead=True
    )
    placed = module.on_chunk(later, sent)
    assert (placed.first_timestamp_ms, placed.lead) == (1_050, False)
    plain = PendingChunk(steps=[[0.0]], dt_ms=10.0, caller_step_count=1)
    assert module.on_chunk(plain, sent) is plain
