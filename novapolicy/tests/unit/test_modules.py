"""The built-in send-pipeline modules on their own."""

from __future__ import annotations

import pytest

from novapolicy.jogging.clock import ServerMs
from novapolicy.jogging.modules import EndpointRamp, HoldPadding, JointClamp, default_pipeline
from novapolicy.jogging.pipeline import ModuleContext, Timestamps, split_pipeline
from novapolicy.jogging.waypoints import PendingChunk
from novapolicy.joint_limits import JointLimits
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
        assert isinstance(pipeline[-1], Timestamps)
    assert [type(m) for m in continuous] == [JointClamp, *[type(m) for m in jog]]
    assert [type(m) for m in sequential] == [EndpointRamp, *[type(m) for m in continuous]]


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
