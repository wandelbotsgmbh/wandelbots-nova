"""The built-in send-pipeline modules on their own."""

from __future__ import annotations

from novapolicy.jogging.clock import ServerMs
from novapolicy.jogging.modules import HoldPadding, JointClamp, default_pipeline
from novapolicy.jogging.pipeline import ModuleContext, Timestamps, split_pipeline
from novapolicy.jogging.waypoints import PendingChunk
from novapolicy.joint_limits import JointLimits

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


def test_the_defaults_are_a_valid_pipeline_and_differ_only_by_the_clamp() -> None:
    jog = default_pipeline()
    executor = default_pipeline(execution=object())
    for pipeline in (jog, executor):
        split_pipeline(pipeline)
        assert isinstance(pipeline[-1], Timestamps)
    assert [type(m) for m in executor] == [JointClamp, *[type(m) for m in jog]]
