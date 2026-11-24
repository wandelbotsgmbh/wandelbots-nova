"""Holding targets inside the robot's limits, and reading those limits.

The point of the clamp is that NOVA pauses the motion on an out-of-range
waypoint, so these assert that an impossible target becomes a saturated one
rather than a stopped episode.
"""

from __future__ import annotations

import logging
from types import SimpleNamespace

import pytest

from novapolicy.joint_limits import (
    DEFAULT_MARGIN_RAD,
    JointLimits,
    fetch_joint_limits,
    limits_from_description,
)

MG_ID = "1@manus"
# The real manus arm, whose elbow has 0.5 deg of positive travel.
ELBOW_LOW, ELBOW_HIGH = -1.7783, 0.0087


def _description(bounds: list[tuple[float, float]] | None) -> SimpleNamespace:
    if bounds is None:
        return SimpleNamespace(operation_limits=SimpleNamespace(auto_limits=None))
    joints = [
        SimpleNamespace(position=SimpleNamespace(lower_limit=low, upper_limit=high))
        for low, high in bounds
    ]
    return SimpleNamespace(
        operation_limits=SimpleNamespace(auto_limits=SimpleNamespace(joints=joints))
    )


def _limits(low: float = -1.0, high: float = 1.0, dof: int = 3) -> JointLimits:
    return JointLimits(lower=(low,) * dof, upper=(high,) * dof)


# -- clamping ---------------------------------------------------------------


def test_a_target_inside_the_bounds_is_untouched() -> None:
    limits = _limits()
    assert limits.clamp([0.0, 0.5, -0.5]) == [0.0, 0.5, -0.5]


def test_a_target_below_the_bound_is_held_at_it() -> None:
    limits = _limits()
    assert limits.clamp([-5.0, 0.0, 0.0]) == [-1.0, 0.0, 0.0]


def test_a_target_above_the_bound_is_held_at_it() -> None:
    limits = _limits()
    assert limits.clamp([0.0, 0.0, 5.0]) == [0.0, 0.0, 1.0]


def test_only_the_offending_joint_moves() -> None:
    """Saturating one joint must not disturb the others' targets."""
    limits = _limits()
    assert limits.clamp([-9.0, 0.25, 9.0]) == [-1.0, 0.25, 1.0]


def test_the_original_list_is_not_mutated() -> None:
    """The caller's buffers are reused; clamping in place would corrupt them."""
    limits = _limits()
    step = [-9.0, 0.0, 0.0]
    limits.clamp(step)
    assert step == [-9.0, 0.0, 0.0]


def test_a_width_mismatch_passes_through_untouched() -> None:
    """Guessing an alignment would silently clamp the wrong joints."""
    limits = _limits(dof=3)
    assert limits.clamp([-9.0, -9.0]) == [-9.0, -9.0]


def test_the_first_clip_per_joint_is_reported(caplog: pytest.LogCaptureFixture) -> None:
    """A saturating joint means out-of-distribution or mis-mounted; say so once."""
    limits = _limits()
    with caplog.at_level(logging.WARNING, logger="novapolicy.joint_limits"):
        for _ in range(5):
            limits.clamp([-9.0, 0.0, 0.0], MG_ID)
    warnings = [r for r in caplog.records if r.levelno == logging.WARNING]
    assert len(warnings) == 1
    assert MG_ID in warnings[0].getMessage()


def test_each_joint_reports_separately(caplog: pytest.LogCaptureFixture) -> None:
    limits = _limits()
    with caplog.at_level(logging.WARNING, logger="novapolicy.joint_limits"):
        limits.clamp([-9.0, 0.0, 0.0], MG_ID)
        limits.clamp([0.0, 0.0, 9.0], MG_ID)
    assert len([r for r in caplog.records if r.levelno == logging.WARNING]) == 2
    assert limits.saturated == {0, 2}


# -- reading the limits -----------------------------------------------------


def test_bounds_are_read_and_pulled_inward_by_the_margin() -> None:
    """Resting exactly on a bound was still refused, so the margin is real."""
    limits = limits_from_description(_description([(ELBOW_LOW, ELBOW_HIGH)]))
    assert limits is not None
    assert limits.lower[0] == pytest.approx(ELBOW_LOW + DEFAULT_MARGIN_RAD)
    assert limits.upper[0] == pytest.approx(ELBOW_HIGH - DEFAULT_MARGIN_RAD)


def test_the_clamped_elbow_stays_off_its_limit() -> None:
    """The case that ended every real-policy run: a target past the elbow stop."""
    limits = limits_from_description(_description([(ELBOW_LOW, ELBOW_HIGH)]))
    assert limits is not None
    held = limits.clamp([-1.9])[0]
    assert held > ELBOW_LOW, "a clamped target on the exact bound can still be refused"
    assert held == pytest.approx(ELBOW_LOW + DEFAULT_MARGIN_RAD)


def test_a_description_without_limits_reads_as_none() -> None:
    assert limits_from_description(_description(None)) is None


def test_a_description_with_no_operation_limits_reads_as_none() -> None:
    assert limits_from_description(SimpleNamespace()) is None


def test_a_partial_bound_rejects_the_whole_set() -> None:
    """Clamping some joints against guessed bounds is worse than not clamping."""
    joints = [
        SimpleNamespace(position=SimpleNamespace(lower_limit=-1.0, upper_limit=1.0)),
        SimpleNamespace(position=None),
    ]
    description = SimpleNamespace(
        operation_limits=SimpleNamespace(auto_limits=SimpleNamespace(joints=joints))
    )
    assert limits_from_description(description) is None


def test_a_degenerate_bound_rejects_the_whole_set() -> None:
    """A range thinner than twice the margin would inverse the bounds."""
    assert limits_from_description(_description([(0.0, DEFAULT_MARGIN_RAD)])) is None


def test_a_non_iterable_joints_field_reads_as_none() -> None:
    """A mock or a changed SDK must not raise on the way to a robot."""
    description = SimpleNamespace(
        operation_limits=SimpleNamespace(auto_limits=SimpleNamespace(joints=object()))
    )
    assert limits_from_description(description) is None


# -- fetching ---------------------------------------------------------------


class _MotionGroup:
    def __init__(self, mg_id: str, description: object | Exception) -> None:
        self.id = mg_id
        self._description = description

    async def get_description(self) -> object:
        if isinstance(self._description, Exception):
            raise self._description
        return self._description


@pytest.mark.asyncio
async def test_limits_are_fetched_per_motion_group() -> None:
    groups = [
        _MotionGroup("1@manus", _description([(ELBOW_LOW, ELBOW_HIGH)])),
        _MotionGroup("2@manus", _description([(-1.0, 1.0)])),
    ]
    limits = await fetch_joint_limits(groups)  # type: ignore[arg-type]
    assert sorted(limits) == ["1@manus", "2@manus"]


@pytest.mark.asyncio
async def test_a_motion_group_without_limits_is_skipped_not_fatal() -> None:
    """Unclamped is how this behaved before; it must still start."""
    groups = [_MotionGroup(MG_ID, _description(None))]
    assert await fetch_joint_limits(groups) == {}  # type: ignore[arg-type]


@pytest.mark.asyncio
async def test_a_failing_query_is_skipped_not_fatal() -> None:
    groups = [_MotionGroup(MG_ID, RuntimeError("gateway down"))]
    assert await fetch_joint_limits(groups) == {}  # type: ignore[arg-type]
