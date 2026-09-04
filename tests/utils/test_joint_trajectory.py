"""Tests for combining synchronized multi-motion-group trajectories."""

import pytest

from nova import api
from nova.utils.joint_trajectory import combine_multi_trajectories


def _segment(*keys: str, end: float = 1.0) -> api.models.MultiJointTrajectory:
    return api.models.MultiJointTrajectory(
        joint_positions_by_motion_group_key={key: [[0.0] * 6, [0.0] * 6] for key in keys},
        times=[0.0, end],
        locations=[0.0, end],
    )


def test_segments_over_different_motion_groups__raises():
    with pytest.raises(ValueError, match="same motion groups"):
        combine_multi_trajectories([_segment("a", "b"), _segment("a", "c")])


def test_concatenates_shifting_the_seam():
    result = combine_multi_trajectories([_segment("a"), _segment("a")])
    # each segment's first sample duplicates the previous seam and is dropped:
    # 2 + (2 - 1) = 3 samples on one continuous 0..2 parameterization.
    assert result.times == [0.0, 1.0, 2.0]
    assert result.locations == [0.0, 1.0, 2.0]
    assert len(result.joint_positions_by_motion_group_key["a"]) == 3
