from math import pi

import numpy as np
import pytest

from nova.api import models
from nova.utils.joint_position import shift_joint_position_close_to_reference


def test_shift_joint_position_close_to_reference_wraps_each_joint():
    result = shift_joint_position_close_to_reference(
        joint_position=np.array([2 * pi + 0.1, -3 * pi / 2]),
        reference_position=np.array([0.0, 0.0]),
        joint_limits=None,
    )

    assert result[0] == pytest.approx(0.1)
    assert result[1] == pytest.approx(pi / 2)


def test_shift_joint_position_close_to_reference_respects_joint_limits():
    joint_limits = [
        models.JointLimits(position=models.LimitRange(lower_limit=-pi / 2, upper_limit=pi / 2))
    ]

    result = shift_joint_position_close_to_reference(
        joint_position=np.array([3 * pi / 4]),
        reference_position=np.array([0.0]),
        joint_limits=joint_limits,
    )

    assert result[0] == pytest.approx(3 * pi / 4)
