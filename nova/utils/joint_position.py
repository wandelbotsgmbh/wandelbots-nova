import numpy as np

from nova import api

TAU = np.pi * 2


def shift_joint_position_close_to_reference(
    joint_position: np.ndarray,
    reference_position: np.ndarray,
    joint_limits: list[api.models.JointLimits] | None,
) -> np.ndarray:
    shifted_joints = joint_position + TAU * np.round((reference_position - joint_position) / TAU)

    if joint_limits is None:
        return shifted_joints

    for i, joint_limit in enumerate(joint_limits):
        if joint_limit.position is None:
            continue

        lower_limit = joint_limit.position.lower_limit
        upper_limit = joint_limit.position.upper_limit
        while upper_limit is not None and shifted_joints[i] > upper_limit:
            shifted_joints[i] -= TAU
        while lower_limit is not None and shifted_joints[i] < lower_limit:
            shifted_joints[i] += TAU
        if (upper_limit is not None and shifted_joints[i] > upper_limit) or (
            lower_limit is not None and shifted_joints[i] < lower_limit
        ):
            return joint_position  # this should not happen, only when initial joints where out of limits

    return shifted_joints
