from geometricalgebra import cga3d

from nova.types import Pose


def pose_to_versor(pose: Pose) -> cga3d.Vector:
    """Convert a Pose to a versor

    Args:
        pose: The pose to convert

    Returns:
        The versor

    Examples:
    >>> import numpy as np
    >>> p = Pose((0, 0, 5, 0, 0, 1))
    >>> assert np.allclose(pose_to_versor(p).to_pos_and_rot_vector(), [0, 0, 5, 0, 0, 1])
    """
    return cga3d.Vector.from_pos_and_rot_vector([*pose.to_tuple()])


def versor_to_pose(versor: cga3d.Vector) -> Pose:
    """Convert a versor to a Pose

    Args:
        versor: The versor to convert

    Returns:
        The Pose

    Examples:
    >>> v = cga3d.Vector.from_pos_and_rot_vector([0, 0, 5, 0, 0, 1])
    >>> versor_to_pose(v).to_tuple()
    (0.0, 0.0, 5.0, 0.0, 0.0, 1.0)
    """
    pos_rot = versor.to_pos_and_rot_vector()
    return Pose(tuple(pos_rot))
