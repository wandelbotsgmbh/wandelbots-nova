import numpy as np
import pytest

from nova.types import Pose, Vector3d


def test_pose_from_euler_degrees():
    """
    Tests creating a Pose from Euler angles specified in degrees.
    A 90-degree rotation around the Z-axis.
    """
    # Input
    position = (0.1, 0.2, 0.3)
    euler_angles_deg = (0, 0, 90)

    # Expected orientation is a rotation vector of magnitude pi/2 around the Z-axis
    expected_orientation = (0, 0, np.pi / 2)

    # Create pose using the new method
    pose = Pose.from_euler(position, euler_angles_deg, convention="xyz", degrees=True)

    # Assert
    assert pose.position == Vector3d.from_tuple(position)
    assert np.allclose(pose.orientation.to_tuple(), expected_orientation, atol=1e-7)


def test_pose_from_euler_radians():
    """
    Tests creating a Pose from Euler angles specified in radians.
    A 45-degree (pi/4) rotation around the Y-axis.
    """
    # Input
    position = (0.5, 0, 0)
    euler_angles_rad = (0, np.pi / 4, 0)

    # Expected orientation is a rotation vector of magnitude pi/4 around the Y-axis
    expected_orientation = (0, np.pi / 4, 0)

    # Create pose using the new method
    pose = Pose.from_euler(position, euler_angles_rad, convention="xyz", degrees=False)

    # Assert
    assert pose.position == Vector3d.from_tuple(position)
    assert np.allclose(pose.orientation.to_tuple(), expected_orientation, atol=1e-7)


def test_pose_from_euler_zero_rotation():
    """
    Tests creating a Pose with zero rotation.
    """
    position = (1, 2, 3)
    euler_angles = (0, 0, 0)
    expected_orientation = (0, 0, 0)

    pose = Pose.from_euler(position, euler_angles, degrees=True)

    assert pose.position == Vector3d.from_tuple(position)
    assert np.allclose(pose.orientation.to_tuple(), expected_orientation, atol=1e-7)
