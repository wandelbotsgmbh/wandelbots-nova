"""Unit tests for the vectorised FK and axis-angle helpers in robot_visualizer.

Verifies that:
* ``_batch_dh_transforms`` produces the same per-link transforms as the scalar
  ``DHRobot.dh_transform`` path (``compute_forward_kinematics``).
* ``_batch_collect`` produces the same axis-angle decomposition as
  ``RobotVisualizer.rotation_matrix_to_axis_angle`` for representative rotations.
"""

from math import pi

import numpy as np
import pytest
from scipy.spatial.transform import Rotation

from nova import api
from nova_rerun_bridge.dh_robot import DHRobot
from nova_rerun_bridge.robot_visualizer import _batch_collect, _batch_dh_transforms


def _make_dh_parameters() -> list[api.models.DHParameter]:
    """Three-joint robot with varied DH parameters."""
    return [
        api.models.DHParameter(
            a=100.0, d=200.0, alpha=pi / 2, theta=0.0, reverse_rotation_direction=False
        ),
        api.models.DHParameter(
            a=300.0, d=0.0, alpha=0.0, theta=0.0, reverse_rotation_direction=False
        ),
        api.models.DHParameter(
            a=50.0, d=0.0, alpha=pi / 2, theta=0.1, reverse_rotation_direction=True
        ),
    ]


def _make_mounting_pose() -> api.models.Pose:
    """Non-identity mounting pose."""
    return api.models.Pose(
        position=api.models.Vector3d([10.0, 20.0, 30.0]),
        orientation=api.models.RotationVector([0.1, 0.2, 0.3]),
    )


SAMPLE_JOINTS: list[list[float]] = [
    [0.0, 0.0, 0.0],
    [0.5, -0.3, 1.2],
    [-1.0, 0.0, 0.7],
    [pi / 4, pi / 3, -pi / 6],
]


class TestBatchDHTransforms:
    """_batch_dh_transforms must match the scalar DHRobot FK for every sample and link."""

    @staticmethod
    def _scalar_fk(robot: DHRobot, joint_positions: list[float]) -> list[np.ndarray]:
        """Reference scalar FK matching compute_forward_kinematics logic."""
        accumulated = robot.pose_to_matrix(robot.mounting)
        transforms = [accumulated.copy()]
        for dh_param, jp in zip(robot.dh_parameters, joint_positions, strict=False):
            transform = robot.dh_transform(dh_param=dh_param, joint_position=jp)
            accumulated = accumulated @ transform
            transforms.append(accumulated.copy())
        return transforms

    def test_matches_scalar_fk_for_sample_joints(self):
        """Each (link, sample) slice must be close to the iterative scalar result."""
        dh_params = _make_dh_parameters()
        mounting_pose = _make_mounting_pose()
        robot = DHRobot(dh_params, mounting_pose)
        mounting_matrix = robot.pose_to_matrix(mounting_pose)

        all_joints = np.array(SAMPLE_JOINTS)  # (N, 3)
        batch_result = _batch_dh_transforms(dh_params, all_joints, mounting_matrix)

        # batch_result shape: (num_links+1, N, 4, 4)
        num_links_plus_one = len(dh_params) + 1
        assert batch_result.shape == (num_links_plus_one, len(SAMPLE_JOINTS), 4, 4)

        for sample_idx, joints in enumerate(SAMPLE_JOINTS):
            scalar_transforms = self._scalar_fk(robot, joints)
            for link_idx in range(num_links_plus_one):
                np.testing.assert_allclose(
                    batch_result[link_idx, sample_idx],
                    scalar_transforms[link_idx],
                    atol=1e-10,
                    err_msg=(f"Mismatch at link={link_idx}, sample={sample_idx}, joints={joints}"),
                )

    def test_identity_mounting_zero_joints(self):
        """With identity mounting and zero joints, link-0 must be identity."""
        dh_params = _make_dh_parameters()
        mounting_matrix = np.eye(4)

        all_joints = np.zeros((1, len(dh_params)))
        batch_result = _batch_dh_transforms(dh_params, all_joints, mounting_matrix)

        np.testing.assert_allclose(
            batch_result[0, 0], np.eye(4), atol=1e-12, err_msg="Link-0 should be identity"
        )

    def test_single_joint_single_sample(self):
        """Sanity check: one joint, one sample, verify the transform directly."""
        dh_params = [
            api.models.DHParameter(
                a=100.0, d=0.0, alpha=0.0, theta=0.0, reverse_rotation_direction=False
            )
        ]
        mounting_matrix = np.eye(4)
        joint_value = pi / 4
        all_joints = np.array([[joint_value]])

        batch_result = _batch_dh_transforms(dh_params, all_joints, mounting_matrix)

        # Build the expected DH transform manually
        c, s = np.cos(joint_value), np.sin(joint_value)
        expected = np.array(
            [
                [c, -s, 0.0, 100.0 * c],
                [s, c, 0.0, 100.0 * s],
                [0.0, 0.0, 1.0, 0.0],
                [0.0, 0.0, 0.0, 1.0],
            ]
        )
        np.testing.assert_allclose(batch_result[1, 0], expected, atol=1e-12)

    def test_reverse_rotation_direction(self):
        """A joint with reverse_rotation_direction should negate the joint value."""
        dh_params = [
            api.models.DHParameter(
                a=0.0, d=100.0, alpha=0.0, theta=0.0, reverse_rotation_direction=True
            )
        ]
        mounting_matrix = np.eye(4)
        joint_value = 0.5
        all_joints = np.array([[joint_value]])

        batch_result = _batch_dh_transforms(dh_params, all_joints, mounting_matrix)

        # With reverse, effective theta = 0 + joint_value * (-1) = -0.5
        effective_theta = -joint_value
        c, s = np.cos(effective_theta), np.sin(effective_theta)
        expected = np.array(
            [[c, -s, 0.0, 0.0], [s, c, 0.0, 0.0], [0.0, 0.0, 1.0, 100.0], [0.0, 0.0, 0.0, 1.0]]
        )
        np.testing.assert_allclose(batch_result[1, 0], expected, atol=1e-12)


# ---------------------------------------------------------------------------
# TestBatchCollect
# ---------------------------------------------------------------------------


class TestBatchCollect:
    """_batch_collect must produce the same axis-angle as rotation_matrix_to_axis_angle."""

    @staticmethod
    def _reference_axis_angle(rotation_matrix: np.ndarray) -> tuple[np.ndarray, float]:
        """Reference axis-angle using the same SVD approach as rotation_matrix_to_axis_angle."""
        U, _, Vt = np.linalg.svd(rotation_matrix)
        Rm_orth = U @ Vt
        if np.linalg.det(Rm_orth) < 0:
            U[:, -1] *= -1
            Rm_orth = U @ Vt

        rot = Rotation.from_matrix(Rm_orth)
        angle = rot.magnitude()
        axis = rot.as_rotvec()
        axis = axis / angle if angle > 1e-8 else np.array([1.0, 0.0, 0.0])
        return axis, float(angle)

    @staticmethod
    def _build_transform(rotation_matrix: np.ndarray, translation: np.ndarray) -> np.ndarray:
        """Build a 4x4 homogeneous transform from a 3x3 rotation and 3-vector translation."""
        T = np.eye(4)
        T[:3, :3] = rotation_matrix
        T[:3, 3] = translation
        return T

    def test_identity_rotation(self):
        """Identity rotation should yield angle ~0 and fallback axis [1,0,0]."""
        transforms = np.eye(4).reshape(1, 4, 4)
        positions: dict[str, list] = {}
        rotations: dict[str, list] = {}

        _batch_collect(transforms, positions, rotations, "test/identity")

        assert len(rotations["test/identity"]) == 1
        rr_rot = rotations["test/identity"][0]
        assert rr_rot.angle == pytest.approx(0.0, abs=1e-7)

    def test_known_rotations_match_reference(self):
        """Several non-trivial rotations must match the reference axis-angle decomposition."""
        test_rotations = [
            Rotation.from_euler("x", 90, degrees=True).as_matrix(),
            Rotation.from_euler("y", 90, degrees=True).as_matrix(),
            Rotation.from_euler("z", 90, degrees=True).as_matrix(),
            Rotation.from_euler("xyz", [30, 45, 60], degrees=True).as_matrix(),
            Rotation.from_euler("zyx", [-120, 10, 80], degrees=True).as_matrix(),
        ]
        translations = [
            np.array([100.0, 200.0, 300.0]),
            np.array([0.0, 0.0, 0.0]),
            np.array([-50.0, 75.0, 1000.0]),
            np.array([1.0, 2.0, 3.0]),
            np.array([0.0, 0.0, 0.0]),
        ]

        batch = np.array(
            [self._build_transform(rot, trans) for rot, trans in zip(test_rotations, translations)]
        )

        positions: dict[str, list] = {}
        rotations_dict: dict[str, list] = {}
        _batch_collect(batch, positions, rotations_dict, "test/rotations")

        assert len(rotations_dict["test/rotations"]) == len(test_rotations)
        assert len(positions["test/rotations"]) == len(translations)

        for i, (rot_matrix, trans) in enumerate(zip(test_rotations, translations)):
            ref_axis, ref_angle = self._reference_axis_angle(rot_matrix)
            ref_rotvec = ref_axis * ref_angle

            rr_rot = rotations_dict["test/rotations"][i]
            batch_rotvec = np.array(rr_rot.axis) * rr_rot.angle

            np.testing.assert_allclose(
                batch_rotvec,
                ref_rotvec,
                atol=1e-7,
                err_msg=f"Rotation vector mismatch at index {i}",
            )

            np.testing.assert_allclose(
                positions["test/rotations"][i],
                trans.tolist(),
                atol=1e-10,
                err_msg=f"Translation mismatch at index {i}",
            )

    def test_appends_to_existing_entity(self):
        """Calling _batch_collect twice on the same entity path should append, not overwrite."""
        t1 = self._build_transform(np.eye(3), np.array([1.0, 2.0, 3.0])).reshape(1, 4, 4)
        t2 = self._build_transform(np.eye(3), np.array([4.0, 5.0, 6.0])).reshape(1, 4, 4)

        positions: dict[str, list] = {}
        rotations: dict[str, list] = {}

        _batch_collect(t1, positions, rotations, "test/append")
        _batch_collect(t2, positions, rotations, "test/append")

        assert len(positions["test/append"]) == 2
        assert len(rotations["test/append"]) == 2
