import numpy as np

from nova import api


def _pose_to_matrix(pose: api.models.Pose) -> np.ndarray:
    """
    Convert a PlannerPose (with rotation-vector orientation) into a 4x4 homogeneous transformation matrix.
    :param pose: A PlannerPose object with position: Vector3d and orientation: RotationVector.
    :return: A 4x4 numpy array representing the transformation.
    """
    # Extract translation
    if pose.position is None:
        x, y, z = 0.0, 0.0, 0.0
    else:
        x, y, z = pose.position[0], pose.position[1], pose.position[2]

    # Extract rotation vector (axis * angle)
    if pose.orientation is None:
        R = np.eye(3)
    else:
        rx, ry, rz = pose.orientation[0], pose.orientation[1], pose.orientation[2]
        theta = np.linalg.norm([rx, ry, rz])

        if theta < 1e-12:
            R = np.eye(3)
        else:
            k = np.array([rx, ry, rz]) / theta
            kx, ky, kz = k
            K = np.array([[0.0, -kz, ky], [kz, 0.0, -kx], [-ky, kx, 0.0]])

            sin_theta = np.sin(theta)
            one_minus_cos = 1.0 - np.cos(theta)
            # Rodrigues' rotation formula
            R = np.eye(3) + sin_theta * K + one_minus_cos * (K @ K)

    # Construct the full homogeneous transformation matrix
    T = np.eye(4)
    T[0:3, 0:3] = R
    T[0:3, 3] = [x, y, z]

    return T


class DHRobot:
    """A class for handling DH parameters and computing joint positions."""

    def __init__(
        self,
        dh_parameters: list[api.models.DHParameter],
        mounting: api.models.Pose,
        kinematic_chain_offset: api.models.Pose | None = None,
    ) -> None:
        """
        Initialize the DHRobot with DH parameters and a mounting pose.
        :param dh_parameters: List of DHParameter objects containing all joint configurations.
        :param mounting: Pose object representing the cell-specific placement of the motion
            group's base frame (world -> base frame).
        :param kinematic_chain_offset: Fixed transform between the base frame and the start
            of the DH chain (base frame -> start of DH chain). Unlike ``mounting``, this is
            intrinsic to the robot model itself, not to a particular installation -- it is
            what :class:`api.models.MotionGroupDescription` calls ``kinematic_chain_offset``.
            Defaults to identity when not given (most robot models don't need one).
        """
        self.dh_parameters = dh_parameters
        self.mounting = mounting
        self.kinematic_chain_offset = kinematic_chain_offset or api.models.Pose(
            position=(0, 0, 0), orientation=(0, 0, 0)
        )

    def pose_to_matrix(self, pose: api.models.Pose) -> np.ndarray:
        """
        Convert a PlannerPose (with rotation-vector orientation) into a 4x4 homogeneous transformation matrix.
        :param pose: A PlannerPose object with position: Vector3d and orientation: RotationVector.
        :return: A 4x4 numpy array representing the transformation.
        """
        return _pose_to_matrix(pose)

    def dh_transform(
        self, dh_param: api.models.DHParameter, joint_position: float | None
    ) -> np.ndarray:
        """
        Compute the homogeneous transformation matrix for a given DH parameter and joint rotation.
        :param dh_param: A single DH parameter.
        :param joint_rotation: The joint rotation value (in radians).
        :return: A 4x4 homogeneous transformation matrix.
        """
        # Adjust the angle based on rotation direction
        effective_joint = joint_position or 0.0
        theta = (dh_param.theta or 0.0) + effective_joint * (
            -1 if dh_param.reverse_rotation_direction else 1
        )
        d = dh_param.d or 0.0
        a = dh_param.a or 0.0
        alpha = dh_param.alpha or 0.0

        # Create the homogeneous transformation matrix for this DH parameter
        transformation = np.array(
            [
                [
                    np.cos(theta),
                    -np.sin(theta) * np.cos(alpha),
                    np.sin(theta) * np.sin(alpha),
                    a * np.cos(theta),
                ],
                [
                    np.sin(theta),
                    np.cos(theta) * np.cos(alpha),
                    -np.cos(theta) * np.sin(alpha),
                    a * np.sin(theta),
                ],
                [0, np.sin(alpha), np.cos(alpha), d],
                [0, 0, 0, 1],
            ]
        )
        return transformation

    def calculate_joint_positions(self, joint_positions: list[float]) -> list[list[float]]:
        """
        Compute joint positions based on joint values.
        :param joint_values: Object containing joint rotation values as a list in joint_values.joints.
        :return: A list of joint positions as [x, y, z].
        """
        # First position is the base frame (mounting only); kinematic_chain_offset applies
        # from the start of the DH chain onward, same as compute_forward_kinematics.
        accumulated_matrix = self.pose_to_matrix(self.mounting)
        positions = [accumulated_matrix[:3, 3].tolist()]
        accumulated_matrix = accumulated_matrix @ self.pose_to_matrix(self.kinematic_chain_offset)

        for dh_param, joint_position in zip(self.dh_parameters, joint_positions, strict=False):
            transform = self.dh_transform(dh_param=dh_param, joint_position=joint_position)
            accumulated_matrix = accumulated_matrix @ transform
            position = accumulated_matrix[:3, 3]  # Extract translation (x, y, z)
            positions.append(position.tolist())

        return positions
