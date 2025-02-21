import numpy as np

from nova.api import models


class DHRobot:
    """A class for handling DH parameters and computing joint positions."""

    def __init__(self, dh_parameters: list[models.DHParameter], mounting: models.PlannerPose):
        """
        Initialize the DHRobot with DH parameters and a mounting pose.
        :param dh_parameters: List of DHParameter objects containing all joint configurations.
        :param mounting: PlannerPose object representing the mounting orientation and position.
        """
        self.dh_parameters = dh_parameters
        self.mounting = mounting

    def pose_to_matrix(self, pose: models.PlannerPose):
        """
        Convert a PlannerPose (with quaternion orientation) into a 4x4 homogeneous transformation matrix.
        :param pose: A PlannerPose object with position: Vector3d and orientation: Quaternion.
        :return: A 4x4 numpy array representing the transformation.
        """
        # Extract translation
        if pose.position is None:
            x, y, z = 0.0, 0.0, 0.0
        else:
            x, y, z = pose.position.x, pose.position.y, pose.position.z

        # Extract quaternion
        if pose.orientation is None:
            # If no orientation is provided, assume identity orientation
            w, qx, qy, qz = 1.0, 0.0, 0.0, 0.0
        else:
            w = pose.orientation.w
            qx = pose.orientation.x
            qy = pose.orientation.y
            qz = pose.orientation.z

        # Compute rotation matrix from quaternion
        # R = [[1 - 2(y²+z²), 2(xy - zw),     2(xz + yw)    ],
        #      [2(xy + zw),     1 - 2(x²+z²), 2(yz - xw)    ],
        #      [2(xz - yw),     2(yz + xw),   1 - 2(x²+y²)]]

        xx = qx * qx
        yy = qy * qy
        zz = qz * qz
        xy = qx * qy
        xz = qx * qz
        yz = qy * qz
        wx = w * qx
        wy = w * qy
        wz = w * qz

        R = np.array(
            [
                [1.0 - 2.0 * (yy + zz), 2.0 * (xy - wz), 2.0 * (xz + wy)],
                [2.0 * (xy + wz), 1.0 - 2.0 * (xx + zz), 2.0 * (yz - wx)],
                [2.0 * (xz - wy), 2.0 * (yz + wx), 1.0 - 2.0 * (xx + yy)],
            ]
        )

        # Construct the full homogeneous transformation matrix
        T = np.eye(4)
        T[0:3, 0:3] = R
        T[0:3, 3] = [x, y, z]

        return T

    def dh_transform(self, dh_param: models.DHParameter, joint_rotation):
        """
        Compute the homogeneous transformation matrix for a given DH parameter and joint rotation.
        :param dh_param: A single DH parameter.
        :param joint_rotation: The joint rotation value (in radians).
        :return: A 4x4 homogeneous transformation matrix.
        """
        # Adjust the angle based on rotation direction
        theta = dh_param.theta + joint_rotation * (-1 if dh_param.reverse_rotation_direction else 1)
        d = dh_param.d
        a = dh_param.a
        alpha = dh_param.alpha if dh_param.alpha is not None else 0.0

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

    def calculate_joint_positions(self, joint_values):
        """
        Compute joint positions based on joint values.
        :param joint_values: Object containing joint rotation values as a list in joint_values.joints.
        :return: A list of joint positions as [x, y, z].
        """
        # Incorporate the mounting pose at the start
        accumulated_matrix = self.pose_to_matrix(self.mounting)

        joint_positions = [
            accumulated_matrix[:3, 3].tolist()
        ]  # Base position after mounting is applied

        for dh_param, joint_rotation in zip(self.dh_parameters, joint_values.joints, strict=False):
            transform = self.dh_transform(dh_param, joint_rotation)
            accumulated_matrix = accumulated_matrix @ transform
            position = accumulated_matrix[:3, 3]  # Extract translation (x, y, z)
            joint_positions.append(position.tolist())

        return joint_positions
