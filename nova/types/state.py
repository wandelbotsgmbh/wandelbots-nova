import pydantic

from nova.types.pose import Pose


class RobotState(pydantic.BaseModel):
    """Collection of information on the current state of the robot.

    This class represents the complete state of a robot at a given point in time.

    Attributes:
        pose (Pose): The current pose (position and orientation) of the robot.
        tcp (str): The TCP name .
        joints (tuple[float, ...]): A tuple of joint angles in radians for each
            joint of the robot.
    """

    pose: Pose
    tcp: str
    joints: tuple[float, ...]


class MotionState(pydantic.BaseModel):
    """Collection of information on the current motion state of the robot"""

    motion_group_id: str
    path_parameter: float
    state: RobotState
