import pydantic

from nova.types.pose import Pose


class RobotState(pydantic.BaseModel):
    """Collection of information on the current state of the robot"""

    pose: Pose
    joints: tuple[float, ...] | None = None


class MotionState(pydantic.BaseModel):
    """Collection of information on the current state of the robot"""

    motion_group_id: str
    path_parameter: float
    state: RobotState
