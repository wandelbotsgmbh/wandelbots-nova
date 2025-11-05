import pydantic

from nova import api
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


# TODO find a better place for this
# TODO this should return different types of MotionState depending on the fields set in the MotionGroupState
def motion_group_state_to_motion_state(
    motion_group_state: api.models.MotionGroupState,
) -> MotionState:
    if not motion_group_state.execute:
        raise ValueError("There is no trajectory execution going on.")

    if motion_group_state.tcp is None:
        raise ValueError("There is no TCP attached to the motion group.")

    tcp_pose = Pose(
        tuple(motion_group_state.tcp_pose.position) + tuple(motion_group_state.tcp_pose.orientation)
    )
    joints = tuple(motion_group_state.joint_position)
    # TODO not very clean
    path_parameter = (
        motion_group_state.execute.details.location
        if motion_group_state.execute
        and motion_group_state.execute.details
        and isinstance(motion_group_state.execute.details, api.models.TrajectoryDetails)
        else None
    )
    return MotionState(
        motion_group_id=motion_group_state.motion_group,
        path_parameter=path_parameter,
        state=RobotState(pose=tcp_pose, tcp=motion_group_state.tcp, joints=joints),
    )
