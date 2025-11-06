import pydantic

from nova import api
from nova.types.pose import Pose


class RobotState(pydantic.BaseModel):
    """Collection of information on the current state of the robot.

    This class represents the complete state of a robot at a given point in time.

    Attributes:
        pose (Pose): The current pose (position and orientation) of the robot.
        tcp (str | None): The TCP name.
        joints (tuple[float, ...]): A tuple of joint angles in radians for each
            joint of the robot.
    """

    pose: Pose
    tcp: str | None
    joints: tuple[float, ...]


class MotionState(pydantic.BaseModel):
    """Collection of information on the current motion state of the robot"""

    motion_group_id: str
    path_parameter: float
    state: RobotState


# TODO this should return different types of MotionState depending on the fields set in the MotionGroupState
def motion_group_state_to_motion_state(
    motion_group_state: api.models.MotionGroupState,
) -> MotionState:
    """Convert a motion group state to a motion state. Should only be used when the motion group is executing a trajectory.

    Args:
        motion_group_state (api.models.MotionGroupState): The motion group state to convert.

    Returns:
        MotionState: The motion state.
    """
    if not motion_group_state.execute:
        raise ValueError("There is no trajectory execution going on.")

    if not isinstance(motion_group_state.execute.details, api.models.TrajectoryDetails):
        raise ValueError("The trajectory execution details are not a trajectory details.")

    tcp_name = motion_group_state.tcp
    if tcp_name is None:
        raise ValueError("There is no TCP attached to the motion group.")

    tcp_pose = motion_group_state.tcp_pose
    if tcp_pose is None:
        raise ValueError("There is no TCP pose attached to the motion group.")

    joints = motion_group_state.joint_position
    path_parameter = motion_group_state.execute.details.location
    return MotionState(
        motion_group_id=motion_group_state.motion_group,
        path_parameter=path_parameter.root,
        state=RobotState(pose=Pose(tcp_pose), tcp=tcp_name, joints=tuple(joints.root)),
    )
