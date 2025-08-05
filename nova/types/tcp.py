from pydantic import BaseModel

from nova.types.pose import Pose


# TODO this is subject to change because of the ongoing discussion about id, name, readable_name
class RobotTcp(BaseModel):
    id: str
    name: str  # should be a readable name for now
    pose: Pose
