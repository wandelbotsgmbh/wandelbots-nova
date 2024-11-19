import pydantic
from typing import Annotated, Literal, Any
from abc import ABC
from wandelbots.types.motion import Motion
from wandelbots.types.pose import Pose


class Action(pydantic.BaseModel, ABC):
    type: Literal["Write", "Read", "ReadPose", "ReadJoints", "Call"]


class WriteAction(Action):
    type: Literal["Write"] = "Write"
    device_id: str
    key: str
    value: Any


class ReadAction(Action):
    type: Literal["Read"] = "Read"
    device_id: str
    key: str


class ReadPoseAction(Action):
    type: Literal["ReadPose"] = "ReadPose"
    device_id: str
    tcp: str | None = None


class ReadJointsAction(Action):
    type: Literal["ReadJoints"] = "ReadJoints"
    device_id: str


class CallAction(Action):
    type: Literal["Call"] = "Call"
    device_id: str
    key: str
    arguments: list


class ActionContainer(pydantic.BaseModel):
    """A container for an action at a specific path parameter"""

    path_parameter: float = 1.0
    action: WriteAction


# TODO: all actions should be allowed (Action)
MotionTrajectoryItem = Motion | WriteAction


class MotionTrajectory(pydantic.BaseModel):
    """A trajectory of motions and actions"""

    # See: https://docs.pydantic.dev/latest/concepts/serialization/#serialize_as_any-runtime-setting
    items: tuple[
        Annotated[pydantic.SerializeAsAny[MotionTrajectoryItem], pydantic.Field(discriminator="type")], ...
    ] = ()

    def __len__(self):
        return len(self.items)

    def __getitem__(self, item):
        return self.items[item]

    def __setattr__(self, key, value):
        if key == "items":
            raise TypeError("Cannot set items directly")
        super().__setattr__(key, value)

    def __iter__(self):
        return iter(self.items)

    def append(self, item: MotionTrajectoryItem):
        super().__setattr__("items", self.items + (item,))

    def _generate_trajectory(self) -> tuple[list[Motion], list[ActionContainer]]:
        """Generate two lists: one of Motion objects and another of ActionContainer objects,
        where each ActionContainer wraps a non-Motion action with its path parameter.

        The path parameter is the index of the last Motion object in the list of Motion objects.
        S - M - M - A - A - M - M - A - M - M
        0 - 1 - 2 - 3 - 3 - 3 - 4 - 5 - 5 - 6

        Returns:
            tuple: A tuple containing:
                - list of Motion objects from self.items.
                - list of ActionContainer objects with indexed path parameters.
        """
        motions = []
        actions = []
        last_motion_index = 0

        for item in self.items:
            if isinstance(item, Motion):
                motions.append(item)
                last_motion_index += 1  # Increment the motion index for each new Motion
            elif isinstance(item, Action):
                # Assign the current value of last_motion_index as path_parameter for actions
                actions.append(ActionContainer(path_parameter=last_motion_index, action=item))

        return motions, actions

    @property
    def motions(self) -> list[Motion]:
        motions, _ = self._generate_trajectory()
        return motions

    @property
    def actions(self) -> list[ActionContainer]:
        _, actions = self._generate_trajectory()
        return actions

    @property
    def start(self) -> MotionTrajectoryItem | None:
        return self.motions[0] if self.motions else None

    @property
    def end(self) -> MotionTrajectoryItem | None:
        return self.motions[-1] if self.motions else None

    def poses(self) -> list[Pose]:
        """Returns the positions of all motions. If a motion is not a cartesian motion, the position is ignored

        Returns: the positions

        """
        motions, _ = self._generate_trajectory()
        return [
            Pose(position=motion.target.position, orientation=motion.target.orientation)
            for motion in motions
            if motion.is_cartesian and isinstance(motion.target, Pose)
        ]

    def positions(self):
        """Returns the positions of all motions. If a motion is not a cartesian motion, the position is ignored

        Returns: the positions

        """
        return [pose.position for pose in self.poses()]

    def orientations(self):
        """Returns the orientations of all motions. If a motion is not a cartesian motion, the orientation is ignored

        Returns: the orientations

        """
        return [pose.orientation for pose in self.poses()]

    # def to_posetensor(self) -> PoseTensor:
    #     return PoseTensor.from_sequence([pose.to_posetensor() for pose in self.poses()])

    def __add__(self, other: "MotionTrajectory") -> "MotionTrajectory":
        return MotionTrajectory(items=self.items + other.items)
