from abc import ABC, abstractmethod
from typing import Annotated, Any, AsyncGenerator, Callable, Literal, Union

import pydantic
import wandelbots_api_client as wb

from nova.types.collision_scene import CollisionScene
from nova.types.pose import Pose


class Action(pydantic.BaseModel, ABC):
    @abstractmethod
    @pydantic.model_serializer
    def serialize_model(self):
        """Serialize the model to a dictionary"""


class WriteAction(Action):
    type: Literal["Write"] = "Write"
    device_id: str
    key: str
    value: Any

    @pydantic.model_serializer
    def serialize_model(self):
        return wb.models.IOValue(io=self.key, boolean_value=self.value).model_dump()


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


class MotionSettings(pydantic.BaseModel):
    """Settings to customize motions.

    Attributes:
        velocity: the cartesian velocity of the TCP on the segment in mm/s
        acceleration: the cartesian acceleration of the TCP on the segment in mm/s
        orientation_velocity: the (tcp) orientation velocity on the segment in rad/s
        orientation_acceleration: the tcp orientation acceleration on the segment in rad/s
        joint_velocities: the joint velocities on the segment in rad/s
        joint_accelerations: the joint accelerations on the segment in rad/s
        blending: the blending radius to connect the previous segment to this one. If blending == 0, blending is
            disabled. If blending == math.inf, blending is enabled and the radius is automatically determined
            based on the robot velocity.
        optimize_approach: define approach axis as DoF and optimize for executable path
    """

    velocity: float | None = pydantic.Field(default=None, ge=0)
    acceleration: float | None = pydantic.Field(default=None, ge=0)
    orientation_velocity: float | None = pydantic.Field(default=None, ge=0)
    orientation_acceleration: float | None = pydantic.Field(default=None, ge=0)
    joint_velocities: tuple[float, ...] | None = None
    joint_accelerations: tuple[float, ...] | None = None
    blending: float = pydantic.Field(default=0, ge=0)
    optimize_approach: bool = False

    @classmethod
    def field_to_varname(cls, field):
        return f"__ms_{field}"


MS = MotionSettings
PoseOrVectorTuple = Union[
    Pose, tuple[float, float, float, float, float, float], tuple[float, float, float]
]


class Motion(Action, ABC):
    """Base model of a motion

    Args:
        type: the type of the motion
        settings: the settings of the motion

    """

    type: Literal["linear", "ptp", "circular", "joint_ptp", "spline"]
    target: Pose | tuple[float, ...]
    settings: MotionSettings = MotionSettings()

    @property
    def is_cartesian(self):
        return isinstance(self.target, Pose)


class UnresolvedMotion(Motion, ABC):
    @abstractmethod
    async def resolve(
        self,
        initial_joints: tuple[float, ...],
        collision_scene: CollisionScene | None,
        configuration: dict,
        moving_robot_identifier: str,
    ) -> tuple[list[Motion], tuple[float, ...]] | None:
        """Convert the motion to a list of motion primitives

        Args:
            initial_joints: Joint positions at start of motion
            collision_scene: The collision scene used to check collisions
            configuration: E.g. data of physical setup of robot system, cell, etc.
            moving_robot_identifier: The identifier of the robot that is moving in the scene

        Returns:
            Tuple of resolved motions and the joint position at the end of the motions. None, if the motion can't be resolved

        """


class Linear(Motion):
    """A linear motion

    Examples:
    >>> Linear(target=Pose((1, 2, 3, 4, 5, 6)), settings=MotionSettings(velocity=10))
    Linear(type='linear', target=Pose(position=Vector3d(x=1, y=2, z=3), orientation=Vector3d(x=4, y=5, z=6)), settings=MotionSettings(velocity=10.0, acceleration=None, orientation_velocity=None, orientation_acceleration=None, joint_velocities=None, joint_accelerations=None, blending=0, optimize_approach=False))
    """

    type: Literal["linear"] = "linear"
    target: Pose

    def _to_api_model(self) -> wb.models.PathLine:
        """Serialize the model to the API model

        Examples:
        >>> Linear(target=Pose((1, 2, 3, 4, 5, 6)), settings=MotionSettings(velocity=10))._to_api_model()
        PathLine(target_pose=Pose2(position=[1, 2, 3], orientation=[4, 5, 6]), path_definition_name='PathLine')
        """
        return wb.models.PathLine(
            target_pose=wb.models.Pose2(**self.target.model_dump()), path_definition_name="PathLine"
        )

    @pydantic.model_serializer
    def serialize_model(self):
        return self._to_api_model().model_dump()


def lin(target: PoseOrVectorTuple, settings: MotionSettings = MotionSettings()) -> Linear:
    """Convenience function to create a linear motion

    Args:
        target: the target pose or vector. If the target is a vector, the orientation is set to (0, 0, 0).
        settings: the motion settings

    Returns: the linear motion

    Examples:
    >>> ms = MotionSettings(velocity=10)
    >>> assert lin((1, 2, 3, 4, 5, 6), settings=ms) == Linear(target=Pose((1, 2, 3, 4, 5, 6)), settings=ms)
    >>> assert lin((1, 2, 3)) == lin((1, 2, 3, 0, 0, 0))

    """
    t = (*target, 0.0, 0.0, 0.0) if len(target) == 3 else target
    return Linear(target=Pose(t), settings=settings)


class PTP(Motion):
    """A point-to-point motion

    Examples:
    >>> PTP(target=Pose((1, 2, 3, 4, 5, 6)), settings=MotionSettings(velocity=30))
    PTP(type='ptp', target=Pose(position=Vector3d(x=1, y=2, z=3), orientation=Vector3d(x=4, y=5, z=6)), settings=MotionSettings(velocity=30.0, acceleration=None, orientation_velocity=None, orientation_acceleration=None, joint_velocities=None, joint_accelerations=None, blending=0, optimize_approach=False))
    """

    type: Literal["ptp"] = "ptp"

    def _to_api_model(self) -> wb.models.PathCartesianPTP:
        """Serialize the model to the API model

        Examples:
        >>> PTP(target=Pose((1, 2, 3, 4, 5, 6)), settings=MotionSettings(velocity=30))._to_api_model()
        PathCartesianPTP(target_pose=Pose2(position=[1, 2, 3], orientation=[4, 5, 6]), path_definition_name='PathCartesianPTP')
        """
        if not isinstance(self.target, Pose):
            raise ValueError("Target must be a Pose object")
        return wb.models.PathCartesianPTP(
            target_pose=wb.models.Pose2(**self.target.model_dump()),
            path_definition_name="PathCartesianPTP",
        )

    @pydantic.model_serializer
    def serialize_model(self):
        return self._to_api_model().model_dump()


def ptp(target: PoseOrVectorTuple, settings: MotionSettings = MotionSettings()) -> PTP:
    """Convenience function to create a point-to-point motion

    Args:
        target: the target pose or vector. If the target is a vector, the orientation is set to (0, 0, 0).
        settings: the motion settings

    Returns: the point-to-point motion

    Examples:
    >>> ms = MotionSettings(acceleration=10)
    >>> assert ptp((1, 2, 3, 4, 5, 6), settings=ms) == PTP(target=Pose((1, 2, 3, 4, 5, 6)), settings=ms)
    >>> assert ptp((1, 2, 3)) == ptp((1, 2, 3, 0, 0, 0))

    """
    if isinstance(target, Pose):
        target = target.to_tuple()

    t = (*target, 0.0, 0.0, 0.0) if len(target) == 3 else target
    return PTP(target=Pose(t), settings=settings)


class Circular(Motion):
    """A circular motion

    Args:
        intermediate: the intermediate pose

    """

    type: Literal["circular"] = "circular"
    intermediate: Pose

    def _to_api_model(self) -> wb.models.PathCircle:
        """Serialize the model to a dictionary

        Examples:
        >>> Circular(target=Pose((1, 2, 3, 4, 5, 6)), intermediate=Pose((10, 20, 30, 40, 50, 60)), settings=MotionSettings(velocity=30))._to_api_model()
        PathCircle(via_pose=Pose2(position=[10, 20, 30], orientation=[40, 50, 60]), target_pose=Pose2(position=[1, 2, 3], orientation=[4, 5, 6]), path_definition_name='PathCircle')
        """
        if not isinstance(self.target, Pose):
            raise ValueError("Target must be a Pose object")
        if not isinstance(self.intermediate, Pose):
            raise ValueError("Intermediate must be a Pose object")
        return wb.models.PathCircle(
            target_pose=wb.models.Pose2(**self.target.model_dump()),
            via_pose=wb.models.Pose2(**self.intermediate.model_dump()),
            path_definition_name="PathCircle",
        )

    @pydantic.model_serializer
    def serialize_model(self):
        return self._to_api_model().model_dump()


def cir(
    target: PoseOrVectorTuple,
    intermediate: PoseOrVectorTuple,
    settings: MotionSettings = MotionSettings(),
) -> Circular:
    """Convenience function to create a circular motion

    Args:
        target: the target pose or vector. If the target is a vector, the orientation is set to (0, 0, 0).
        intermediate: the intermediate pose or vector. If the intermediate is a vector, the orientation is set to
            (0, 0, 0).
        settings: the motion settings

    Returns: the circular motion

    Examples:
    >>> ms = MotionSettings(acceleration=10)
    >>> assert cir((1, 2, 3, 4, 5, 6), (7, 8, 9, 10, 11, 12), settings=ms) == Circular(target=Pose((1, 2, 3, 4, 5, 6)), intermediate=Pose((7, 8, 9, 10, 11, 12)), settings=ms)
    >>> assert cir((1, 2, 3), (4, 5, 6)) == cir((1, 2, 3, 0, 0, 0), (4, 5, 6, 0, 0, 0))

    """
    t = (*target, 0.0, 0.0, 0.0) if len(target) == 3 else target
    i = (*intermediate, 0.0, 0.0, 0.0) if len(intermediate) == 3 else intermediate
    return Circular(target=Pose(t), intermediate=Pose(i), settings=settings)


class JointPTP(Motion):
    """A joint PTP motion

    Examples:
    >>> JointPTP(target=(1, 2, 3, 4, 5, 6), settings=MotionSettings(velocity=30))
    JointPTP(type='joint_ptp', target=(1.0, 2.0, 3.0, 4.0, 5.0, 6.0), settings=MotionSettings(velocity=30.0, acceleration=None, orientation_velocity=None, orientation_acceleration=None, joint_velocities=None, joint_accelerations=None, blending=0, optimize_approach=False))

    """

    type: Literal["joint_ptp"] = "joint_ptp"

    def _to_api_model(self) -> wb.models.PathJointPTP:
        """Serialize the model to the API model

        Examples:
        >>> JointPTP(target=(1, 2, 3, 4, 5, 6, 7), settings=MotionSettings(velocity=30))._to_api_model()
        PathJointPTP(target_joint_position=[1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0], path_definition_name='PathJointPTP')
        """
        if not isinstance(self.target, tuple):
            raise ValueError("Target must be a tuple object")
        return wb.models.PathJointPTP(
            target_joint_position=list(self.target), path_definition_name="PathJointPTP"
        )

    @pydantic.model_serializer
    def serialize_model(self):
        return self._to_api_model().model_dump()


def jnt(target: tuple[float, ...], settings: MotionSettings = MotionSettings()) -> JointPTP:
    """Convenience function to create a joint PTP motion

    Args:
        target: the target joint configuration
        settings: the motion settings

    Returns: the joint PTP motion

    Examples:
    >>> ms = MotionSettings(acceleration=10)
    >>> assert jnt((1, 2, 3, 4, 5, 6), settings=ms) == JointPTP(target=(1, 2, 3, 4, 5, 6), settings=ms)

    """
    return JointPTP(target=target, settings=settings)


class Spline(Motion):
    """A spline motion

    Args:
        path_parameter: the path parameter between 0 and 1
        time: the time in seconds

    """

    type: Literal["spline"] = "spline"
    path_parameter: float = pydantic.Field(1, ge=0)
    time: float | None = pydantic.Field(default=None, ge=0)

    @pydantic.model_serializer
    def serialize_model(self):
        """Serialize the model to a dictionary

        Examples:
        >>> JointPTP(target=(1, 2, 3, 4, 5, 6, 7), settings=MotionSettings(velocity=30)).model_dump()
        {'target_joint_position': [1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0], 'path_definition_name': 'PathJointPTP'}
        """
        raise NotImplementedError("Spline motion is not yet implemented")


def spl(
    target: PoseOrVectorTuple,
    settings: MotionSettings = MotionSettings(),
    path_parameter: float = 1,
    time=None,
) -> Spline:
    """Convenience function to create a spline motion

    Args:
        target: the target pose or vector. If the target is a vector, the orientation is set to (0, 0, 0).
        settings: the motion settings
        path_parameter: the path parameter between 0 and 1
        time: the time in seconds

    Returns: the spline motion

    Examples:
    >>> ms = MotionSettings(acceleration=10)
    >>> assert spl((1, 2, 3, 4, 5, 6), settings=ms) == Spline(target=Pose((1, 2, 3, 4, 5, 6)), settings=ms)
    >>> assert spl((1, 2, 3)) == spl((1, 2, 3, 0, 0, 0))

    """
    t = (*target, 0.0, 0.0, 0.0) if len(target) == 3 else target
    return Spline(target=Pose(t), settings=settings, path_parameter=path_parameter, time=time)


class ActionLocation(pydantic.BaseModel):
    """A container for an action at a specific path parameter"""

    path_parameter: float = 1.0
    action: WriteAction


# TODO: all actions should be allowed (Action)
ActionContainerItem = Motion | WriteAction


class CombinedActions(pydantic.BaseModel):
    """A trajectory of motions and actions"""

    # See: https://docs.pydantic.dev/latest/concepts/serialization/#serialize_as_any-runtime-setting
    items: tuple[
        Annotated[
            pydantic.SerializeAsAny[ActionContainerItem], pydantic.Field(discriminator="type")
        ],
        ...,
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

    def append(self, item: ActionContainerItem):
        super().__setattr__("items", self.items + (item,))

    def _generate_trajectory(self) -> tuple[list[Motion], list[ActionLocation]]:
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
            else:
                # Assign the current value of last_motion_index as path_parameter for actions
                actions.append(ActionLocation(path_parameter=last_motion_index, action=item))

        return motions, actions

    @property
    def motions(self) -> list[Motion]:
        motions, _ = self._generate_trajectory()
        return motions

    @property
    def actions(self) -> list[ActionLocation]:
        _, actions = self._generate_trajectory()
        return actions

    @property
    def start(self) -> ActionContainerItem | None:
        return self.motions[0] if self.motions else None

    @property
    def end(self) -> ActionContainerItem | None:
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

    def __add__(self, other: "CombinedActions") -> "CombinedActions":
        return CombinedActions(items=self.items + other.items)

    def to_motion_command(self) -> list[wb.models.MotionCommand]:
        motions = [
            wb.models.MotionCommandPath.from_dict(motion.model_dump()) for motion in self.motions
        ]
        return [wb.models.MotionCommand(path=motion) for motion in motions]

    def to_set_io(self) -> list[wb.models.SetIO]:
        return [
            wb.models.SetIO(
                io=wb.models.IOValue(**action.action.model_dump(exclude_unset=True)),
                location=action.path_parameter,
            )
            for action in self.actions
        ]


class MovementControllerContext(pydantic.BaseModel):
    combined_actions: CombinedActions
    motion_id: str


ExecuteTrajectoryRequestStream = AsyncGenerator[wb.models.ExecuteTrajectoryRequest, None]
ExecuteTrajectoryResponseStream = AsyncGenerator[wb.models.ExecuteTrajectoryResponse, None]
MovementControllerFunction = Callable[
    [ExecuteTrajectoryResponseStream], ExecuteTrajectoryRequestStream
]
MovementController = Callable[[MovementControllerContext], MovementControllerFunction]
