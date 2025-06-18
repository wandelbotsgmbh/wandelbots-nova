from dataclasses import dataclass

from nova.actions import cartesian_ptp, circular, joint_ptp, linear
from nova.actions.motions import CartesianPTP, Circular, JointPTP, Linear
from nova.types import MotionSettings, Pose, Vector3d
from wandelscript.exception import GenericRuntimeError
from wandelscript.metamodel import Connector


@dataclass(repr=False)
class JointPointToPoint(Connector.Impl, func_name="joint_ptp"):
    def __call__(
        self,
        start: Pose | None,
        end: tuple[float, ...],
        args: Connector.Impl.Args,
        motion_settings: MotionSettings,
    ) -> JointPTP:
        return joint_ptp(end, settings=motion_settings)


@dataclass(repr=False)
class JointPoint2Point(JointPointToPoint, func_name="joint_p2p"):
    def __call__(
        self,
        start: Pose | None,
        end: tuple[float, ...],
        args: Connector.Impl.Args,
        motion_settings: MotionSettings,
    ) -> JointPTP:
        return joint_ptp(end, settings=motion_settings)


@dataclass(repr=False)
class Line(Connector.Impl, func_name="line"):
    def __call__(
        self,
        start: Pose | None,
        end: Pose,
        args: Connector.Impl.Args,
        motion_settings: MotionSettings,
    ) -> Linear:
        return linear(end.to_tuple(), settings=motion_settings)


@dataclass(repr=False)
class PointToPoint(Connector.Impl, func_name="ptp"):
    def __call__(
        self,
        start: Pose | None,
        end: Pose,
        args: Connector.Impl.Args,
        motion_settings: MotionSettings,
    ) -> CartesianPTP:
        return cartesian_ptp(end.to_tuple(), settings=motion_settings)


@dataclass(repr=False)
class Point2Point(PointToPoint, func_name="p2p"):
    def __call__(
        self,
        start: Pose | None,
        end: Pose,
        args: Connector.Impl.Args,
        motion_settings: MotionSettings,
    ) -> CartesianPTP:
        return cartesian_ptp(end.to_tuple(), settings=motion_settings)


@dataclass(repr=False)
class Arc(Connector.Impl, func_name="arc"):
    @dataclass
    class Args(Connector.Impl.Args):
        intermediate: Vector3d | Pose

    def __call__(
        self, start: Pose | None, end: Pose, args: Args, motion_settings: MotionSettings
    ) -> Circular:
        if isinstance(args.intermediate, Pose):
            intermediate = args.intermediate
        else:
            raise GenericRuntimeError(location=None, text="Intermediate must be a pose")
        return circular(end.to_tuple(), intermediate.to_tuple(), settings=motion_settings)
