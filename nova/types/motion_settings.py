import pydantic

from nova import api

DEFAULT_TCP_VELOCITY_LIMIT = 50.0  # mm/s


class MotionSettings(pydantic.BaseModel):
    """
    Settings for an action. This is closely related to the `MotionCommand` in the API.
    See planTrajectory.motion_commands for more information.

    Motion settings are immutable; if you need to change a setting, create a copy and update the new object.

    Attributes:
        blending:
            Blending alters the TCP path at the target point of a motion command to ensure that
            the velocity does not drop to zero between two motion commands.
            Accepts either of the two API blending messages:

            `api.models.BlendingAuto`:
                min_velocity_in_percent:
                    Percentage [0-100] of the original velocity that is kept while blending.

            `api.models.BlendingPosition`:
                position_zone_radius:
                    Maximum radius in [mm] around the target point where the TCP path can be
                    altered to blend into the following motion command.
                position_zone_percentage:
                    Maximum blending percentage [0-100] based on the trajectory length in
                    position space around the target point.
                orientation_zone_radius:
                    Maximum radius in [rad] for orientation blending around the target orientation.
                orientation_zone_percentage:
                    Maximum blending percentage [0-100] for orientation blending based on the
                    trajectory length in orientation space.
                joints_zone_radius:
                    Maximum radius in [rad] for joint space blending around the target joint
                    configuration.
                joints_zone_percentage:
                    Maximum blending percentage [0-100] for joint space blending based on the
                    trajectory length in joint space.
                space:
                    `api.models.BlendingSpace.JOINT` or `api.models.BlendingSpace.CARTESIAN`;
                    defines the space in which blending is performed.

            This setting is not supported for collision-free motions.

        blending_auto:
            Deprecated, use `blending=api.models.BlendingAuto(min_velocity_in_percent=...)`.

        blending_radius:
            Deprecated, use `blending=api.models.BlendingPosition(position_zone_radius=...)`.

        joint_velocity_limits:
            Maximum joint velocity in [rad/s] for each joint.
            Either leave this field empty or set a value for each joint.

        joint_acceleration_limits:
            Maximum joint acceleration in [rad/s^2] for each joint.
            Either leave this field empty or set a value for each joint.

        joint_jerk_limits:
            Maximum joint jerk in [rad/s^3] for each joint.
            Either leave this field empty or set a value for each joint. (experimental)

        tcp_velocity_limit:
            Maximum allowed TCP velocity in [mm/s].

        tcp_acceleration_limit:
            Maximum allowed TCP acceleration in [mm/s^2].

        tcp_jerk_limit:
            Maximum allowed TCP jerk in [mm/s^3]. (experimental)

        tcp_orientation_velocity_limit:
            Maximum allowed TCP rotation velocity in [rad/s].

        tcp_orientation_acceleration_limit:
            Maximum allowed TCP rotation acceleration in [rad/s^2].

        tcp_orientation_jerk_limit:
            Maximum allowed TCP rotation jerk in [rad/s^3]. (experimental)
    """

    blending: api.models.BlendingAuto | api.models.BlendingPosition | None = pydantic.Field(
        default=None, discriminator="blending_name"
    )
    blending_auto: int | None = pydantic.Field(default=None, deprecated=True)
    blending_radius: float | None = pydantic.Field(default=None, deprecated=True)
    joint_velocity_limits: tuple[float, ...] | None = pydantic.Field(default=None)
    joint_acceleration_limits: tuple[float, ...] | None = pydantic.Field(default=None)
    joint_jerk_limits: tuple[float, ...] | None = pydantic.Field(default=None)
    tcp_velocity_limit: float | None = pydantic.Field(default=DEFAULT_TCP_VELOCITY_LIMIT)
    tcp_acceleration_limit: float | None = pydantic.Field(default=None)
    tcp_jerk_limit: float | None = pydantic.Field(default=None)
    tcp_orientation_velocity_limit: float | None = pydantic.Field(default=None)
    tcp_orientation_acceleration_limit: float | None = pydantic.Field(default=None)
    tcp_orientation_jerk_limit: float | None = pydantic.Field(default=None)

    position_zone_radius: float | None = pydantic.Field(default=None, deprecated=True)
    min_blending_velocity: int | None = pydantic.Field(default=None, deprecated=True)

    class Config:
        frozen = True

    @classmethod
    def field_to_varname(cls, field):
        return f"__ms_{field}"

    def _get_blending_radius(self) -> float | None:
        # read through __dict__ so the internal fallback does not raise the field deprecation warning
        if self.__dict__["blending_radius"] is not None:
            return self.__dict__["blending_radius"]
        return self.__dict__["position_zone_radius"]

    def _get_blending_auto(self) -> int | None:
        if self.__dict__["blending_auto"] is not None:
            return self.__dict__["blending_auto"]
        return self.__dict__["min_blending_velocity"]

    @pydantic.model_validator(mode="after")
    def validate_blending_settings(self) -> "MotionSettings":
        blending_radius = self._get_blending_radius()
        blending_auto = self._get_blending_auto()

        if blending_radius is not None and blending_auto is not None:
            raise ValueError("Can't set both blending_radius and blending_auto")

        if self.blending is not None and (blending_radius is not None or blending_auto is not None):
            raise ValueError("Can't set both blending and the deprecated blending settings")

        joint_limits_lengths = [
            len(lim)
            for lim in (
                self.joint_velocity_limits,
                self.joint_acceleration_limits,
                self.joint_jerk_limits,
            )
            if lim is not None
        ]
        if len(set(joint_limits_lengths)) > 1:
            raise ValueError(
                "joint_velocity_limits, joint_acceleration_limits, and joint_jerk_limits must have the same length."
            )
        return self

    def has_blending_settings(self) -> bool:
        return (
            self.blending is not None
            or self._get_blending_auto() is not None
            or self._get_blending_radius() is not None
        )

    def has_limits_override(self) -> bool:
        return any(
            value is not None
            for value in [
                self.tcp_velocity_limit,
                self.tcp_acceleration_limit,
                self.tcp_jerk_limit,
                self.tcp_orientation_velocity_limit,
                self.tcp_orientation_acceleration_limit,
                self.tcp_orientation_jerk_limit,
            ]
        ) or any(
            joint_limits is not None and len(joint_limits) > 0
            for joint_limits in [
                self.joint_velocity_limits,
                self.joint_acceleration_limits,
                self.joint_jerk_limits,
            ]
        )

    def as_limits_settings(self) -> api.models.LimitsOverride:
        return api.models.LimitsOverride(
            joint_velocity_limits=list(self.joint_velocity_limits)
            if self.joint_velocity_limits
            else None,
            joint_acceleration_limits=list(self.joint_acceleration_limits)
            if self.joint_acceleration_limits
            else None,
            joint_jerk_limits=list(self.joint_jerk_limits) if self.joint_jerk_limits else None,
            tcp_velocity_limit=self.tcp_velocity_limit,
            tcp_acceleration_limit=self.tcp_acceleration_limit,
            tcp_jerk_limit=self.tcp_jerk_limit,
            tcp_orientation_velocity_limit=self.tcp_orientation_velocity_limit,
            tcp_orientation_acceleration_limit=self.tcp_orientation_acceleration_limit,
            tcp_orientation_jerk_limit=self.tcp_orientation_jerk_limit,
        )

    def as_blending_setting(self) -> api.models.BlendingPosition | api.models.BlendingAuto:
        if not self.has_blending_settings():
            raise ValueError("No blending settings set")

        if self.blending is not None:
            return self.blending

        blending_radius = self._get_blending_radius()
        if blending_radius is not None:
            return api.models.BlendingPosition(
                position_zone_radius=blending_radius, blending_name="BlendingPosition"
            )
        return api.models.BlendingAuto(
            min_velocity_in_percent=self._get_blending_auto(), blending_name="BlendingAuto"
        )

    def as_tcp_cartesian_limits(self) -> api.models.CartesianLimits:
        return api.models.CartesianLimits(
            velocity=self.tcp_velocity_limit,
            acceleration=self.tcp_acceleration_limit,
            jerk=self.tcp_jerk_limit,
            orientation_velocity=self.tcp_orientation_velocity_limit,
            orientation_acceleration=self.tcp_orientation_acceleration_limit,
            orientation_jerk=self.tcp_orientation_jerk_limit,
        )

    def as_joint_limits(self) -> list[api.models.JointLimits] | None:
        if not any(
            [self.joint_velocity_limits, self.joint_acceleration_limits, self.joint_jerk_limits]
        ):
            return None

        if self.joint_velocity_limits is not None:
            length = len(self.joint_velocity_limits)

        if self.joint_acceleration_limits is not None:
            length = len(self.joint_acceleration_limits)

        if self.joint_jerk_limits is not None:
            length = len(self.joint_jerk_limits)

        limits = []
        for i in range(length):
            # we assume self.joint_velocity_limits and self.joint_acceleration_limits have the same length
            # check the validator
            velocity = (
                self.joint_velocity_limits[i] if self.joint_velocity_limits is not None else None
            )
            acceleration = (
                self.joint_acceleration_limits[i]
                if self.joint_acceleration_limits is not None
                else None
            )
            jerk = self.joint_jerk_limits[i] if self.joint_jerk_limits is not None else None
            limit = api.models.JointLimits(velocity=velocity, acceleration=acceleration, jerk=jerk)
            limits.append(limit)

        return limits
