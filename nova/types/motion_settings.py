import pydantic

from nova import api

DEFAULT_TCP_VELOCITY_LIMIT = 50.0  # mm/s

class MotionSettings(pydantic.BaseModel):
    """
    Settings for an action. This is closely related to the `MotionCommand` in the API.
    See planTrajectory.motion_commands for more information.

    Motion settings are immutable; if you need to change a setting, create a copy and update the new object.

    Attributes:
        blending_auto:
            Auto-blending is used to keep a constant velocity when blending between two motion commands.
            It changes the TCP path around the target point of the motion command.
            The value represents the percentage of the original velocity.

        blending_radius:
            Specifies the maximum radius in [mm] around the motion command's target point
            where the TCP path can be altered to blend the motion command into the following one.
            If auto-blending blends too much of the resulting trajectory, use position-blending to restrict the blending zone radius.

        joint_velocity_limits:
            Maximum joint velocity in [rad/s] for each joint.
            Either leave this field empty or set a value for each joint.

        joint_acceleration_limits:
            Maximum joint acceleration in [rad/s^2] for each joint.
            Either leave this field empty or set a value for each joint.

        tcp_velocity_limit:
            Maximum allowed TCP velocity in [mm/s].

        tcp_acceleration_limit:
            Maximum allowed TCP acceleration in [mm/s^2].

        tcp_orientation_velocity_limit:
            Maximum allowed TCP rotation velocity in [rad/s].

        tcp_orientation_acceleration_limit:
            Maximum allowed TCP rotation acceleration in [rad/s^2].
    """

    blending_radius: float | None = pydantic.Field(default=None)
    blending_auto: int | None = pydantic.Field(default=None)
    joint_velocity_limits: tuple[float, ...] | None = pydantic.Field(default=None)
    joint_acceleration_limits: tuple[float, ...] | None = pydantic.Field(default=None)
    tcp_velocity_limit: float | None = pydantic.Field(default=DEFAULT_TCP_VELOCITY_LIMIT)
    tcp_acceleration_limit: float | None = pydantic.Field(default=None)
    tcp_orientation_velocity_limit: float | None = pydantic.Field(default=None)
    tcp_orientation_acceleration_limit: float | None = pydantic.Field(default=None)

    # TODO: check with Christoph, should we just rename these? otherwise we need to update validate_blending_settings as well
    # TODO: doc tests are failing because of the two newly added fields, if we rename fix is different
    position_zone_radius: float | None = pydantic.Field(default=None, deprecated=True)
    min_blending_velocity: int | None = pydantic.Field(default=None, deprecated=True)

    class Config:
        frozen = True

    @classmethod
    def field_to_varname(cls, field):
        return f"__ms_{field}"

    @pydantic.model_validator(mode="after")
    def validate_blending_settings(self) -> "MotionSettings":
        if self.min_blending_velocity and self.position_zone_radius:
            raise ValueError("Can't set both min_blending_velocity and blending")

        if self.joint_acceleration_limits is not None and self.joint_velocity_limits is not None:
            if len(self.joint_acceleration_limits) != len(self.joint_velocity_limits):
                raise ValueError(
                    "joint_acceleration_limits and joint_velocity_limits must have the same length."
                )
        return self

    def has_blending_settings(self) -> bool:
        return any(
            [
                self.blending_auto,
                self.blending_radius,
                self.min_blending_velocity,
                self.position_zone_radius,
            ]
        )

    def has_limits_override(self) -> bool:
        return any(
            [
                self.tcp_velocity_limit,
                self.tcp_acceleration_limit,
                self.tcp_orientation_velocity_limit,
                self.tcp_orientation_acceleration_limit,
                self.joint_velocity_limits,
                self.joint_acceleration_limits,
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
            tcp_velocity_limit=self.tcp_velocity_limit,
            tcp_acceleration_limit=self.tcp_acceleration_limit,
            tcp_orientation_velocity_limit=self.tcp_orientation_velocity_limit,
            tcp_orientation_acceleration_limit=self.tcp_orientation_acceleration_limit,
        )

    def as_blending_setting(self) -> api.models.BlendingPosition | api.models.BlendingAuto:
        if self.position_zone_radius:
            return api.models.BlendingPosition(
                position_zone_radius=self.blending_radius or self.position_zone_radius,
                blending_name="BlendingPosition",
            )
        return api.models.BlendingAuto(
            min_velocity_in_percent=self.blending_auto or self.min_blending_velocity,
            blending_name="BlendingAuto",
        )

    def as_tcp_cartesian_limits(self) -> api.models.CartesianLimits:
        return api.models.CartesianLimits(
            velocity=self.tcp_velocity_limit,
            acceleration=self.tcp_acceleration_limit,
            orientation_velocity=self.tcp_orientation_velocity_limit,
            orientation_acceleration=self.tcp_orientation_acceleration_limit,
        )

    def as_joint_limits(self) -> list[api.models.JointLimits] | None:
        if self.joint_velocity_limits is None and self.joint_acceleration_limits is None:
            return None

        if self.joint_velocity_limits is not None:
            length = len(self.joint_velocity_limits)

        if self.joint_acceleration_limits is not None:
            length = len(self.joint_acceleration_limits)

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
            limit = api.models.JointLimits(velocity=velocity, acceleration=acceleration)
            limits.append(limit)

        return limits
