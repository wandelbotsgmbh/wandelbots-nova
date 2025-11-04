import pydantic

from nova import api


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
    tcp_velocity_limit: float | None = pydantic.Field(default=50)
    tcp_acceleration_limit: float | None = pydantic.Field(default=None)
    tcp_orientation_velocity_limit: float | None = pydantic.Field(default=None)
    tcp_orientation_acceleration_limit: float | None = pydantic.Field(default=None)

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
        return self

    def has_blending_settings(self) -> bool:
        return any([self.min_blending_velocity, self.position_zone_radius])

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
            joint_velocity_limits=wb.models.Joints(joints=self.joint_velocity_limits)  # type: ignore
            if self.joint_velocity_limits
            else None,
            joint_acceleration_limits=wb.models.Joints(joints=self.joint_acceleration_limits)  # type: ignore
            if self.joint_acceleration_limits
            else None,
            tcp_velocity_limit=self.tcp_velocity_limit,
            tcp_acceleration_limit=self.tcp_acceleration_limit,
            tcp_orientation_velocity_limit=self.tcp_orientation_velocity_limit,
            tcp_orientation_acceleration_limit=self.tcp_orientation_acceleration_limit,
        )

    def as_blending_setting(self) -> api.models.MotionCommandBlending:
        if self.position_zone_radius:
            return api.models.MotionCommandBlending(
                api.models.BlendingPosition(
                    position_zone_radius=self.blending_radius or self.position_zone_radius,
                    blending_name="BlendingPosition",
                )
            )
        return api.models.MotionCommandBlending(
            api.models.BlendingAuto(
                min_velocity_in_percent=self.blending_auto or self.min_blending_velocity,
                blending_name="BlendingAuto",
            )
        )
