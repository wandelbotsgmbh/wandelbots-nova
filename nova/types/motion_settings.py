import pydantic
import wandelbots_api_client as wb


class MotionSettings(pydantic.BaseModel):
    """
    Settings for an action. This is closely related to the `MotionCommand` in the API.
    See planTrajectory.motion_commands for more information.

    Motion settings are immutable; if you need to change a setting, create a copy and update the new object.

    Attributes:
        min_blending_velocity:
            A minimum velocity for blending, in percent. Cannot be used if `blending` is set.

        position_zone_radius:
            Defines the position zone radius.

        joint_velocity_limits:
            Maximum joint velocities

        joint_acceleration_limits:
            Maximum joint accelerations

        tcp_velocity_limit:
            Maximum TCP velocity

        tcp_acceleration_limit:
            Maximum TCP acceleration

        tcp_orientation_velocity_limit:
            Maximum TCP orientation velocity

        tcp_orientation_acceleration_limit:
            Maximum TCP orientation acceleration
    """

    min_blending_velocity: int | None = pydantic.Field(default=None)
    position_zone_radius: float | None = pydantic.Field(default=None)
    joint_velocity_limits: tuple[float, ...] | None = pydantic.Field(default=None)
    joint_acceleration_limits: tuple[float, ...] | None = pydantic.Field(default=None)
    tcp_velocity_limit: float | None = pydantic.Field(default=50)
    tcp_acceleration_limit: float | None = pydantic.Field(default=None)
    tcp_orientation_velocity_limit: float | None = pydantic.Field(default=None)
    tcp_orientation_acceleration_limit: float | None = pydantic.Field(default=None)

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

    def as_limits_settings(self) -> wb.models.LimitsOverride:
        return wb.models.LimitsOverride(
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

    def as_blending_setting(self) -> wb.models.MotionCommandBlending:
        if self.position_zone_radius:
            return wb.models.MotionCommandBlending(
                wb.models.BlendingPosition(
                    position_zone_radius=self.position_zone_radius, blending_name="BlendingPosition"
                )
            )
        return wb.models.MotionCommandBlending(
            wb.models.BlendingAuto(
                min_velocity_in_percent=self.min_blending_velocity, blending_name="BlendingAuto"
            )
        )
