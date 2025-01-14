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

        blending:
            Defines the position zone radius.

        joint_velocities:
            Maximum joint velocities (corresponds to `max_joint_velocity` in the API).

        joint_accelerations:
            Maximum joint accelerations (corresponds to `max_joint_acceleration` in the API).

        velocity:
            Maximum TCP velocity (corresponds to `max_tcp_velocity` in the API).

        acceleration:
            Maximum TCP acceleration (corresponds to `max_tcp_acceleration` in the API).

        orientation_velocity:
            Maximum TCP orientation velocity (corresponds to `max_tcp_orientation_velocity` in the API).

        orientation_acceleration:
            Maximum TCP orientation acceleration (corresponds to `max_tcp_orientation_acceleration` in the API).
    """

    min_blending_velocity: int | None = pydantic.Field(default=None)
    blending: float | None = pydantic.Field(default=None)
    joint_velocities: tuple[float, ...] | None = pydantic.Field(default=None)
    joint_accelerations: tuple[float, ...] | None = pydantic.Field(default=None)
    velocity: float | None = pydantic.Field(default=None)
    acceleration: float | None = pydantic.Field(default=None)
    orientation_velocity: float | None = pydantic.Field(default=None)
    orientation_acceleration: float | None = pydantic.Field(default=None)

    class Config:
        frozen = True

    @pydantic.model_validator(mode="after")
    def validate_blending_settings(self) -> None:
        if self.min_blending_velocity and self.blending:
            raise ValueError("Can't set both min_blending_velocity and blending")

    def has_blending_settings(self) -> bool:
        return any([self.min_blending_velocity, self.blending])

    def has_limits_override(self) -> bool:
        return any(
            [
                self.velocity,
                self.acceleration,
                self.orientation_velocity,
                self.orientation_acceleration,
                self.joint_velocities,
                self.joint_accelerations,
            ]
        )

    def as_limits_settings(self) -> wb.models.LimitsOverride:
        return wb.models.LimitsOverride(
            joint_velocity_limits=wb.models.Joints(joints=self.joint_velocities)  # type: ignore
            if self.joint_velocities
            else None,
            joint_acceleration_limits=wb.models.Joints(joints=self.joint_accelerations)  # type: ignore
            if self.joint_accelerations
            else None,
            tcp_velocity_limit=self.velocity,
            tcp_acceleration_limit=self.acceleration,
            tcp_orientation_velocity_limit=self.orientation_velocity,
            tcp_orientation_acceleration_limit=self.orientation_acceleration,
        )

    def as_blending_setting(self) -> wb.models.MotionCommandBlending:
        if self.blending:
            return wb.models.MotionCommandBlending(
                wb.models.BlendingPosition(
                    position_zone_radius=self.blending, blending_name="BlendingPosition"
                )
            )
        return wb.models.MotionCommandBlending(
            wb.models.BlendingAuto(
                min_velocity_in_percent=self.min_blending_velocity, blending_name="BlendingAuto"
            )
        )
