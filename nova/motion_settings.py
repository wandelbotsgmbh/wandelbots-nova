import pydantic
import wandelbots_api_client as wb


class MotionSettings(pydantic.BaseModel):
    # blending settings
    min_blending_velocity: int | None = pydantic.Field(default=None)

    # max_position_zone_radius in the API, from the docs it looks like this is old 'blending' setting
    blending: float | None = pydantic.Field(default=None)

    # limits override
    # max_joint_velocity in the API
    joint_velocities: tuple[float, ...] | None = pydantic.Field(default=None)

    # max_joint_acceleration in the API
    joint_accelerations: tuple[float, ...] | None = pydantic.Field(default=None)

    # max_tcp_velocity in the API
    velocity: float | None = pydantic.Field(default=None)

    # max_tcp_acceleration in the API
    acceleration: float | None = pydantic.Field(default=None)

    # max_tcp_orientation_velocity in the API
    orientation_velocity: float | None = pydantic.Field(default=None)

    # max_tcp_orientation_acceleration in the API
    orientation_acceleration: float | None = pydantic.Field(default=None)

    @classmethod
    def field_to_varname(cls, field):
        return f"__ms_{field}"

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
        # don't allow setting both, API only accepts one at a time
        if self.min_blending_velocity and self.blending:
            raise ValueError("Can't set both min_blending_velocity and blending")

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
