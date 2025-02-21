import rerun as rr
import rerun.blueprint as rrb

from nova_rerun_bridge import colors
from nova_rerun_bridge.consts import TIME_INTERVAL_NAME


def configure_joint_line_colors(motion_group: str):
    """
    Log the visualization lines for joint limit boundaries.
    """

    for i in range(1, 7):
        prefix = f"motion/{motion_group}/joint"
        color = colors.colors[i - 1]

        rr.log(
            f"{prefix}_velocity_lower_limit_{i}",
            rr.SeriesLine(color=[176, 49, 40], name=f"joint_velocity_lower_limit_{i}", width=4),
            static=True,
        )
        rr.log(
            f"{prefix}_velocity_upper_limit_{i}",
            rr.SeriesLine(color=[176, 49, 40], name=f"joint_velocity_upper_limit_{i}", width=4),
            static=True,
        )

        rr.log(
            f"{prefix}_acceleration_lower_limit_{i}",
            rr.SeriesLine(color=[176, 49, 40], name=f"joint_acceleration_lower_limit_{i}", width=4),
            static=True,
        )
        rr.log(
            f"{prefix}_acceleration_upper_limit_{i}",
            rr.SeriesLine(color=[176, 49, 40], name=f"joint_acceleration_upper_limit_{i}", width=4),
            static=True,
        )

        rr.log(
            f"{prefix}_position_lower_limit_{i}",
            rr.SeriesLine(color=[176, 49, 40], name=f"joint_position_lower_limit_{i}", width=4),
            static=True,
        )
        rr.log(
            f"{prefix}_position_upper_limit_{i}",
            rr.SeriesLine(color=[176, 49, 40], name=f"joint_position_upper_limit_{i}", width=4),
            static=True,
        )

        rr.log(
            f"{prefix}_torque_limit_{i}",
            rr.SeriesLine(color=[176, 49, 40], name=f"joint_torques_lower_limit_{i}", width=4),
            static=True,
        )

    for i in range(1, 7):
        prefix = f"motion/{motion_group}/joint"
        color = colors.colors[i - 1]

        rr.log(
            f"{prefix}_velocity_{i}",
            rr.SeriesLine(color=color, name=f"joint_velocity_{i}", width=2),
            static=True,
        )
        rr.log(
            f"{prefix}_velocity_{i}",
            rr.SeriesLine(color=color, name=f"joint_velocity_{i}", width=2),
            static=True,
        )

        rr.log(
            f"{prefix}_acceleration_{i}",
            rr.SeriesLine(color=color, name=f"joint_acceleration_{i}", width=2),
            static=True,
        )
        rr.log(
            f"{prefix}_acceleration_{i}",
            rr.SeriesLine(color=color, name=f"joint_acceleration_{i}", width=2),
            static=True,
        )

        rr.log(
            f"{prefix}_position_{i}",
            rr.SeriesLine(color=color, name=f"joint_position_{i}", width=2),
            static=True,
        )
        rr.log(
            f"{prefix}_position_{i}",
            rr.SeriesLine(color=color, name=f"joint_position_{i}", width=2),
            static=True,
        )

        rr.log(
            f"{prefix}_torque_{i}",
            rr.SeriesLine(color=color, name=f"joint_torques_{i}", width=2),
            static=True,
        )


def configure_tcp_line_colors(motion_group: str):
    """
    Configure time series lines for motion data.
    """
    series_specs = [
        ("tcp_velocity", [136, 58, 255], 2),
        ("tcp_acceleration", [136, 58, 255], 2),
        ("tcp_orientation_velocity", [136, 58, 255], 2),
        ("tcp_orientation_acceleration", [136, 58, 255], 2),
        ("time", [136, 58, 255], 2),
        ("location_on_trajectory", [136, 58, 255], 2),
        ("tcp_acceleration_lower_limit", [176, 49, 40], 4),
        ("tcp_acceleration_upper_limit", [176, 49, 40], 4),
        ("tcp_orientation_acceleration_lower_limit", [176, 49, 40], 4),
        ("tcp_orientation_acceleration_upper_limit", [176, 49, 40], 4),
        ("tcp_velocity_limit", [176, 49, 40], 4),
        ("tcp_orientation_velocity_limit", [176, 49, 40], 4),
    ]
    for name, color, width in series_specs:
        rr.log(
            f"motion/{motion_group}/{name}",
            rr.SeriesLine(color=color, name=name, width=width),
            static=True,
        )


def joint_content_lists(motion_group: str):
    """
    Generate content lists for joint-related time series.
    """
    velocity_contents = [f"motion/{motion_group}/joint_velocity_{i}" for i in range(1, 7)]
    velocity_limits = [
        f"motion/{motion_group}/joint_velocity_lower_limit_{i}" for i in range(1, 7)
    ] + [f"motion/{motion_group}/joint_velocity_upper_limit_{i}" for i in range(1, 7)]

    accel_contents = [f"motion/{motion_group}/joint_acceleration_{i}" for i in range(1, 7)]
    accel_limits = [
        f"motion/{motion_group}/joint_acceleration_lower_limit_{i}" for i in range(1, 7)
    ] + [f"motion/{motion_group}/joint_acceleration_upper_limit_{i}" for i in range(1, 7)]

    pos_contents = [f"motion/{motion_group}/joint_position_{i}" for i in range(1, 7)]
    pos_limits = [f"motion/{motion_group}/joint_position_lower_limit_{i}" for i in range(1, 7)] + [
        f"motion/{motion_group}/joint_position_upper_limit_{i}" for i in range(1, 7)
    ]

    torque_contents = [f"motion/{motion_group}/joint_torque_{i}" for i in range(1, 7)]
    torque_limits = [f"motion/{motion_group}/joint_torque_limit_{i}" for i in range(1, 7)]

    return (
        velocity_contents,
        velocity_limits,
        accel_contents,
        accel_limits,
        pos_contents,
        pos_limits,
        torque_contents,
        torque_limits,
    )


def create_tcp_tabs(
    motion_group: str, time_ranges: rrb.VisibleTimeRange, plot_legend: rrb.PlotLegend
) -> rrb.Vertical:
    """Create TCP-related time series views."""
    return rrb.Vertical(
        rrb.TimeSeriesView(
            contents=[
                f"motion/{motion_group}/tcp_velocity/**",
                f"motion/{motion_group}/tcp_velocity_limit/**",
            ],
            name="TCP velocity",
            time_ranges=time_ranges,
            plot_legend=plot_legend,
        ),
        rrb.TimeSeriesView(
            contents=[
                f"motion/{motion_group}/tcp_acceleration/**",
                f"motion/{motion_group}/tcp_acceleration_lower_limit/**",
                f"motion/{motion_group}/tcp_acceleration_upper_limit/**",
            ],
            name="TCP acceleration",
            time_ranges=time_ranges,
            plot_legend=plot_legend,
        ),
        rrb.TimeSeriesView(
            contents=[
                f"motion/{motion_group}/tcp_orientation_velocity/**",
                f"motion/{motion_group}/tcp_orientation_velocity_limit/**",
            ],
            name="TCP orientation velocity",
            time_ranges=time_ranges,
            plot_legend=plot_legend,
        ),
        rrb.TimeSeriesView(
            contents=[
                f"motion/{motion_group}/tcp_orientation_acceleration/**",
                f"motion/{motion_group}/tcp_orientation_acceleration_lower_limit/**",
                f"motion/{motion_group}/tcp_orientation_acceleration_upper_limit/**",
            ],
            name="TCP orientation acceleration",
            time_ranges=time_ranges,
            plot_legend=plot_legend,
        ),
        name="TCP",
    )


def create_joint_tabs(
    motion_group: str, time_ranges: rrb.VisibleTimeRange, plot_legend: rrb.PlotLegend
) -> rrb.Vertical:
    """Create joint-related time series views."""
    (
        velocity_contents,
        velocity_limits,
        accel_contents,
        accel_limits,
        pos_contents,
        pos_limits,
        torque_contents,
        torque_limits,
    ) = joint_content_lists(motion_group)

    return rrb.Vertical(
        rrb.TimeSeriesView(
            contents=velocity_contents + velocity_limits,
            name="Joint velocity",
            time_ranges=time_ranges,
            plot_legend=plot_legend,
        ),
        rrb.TimeSeriesView(
            contents=accel_contents + accel_limits,
            name="Joint acceleration",
            time_ranges=time_ranges,
            plot_legend=plot_legend,
        ),
        rrb.TimeSeriesView(
            contents=pos_contents + pos_limits,
            name="Joint position",
            time_ranges=time_ranges,
            plot_legend=plot_legend,
        ),
        rrb.TimeSeriesView(
            contents=torque_contents + torque_limits,
            name="Joint torque",
            time_ranges=time_ranges,
            plot_legend=plot_legend,
        ),
        name="Joints",
    )


def create_motion_group_tabs(
    motion_group: str, time_ranges: rrb.VisibleTimeRange, plot_legend: rrb.PlotLegend
) -> rrb.Vertical:
    """Create nested tab structure for a motion group."""
    return rrb.Vertical(
        rrb.Tabs(
            create_tcp_tabs(motion_group, time_ranges, plot_legend),
            create_joint_tabs(motion_group, time_ranges, plot_legend),
        ),
        name=f"Motion Group: {motion_group}",
    )


def get_blueprint(motion_group_list: list[str]) -> rrb.Blueprint:
    """Send blueprint with nested tab structure."""
    for motion_group in motion_group_list:
        configure_tcp_line_colors(motion_group)
        configure_joint_line_colors(motion_group)

    contents = ["motion/**", "collision_scenes/**", "coordinate_system_world/**"] + [
        f"{group}/**" for group in motion_group_list
    ]

    time_ranges = rrb.VisibleTimeRange(
        TIME_INTERVAL_NAME,
        start=rrb.TimeRangeBoundary.cursor_relative(seconds=-2),
        end=rrb.TimeRangeBoundary.cursor_relative(seconds=2),
    )
    plot_legend = rrb.PlotLegend(visible=False)

    motion_group_tabs = [
        create_motion_group_tabs(group, time_ranges, plot_legend) for group in motion_group_list
    ]

    # Create overrides to hide collision links for each motion group by default
    overrides = {
        **{
            f"motion/{group}/collision/links": [rrb.components.Visible(False)]
            for group in motion_group_list
        }
    }

    return rrb.Blueprint(
        rrb.Horizontal(
            rrb.Spatial3DView(
                contents=contents, name="3D Nova", background=[20, 22, 35], overrides=overrides
            ),
            rrb.Tabs(
                *motion_group_tabs,
                rrb.TextLogView(origin="/logs/motion", name="Motions"),
                rrb.TextLogView(origin="/logs", name="API Call Logs"),
            ),
            column_shares=[1, 0.3],
        ),
        collapse_panels=True,
    )


def send_blueprint(motion_group_list: list[str]) -> None:
    """Send blueprint with nested tab structure."""
    rr.send_blueprint(get_blueprint(motion_group_list))
