import rerun as rr
import rerun.blueprint as rrb


def get_blueprint(motion_group_list: list[str]) -> rrb.Blueprint:
    """Create blueprint with 3D view.

    Args:
        motion_group_list: List of motion group names
    """
    contents = ["//**", "motion/**", "collision_scenes/**", "coordinate_system_world/**"] + [
        f"{group}/**" for group in motion_group_list
    ]

    return rrb.Blueprint(
        rrb.Spatial3DView(contents=contents, name="3D Nova", background=[20, 22, 35]),
        collapse_panels=True,
    )


def send_blueprint(motion_group_list: list[str], include_coordinate_system: bool = True) -> None:
    """Send blueprint with 3D view.

    Args:
        motion_group_list: List of motion group names
        include_coordinate_system: Currently unused flag to align with caller expectations.
    """
    rr.send_blueprint(get_blueprint(motion_group_list))
