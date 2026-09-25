"""The public surface callers had before the URDF renderer still holds.

The GLB-driven renderer is gone, but a caller's arguments and imports are not
its business: what used to be two collision flags is one setting now, and a
tool mesh moved into the URDF. Both are accepted here and mapped, so old code
runs. What cannot be kept is listed at the bottom.
"""

import warnings

import pytest

from nova import viewers
from nova_rerun_bridge import hull_visualizer, model_loader, motion_storage  # noqa: F401
from nova_rerun_bridge.collision_scene import extract_link_chain_and_tcp
from nova_rerun_bridge.robot_visualizer import RobotVisualizer
from nova_rerun_bridge.trajectory import log_tcp_pose


class TestDeprecatedCollisionFlags:
    def test_viewer_maps_the_two_old_flags(self):
        with pytest.warns(DeprecationWarning):
            viewer = viewers.Rerun(
                spawn=False, show_collision_link_chain=False, show_collision_tool=False
            )
        assert viewer.show_collision is False
        viewer.cleanup()

    def test_one_of_them_left_on_keeps_collision_on(self):
        with pytest.warns(DeprecationWarning):
            viewer = viewers.Rerun(
                spawn=False, show_collision_link_chain=False, show_collision_tool=True
            )
        assert viewer.show_collision is True
        viewer.cleanup()

    def test_new_flag_wins_and_no_warning_without_the_old_ones(self):
        with warnings.catch_warnings():
            warnings.simplefilter("error", DeprecationWarning)
            viewer = viewers.Rerun(spawn=False, show_collision=True)
        assert viewer.show_collision is True
        viewer.cleanup()

    def test_visualizer_takes_the_old_arguments(self):
        with pytest.warns(DeprecationWarning):
            visualizer = RobotVisualizer(
                show_collision_link_chain=False,
                show_collision_tool=False,
                collision_link_chain=None,
                collision_tcp=None,
                model_data=None,
            )
        assert visualizer.show_collision is False
        assert visualizer.collision_link_geometries == []
        assert visualizer.collision_tcp_geometries == {}


class TestKeptHelpers:
    def test_tool_asset_is_still_accepted(self):
        """The mesh rides the robot from the URDF; the argument stays harmless."""
        log_tcp_pose(tcp_poses=[], motion_group_id="0@robot", times_column=None, tool_asset="t.stl")

    def test_extract_link_chain_and_tcp_on_an_empty_setup(self):
        assert extract_link_chain_and_tcp({}) == (None, None)

    def test_hull_outlines_from_geometries_needs_four_points(self):
        assert hull_visualizer.HullVisualizer.compute_hull_outlines_from_geometries([]) == []

    def test_forward_kinematics_without_a_robot(self):
        assert RobotVisualizer().compute_forward_kinematics([0.0]) == []


def test_the_glb_era_internals_are_gone():
    """Everything that only existed to render a downloaded mesh.

    Three walked the GLB scene graph itself, so nothing could stand in for
    them. The rest were helpers of that renderer that happened to be reachable:
    an axis swap, a material tweak, a mesh logger, and three bits of maths over
    ``DHRobot`` and ``scipy``. ``compute_forward_kinematics`` stays -- it is DH,
    not mesh, and reads as something a caller would ask a visualizer for.
    """
    for name in (
        "discover_joints",
        "get_nodes_on_same_layer",
        "init_geometry",
        "get_dh_theta_mesh_correction",
        "geometry_pose_to_matrix",
        "get_transform_matrix",
        "gamma_lift_single_color",
        "rotation_matrix_to_axis_angle",
        "init_mesh",
        "init_collision_geometry",
        "logged_meshes",
        "zero_link_transforms_without_mounting",
        "inverse_mounting_transform",
    ):
        assert not hasattr(RobotVisualizer, name)
