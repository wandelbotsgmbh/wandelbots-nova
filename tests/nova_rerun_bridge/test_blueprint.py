"""Unit tests for nova_rerun_bridge.blueprint module.

This test suite focuses on business logic and data transformations within the
blueprint module. It tests the core functionality without directly testing
rerun library internals, using mocks to isolate the code under test.

The tests cover:
- Joint content list generation and path formatting
- Blueprint creation and configuration
- Motion group handling and integration
- Error handling and edge cases
"""

from unittest.mock import Mock, patch

import pytest

from nova_rerun_bridge.blueprint import (
    configure_joint_line_colors,
    configure_tcp_line_colors,
    create_joint_tabs,
    get_blueprint,
    joint_content_lists,
    send_blueprint,
)


class TestJointContentLists:
    """Test joint content list generation and path formatting."""

    def test_returns_eight_lists(self):
        """Should return 8 lists for different joint data types."""
        result = joint_content_lists("test_group")
        assert len(result) == 8

    def test_joint_content_structure(self):
        """Should return correctly structured joint content lists."""
        motion_group = "test_group"
        result = joint_content_lists(motion_group)

        (
            velocity_contents,
            velocity_limits,
            accel_contents,
            accel_limits,
            pos_contents,
            pos_limits,
            torque_contents,
            torque_limits,
        ) = result

        # Main content lists should have 6 items (one per joint)
        assert len(velocity_contents) == 6
        assert len(accel_contents) == 6
        assert len(pos_contents) == 6
        assert len(torque_contents) == 6

        # Limit lists should have correct sizes
        assert len(velocity_limits) == 12  # upper + lower for 6 joints
        assert len(accel_limits) == 12
        assert len(pos_limits) == 12
        assert len(torque_limits) == 6  # only one limit type for torque

    def test_path_formatting(self):
        """Should generate correctly formatted paths for all joint types."""
        motion_group = "robot_1"
        result = joint_content_lists(motion_group)

        velocity_contents, velocity_limits = result[0], result[1]
        accel_contents, pos_contents, torque_contents = result[2], result[4], result[6]

        # Check velocity content paths
        expected_velocity_paths = [f"motion/{motion_group}/joint_velocity_{i}" for i in range(1, 7)]
        assert velocity_contents == expected_velocity_paths

        # Check velocity limit paths
        expected_lower_limits = [
            f"motion/{motion_group}/joint_velocity_lower_limit_{i}" for i in range(1, 7)
        ]
        expected_upper_limits = [
            f"motion/{motion_group}/joint_velocity_upper_limit_{i}" for i in range(1, 7)
        ]
        assert velocity_limits == expected_lower_limits + expected_upper_limits

        # Test other joint types
        expected_accel = [f"motion/{motion_group}/joint_acceleration_{i}" for i in range(1, 7)]
        assert accel_contents == expected_accel

        expected_pos = [f"motion/{motion_group}/joint_position_{i}" for i in range(1, 7)]
        assert pos_contents == expected_pos

        expected_torque = [f"motion/{motion_group}/joint_torque_{i}" for i in range(1, 7)]
        assert torque_contents == expected_torque

    def test_joint_numbering_convention(self):
        """Should number joints starting from 1, not 0."""
        result = joint_content_lists("test")
        velocity_contents = result[0]

        # First joint should be numbered 1
        assert "joint_velocity_1" in velocity_contents[0]
        # Last joint should be numbered 6
        assert "joint_velocity_6" in velocity_contents[5]

        # Should not have joint_velocity_0
        for path in velocity_contents:
            assert "joint_velocity_0" not in path

    def test_path_structure_consistency(self):
        """Should generate paths with consistent structure across all types."""
        motion_group = "test_robot"
        result = joint_content_lists(motion_group)

        all_paths = []
        for content_list in result:
            all_paths.extend(content_list)

        for path in all_paths:
            # All paths should start with "motion/"
            assert path.startswith("motion/")
            # Should contain motion group name
            assert motion_group in path
            # Should contain "joint"
            assert "joint" in path

            # Should end with a number 1-6
            path_parts = path.split("_")
            last_part = path_parts[-1]
            assert last_part.isdigit()
            joint_num = int(last_part)
            assert 1 <= joint_num <= 6

    @pytest.mark.parametrize(
        "motion_group",
        [
            "robot_1",
            "arm_left",
            "manipulator_main",
            "",
            "test-group-with-dashes",
            "group_with_underscores",
            "123_numeric_start",
        ],
    )
    def test_motion_group_name_handling(self, motion_group):
        """Should handle various motion group names correctly."""
        result = joint_content_lists(motion_group)

        # Should always return 8 lists
        assert len(result) == 8

        # Each list should have expected length
        velocity_contents, velocity_limits = result[0], result[1]
        assert len(velocity_contents) == 6
        assert len(velocity_limits) == 12

        # Paths should contain the motion group name
        for path in velocity_contents:
            assert motion_group in path
            assert path.startswith("motion/")
            assert "joint_velocity_" in path

    def test_special_character_handling(self):
        """Should handle special characters and unicode in motion group names."""
        special_names = ["robot@123", "robot#test", "robot%special", "robot space", "robot\ttab"]
        unicode_names = ["机器人", "ロボット", "робот"]

        for name in special_names + unicode_names:
            result = joint_content_lists(name)
            assert len(result) == 8

            # Paths should contain the name as-is
            velocity_contents = result[0]
            assert f"motion/{name}/joint_velocity_1" == velocity_contents[0]

    def test_empty_motion_group_handling(self):
        """Should handle empty motion group name gracefully."""
        result = joint_content_lists("")
        assert len(result) == 8

        velocity_contents = result[0]
        assert velocity_contents[0] == "motion//joint_velocity_1"

    def test_joint_count_consistency(self):
        """Should use consistent joint count (6) across all data types."""
        result = joint_content_lists("test")

        # Each main content type should have 6 joints
        velocity_contents, _, accel_contents, _, pos_contents, _, torque_contents, _ = result

        assert len(velocity_contents) == 6
        assert len(accel_contents) == 6
        assert len(pos_contents) == 6
        assert len(torque_contents) == 6

        # All joint ranges should be 1-6
        main_content_indices = [0, 2, 4, 6]
        for idx in main_content_indices:
            content_list = result[idx]
            joint_numbers = [int(path.split("_")[-1]) for path in content_list]
            assert joint_numbers == list(range(1, 7))


class TestCreateJointTabs:
    """Test joint tab creation and integration."""

    @patch("nova_rerun_bridge.blueprint.joint_content_lists")
    def test_calls_joint_content_lists_with_correct_motion_group(self, mock_joint_content):
        """Should call joint_content_lists with the correct motion group."""
        mock_joint_content.return_value = (
            ["vel1"],
            ["vel_lim"],
            ["acc1"],
            ["acc_lim"],
            ["pos1"],
            ["pos_lim"],
            ["torque1"],
            ["torque_lim"],
        )

        motion_group = "test_group"
        time_ranges = Mock()
        plot_legend = Mock()

        with (
            patch("nova_rerun_bridge.blueprint.rrb.TimeSeriesView"),
            patch("nova_rerun_bridge.blueprint.rrb.Vertical"),
        ):
            create_joint_tabs(motion_group, time_ranges, plot_legend)
            mock_joint_content.assert_called_once_with(motion_group)


class TestGetBlueprint:
    """Test blueprint generation and configuration."""

    @patch("nova_rerun_bridge.blueprint.configure_tcp_line_colors")
    @patch("nova_rerun_bridge.blueprint.configure_joint_line_colors")
    def test_configures_colors_for_all_motion_groups(self, mock_joint_colors, mock_tcp_colors):
        """Should configure colors for each motion group in the list."""
        motion_group_list = ["group1", "group2", "group3"]

        with (
            patch("nova_rerun_bridge.blueprint.create_motion_group_tabs"),
            patch("nova_rerun_bridge.blueprint.rrb.Blueprint"),
        ):
            get_blueprint(motion_group_list)

            # Should configure colors for each motion group
            assert mock_tcp_colors.call_count == 3
            assert mock_joint_colors.call_count == 3

            # Check specific calls
            for group in motion_group_list:
                mock_tcp_colors.assert_any_call(group)
                mock_joint_colors.assert_any_call(group)

    def test_handles_empty_motion_group_list(self):
        """Should handle empty motion group list without errors."""
        with patch("nova_rerun_bridge.blueprint.rrb.Blueprint") as mock_blueprint:
            mock_blueprint.return_value = Mock()
            get_blueprint([])
            mock_blueprint.assert_called_once()

    def test_handles_single_motion_group(self):
        """Should handle single motion group correctly."""
        with (
            patch("nova_rerun_bridge.blueprint.configure_tcp_line_colors") as mock_tcp,
            patch("nova_rerun_bridge.blueprint.configure_joint_line_colors") as mock_joint,
            patch("nova_rerun_bridge.blueprint.rrb.Blueprint") as mock_blueprint,
        ):
            mock_blueprint.return_value = Mock()
            get_blueprint(["single_group"])

            mock_tcp.assert_called_once_with("single_group")
            mock_joint.assert_called_once_with("single_group")


class TestSendBlueprint:
    """Test blueprint sending functionality."""

    @patch("nova_rerun_bridge.blueprint.rr.send_blueprint")
    @patch("nova_rerun_bridge.blueprint.get_blueprint")
    def test_sends_generated_blueprint_to_rerun(self, mock_get_blueprint, mock_send):
        """Should call rerun's send_blueprint with the generated blueprint."""
        motion_group_list = ["group1"]
        mock_blueprint = Mock()
        mock_get_blueprint.return_value = mock_blueprint

        send_blueprint(motion_group_list)

        mock_get_blueprint.assert_called_once_with(motion_group_list, True)
        mock_send.assert_called_once_with(mock_blueprint)


class TestMotionGroupIntegration:
    """Test motion group integration across blueprint functions."""

    def test_motion_groups_propagate_through_path_generation(self):
        """Should properly integrate motion groups throughout blueprint creation."""
        motion_groups = ["robot_1", "robot_2"]

        for motion_group in motion_groups:
            result = joint_content_lists(motion_group)

            # Extract all paths
            all_paths = []
            for content_list in result:
                all_paths.extend(content_list)

            # All paths should contain the specific motion group
            for path in all_paths:
                assert f"motion/{motion_group}/" in path


class TestErrorHandling:
    """Test error handling and edge cases."""

    @patch("nova_rerun_bridge.blueprint.rr.log")
    def test_configure_functions_handle_empty_motion_group(self, mock_log):
        """Should handle empty motion group names without errors."""
        configure_joint_line_colors("")
        configure_tcp_line_colors("")

        # Should still make log calls
        assert mock_log.call_count > 0
