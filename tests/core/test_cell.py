from unittest.mock import MagicMock

import pytest
from wandelbots_api_client.models import RobotTcp, RotationAngles, RotationAngleTypes, Vector3d

from nova.cell.cell import Cell
from nova.core.gateway import ApiGateway


@pytest.fixture
def mock_cell():
    """Create a Cell instance for testing."""
    mock_api_gateway = MagicMock(spec=ApiGateway)
    return Cell(api_gateway=mock_api_gateway, cell_id="test_cell")


class TestCellTcpConfigsEqual:
    """Test cases for the _tcp_configs_equal method in Cell class."""

    def test_tcp_configs_equal_identical(self, mock_cell):
        """Test that identical TCP configurations are considered equal."""
        tcp1 = RobotTcp(
            id="test_tcp",
            position=Vector3d(x=0, y=0, z=150),
            rotation=RotationAngles(
                angles=[0, 0, 0], type=RotationAngleTypes.EULER_ANGLES_EXTRINSIC_XYZ
            ),
        )
        tcp2 = RobotTcp(
            id="test_tcp",
            position=Vector3d(x=0, y=0, z=150),
            rotation=RotationAngles(
                angles=[0, 0, 0], type=RotationAngleTypes.EULER_ANGLES_EXTRINSIC_XYZ
            ),
        )

        assert mock_cell._tcp_configs_equal(tcp1, tcp2) is True

    def test_tcp_configs_different_ids(self, mock_cell):
        """Test that TCPs with different IDs are not equal."""
        tcp1 = RobotTcp(
            id="tcp1",
            position=Vector3d(x=0, y=0, z=150),
            rotation=RotationAngles(
                angles=[0, 0, 0], type=RotationAngleTypes.EULER_ANGLES_EXTRINSIC_XYZ
            ),
        )
        tcp2 = RobotTcp(
            id="tcp2",
            position=Vector3d(x=0, y=0, z=150),
            rotation=RotationAngles(
                angles=[0, 0, 0], type=RotationAngleTypes.EULER_ANGLES_EXTRINSIC_XYZ
            ),
        )

        assert mock_cell._tcp_configs_equal(tcp1, tcp2) is False

    @pytest.mark.parametrize("x,y,z", [(10, 0, 150), (0, 10, 150), (0, 0, 100)])
    def test_tcp_configs_different_positions(self, mock_cell, x, y, z):
        """Test that TCPs with different positions are not equal."""
        tcp1 = RobotTcp(
            id="test_tcp",
            position=Vector3d(x=0, y=0, z=150),
            rotation=RotationAngles(
                angles=[0, 0, 0], type=RotationAngleTypes.EULER_ANGLES_EXTRINSIC_XYZ
            ),
        )
        tcp2 = RobotTcp(
            id="test_tcp",
            position=Vector3d(x=x, y=y, z=z),
            rotation=RotationAngles(
                angles=[0, 0, 0], type=RotationAngleTypes.EULER_ANGLES_EXTRINSIC_XYZ
            ),
        )

        assert mock_cell._tcp_configs_equal(tcp1, tcp2) is False

    @pytest.mark.parametrize("angle_diff_index", [0, 1, 2])
    def test_tcp_configs_single_angle_difference(self, mock_cell, angle_diff_index):
        """Test that TCPs with a single different angle are not equal."""
        angles1 = [0.0, 0.0, 0.0]
        angles2 = [0.0, 0.0, 0.0]
        angles2[angle_diff_index] = 0.1

        tcp1 = RobotTcp(
            id="test_tcp",
            position=Vector3d(x=0, y=0, z=150),
            rotation=RotationAngles(
                angles=angles1, type=RotationAngleTypes.EULER_ANGLES_EXTRINSIC_XYZ
            ),
        )
        tcp2 = RobotTcp(
            id="test_tcp",
            position=Vector3d(x=0, y=0, z=150),
            rotation=RotationAngles(
                angles=angles2, type=RotationAngleTypes.EULER_ANGLES_EXTRINSIC_XYZ
            ),
        )

        assert mock_cell._tcp_configs_equal(tcp1, tcp2) is False

    def test_tcp_configs_different_rotation_types(self, mock_cell):
        """Test that TCPs with different rotation types are not equal."""
        tcp1 = RobotTcp(
            id="test_tcp",
            position=Vector3d(x=0, y=0, z=150),
            rotation=RotationAngles(
                angles=[0, 0, 0], type=RotationAngleTypes.EULER_ANGLES_EXTRINSIC_XYZ
            ),
        )
        tcp2 = RobotTcp(
            id="test_tcp",
            position=Vector3d(x=0, y=0, z=150),
            rotation=RotationAngles(
                angles=[0, 0, 0], type=RotationAngleTypes.EULER_ANGLES_INTRINSIC_XYZ
            ),
        )

        assert mock_cell._tcp_configs_equal(tcp1, tcp2) is False

    def test_tcp_configs_multiple_angle_differences(self, mock_cell):
        """Test that TCPs with multiple different angles are not equal."""
        tcp1 = RobotTcp(
            id="test_tcp",
            position=Vector3d(x=0, y=0, z=150),
            rotation=RotationAngles(
                angles=[0.1, 0.2, 0.3], type=RotationAngleTypes.EULER_ANGLES_EXTRINSIC_XYZ
            ),
        )
        tcp2 = RobotTcp(
            id="test_tcp",
            position=Vector3d(x=0, y=0, z=150),
            rotation=RotationAngles(
                angles=[0.4, 0.5, 0.6], type=RotationAngleTypes.EULER_ANGLES_EXTRINSIC_XYZ
            ),
        )

        assert mock_cell._tcp_configs_equal(tcp1, tcp2) is False

    def test_tcp_configs_precision_differences(self, mock_cell):
        """Test that TCPs with small precision differences in angles are not equal."""
        tcp1 = RobotTcp(
            id="test_tcp",
            position=Vector3d(x=0, y=0, z=150),
            rotation=RotationAngles(
                angles=[1.0, 2.0, 3.0], type=RotationAngleTypes.EULER_ANGLES_EXTRINSIC_XYZ
            ),
        )
        tcp2 = RobotTcp(
            id="test_tcp",
            position=Vector3d(x=0, y=0, z=150),
            rotation=RotationAngles(
                angles=[1.0001, 2.0, 3.0], type=RotationAngleTypes.EULER_ANGLES_EXTRINSIC_XYZ
            ),
        )

        assert mock_cell._tcp_configs_equal(tcp1, tcp2) is False
