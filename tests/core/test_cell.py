from unittest.mock import AsyncMock, MagicMock

import pytest
from wandelbots_api_client.models import RobotTcp, RotationAngles, RotationAngleTypes, Vector3d

from nova.core.gateway import ApiGateway
from nova.core.motion_group import MotionGroup


@pytest.fixture
def mock_motion_group():
    """Create a MotionGroup instance for testing."""
    mock_api_gateway = MagicMock(spec=ApiGateway)
    mock_api_gateway.virtual_robot_setup_api = AsyncMock()
    return MotionGroup(
        api_gateway=mock_api_gateway, cell="test_cell", motion_group_id="0@test-controller"
    )


class TestMotionGroupEnsureVirtualTcp:
    """Test cases for the ensure_virtual_tcp method in MotionGroup class."""

    @pytest.mark.asyncio
    async def test_ensure_virtual_tcp_creates_new_tcp(self, mock_motion_group):
        """Test that ensure_virtual_tcp creates a new TCP when it doesn't exist."""
        tcp = RobotTcp(
            id="test_tcp",
            position=Vector3d(x=0, y=0, z=150),
            rotation=RotationAngles(
                angles=[0, 0, 0], type=RotationAngleTypes.EULER_ANGLES_EXTRINSIC_XYZ
            ),
        )

        mock_motion_group.tcps = AsyncMock(return_value=[])

        result = await mock_motion_group.ensure_virtual_tcp(tcp)

        assert result == tcp
        mock_motion_group._api_gateway.virtual_robot_setup_api.add_virtual_robot_tcp.assert_called_once_with(
            cell="test_cell", controller="test-controller", id=0, robot_tcp=tcp
        )

    @pytest.mark.asyncio
    async def test_ensure_virtual_tcp_returns_existing_identical_tcp(self, mock_motion_group):
        """Test that ensure_virtual_tcp returns existing TCP when configurations are identical."""
        tcp = RobotTcp(
            id="test_tcp",
            position=Vector3d(x=0, y=0, z=150),
            rotation=RotationAngles(
                angles=[0, 0, 0], type=RotationAngleTypes.EULER_ANGLES_EXTRINSIC_XYZ
            ),
        )

        existing_tcp = RobotTcp(
            id="test_tcp",
            position=Vector3d(x=0, y=0, z=150),
            rotation=RotationAngles(
                angles=[0, 0, 0], type=RotationAngleTypes.EULER_ANGLES_EXTRINSIC_XYZ
            ),
        )

        mock_motion_group.tcps = AsyncMock(return_value=[existing_tcp])

        result = await mock_motion_group.ensure_virtual_tcp(tcp)

        assert result == existing_tcp
        mock_motion_group._api_gateway.virtual_robot_setup_api.add_virtual_robot_tcp.assert_not_called()

    @pytest.mark.asyncio
    async def test_ensure_virtual_tcp_updates_different_tcp(self, mock_motion_group):
        """Test that ensure_virtual_tcp updates TCP when configurations differ."""
        tcp = RobotTcp(
            id="test_tcp",
            position=Vector3d(x=0, y=0, z=150),
            rotation=RotationAngles(
                angles=[0, 0, 0], type=RotationAngleTypes.EULER_ANGLES_EXTRINSIC_XYZ
            ),
        )

        existing_tcp = RobotTcp(
            id="test_tcp",
            position=Vector3d(x=10, y=0, z=150),
            rotation=RotationAngles(
                angles=[0, 0, 0], type=RotationAngleTypes.EULER_ANGLES_EXTRINSIC_XYZ
            ),
        )

        mock_motion_group.tcps = AsyncMock(return_value=[existing_tcp])

        result = await mock_motion_group.ensure_virtual_tcp(tcp)

        assert result == tcp
        mock_motion_group._api_gateway.virtual_robot_setup_api.add_virtual_robot_tcp.assert_called_once_with(
            cell="test_cell", controller="test-controller", id=0, robot_tcp=tcp
        )

    @pytest.mark.parametrize(
        "rotation_type",
        [
            RotationAngleTypes.EULER_ANGLES_EXTRINSIC_XYZ,
            RotationAngleTypes.EULER_ANGLES_INTRINSIC_XYZ,
            RotationAngleTypes.QUATERNION,
            RotationAngleTypes.ROTATION_VECTOR,
        ],
    )
    @pytest.mark.asyncio
    async def test_ensure_virtual_tcp_different_rotation_types(
        self, mock_motion_group, rotation_type
    ):
        """Test that ensure_virtual_tcp works with different rotation types."""
        angles = [0, 0, 0] if rotation_type != RotationAngleTypes.QUATERNION else [0, 0, 0, 1]

        tcp = RobotTcp(
            id="test_tcp",
            position=Vector3d(x=0, y=0, z=150),
            rotation=RotationAngles(angles=angles, type=rotation_type),
        )

        mock_motion_group.tcps = AsyncMock(return_value=[])

        result = await mock_motion_group.ensure_virtual_tcp(tcp)

        assert result == tcp
        mock_motion_group._api_gateway.virtual_robot_setup_api.add_virtual_robot_tcp.assert_called_once_with(
            cell="test_cell", controller="test-controller", id=0, robot_tcp=tcp
        )

    @pytest.mark.asyncio
    async def test_ensure_virtual_tcp_different_rotation_types_not_equal(self, mock_motion_group):
        """Test that TCPs with different rotation types are not considered equal."""
        tcp = RobotTcp(
            id="test_tcp",
            position=Vector3d(x=0, y=0, z=150),
            rotation=RotationAngles(
                angles=[0, 0, 0], type=RotationAngleTypes.EULER_ANGLES_EXTRINSIC_XYZ
            ),
        )

        existing_tcp = RobotTcp(
            id="test_tcp",
            position=Vector3d(x=0, y=0, z=150),
            rotation=RotationAngles(
                angles=[0, 0, 0], type=RotationAngleTypes.EULER_ANGLES_INTRINSIC_XYZ
            ),
        )

        mock_motion_group.tcps = AsyncMock(return_value=[existing_tcp])

        result = await mock_motion_group.ensure_virtual_tcp(tcp)

        assert result == tcp
        mock_motion_group._api_gateway.virtual_robot_setup_api.add_virtual_robot_tcp.assert_called_once_with(
            cell="test_cell", controller="test-controller", id=0, robot_tcp=tcp
        )
