from unittest.mock import AsyncMock, MagicMock

import pytest
import wandelbots_api_client as wb
from wandelbots_api_client.models import RobotTcp, RotationAngles, RotationAngleTypes, Vector3d

from nova import Nova
from nova.actions import cartesian_ptp, io_write, linear, wait
from nova.actions.base import Action
from nova.actions.motions import CollisionFreeMotion
from nova.core.gateway import ApiGateway
from nova.core.motion_group import (
    MotionGroup,
    compare_collision_scenes,
    split_actions_into_batches,
    validate_collision_scenes,
)
from nova.types import Pose


@pytest.mark.skip
@pytest.mark.asyncio
async def test_motion_group(nova_api):
    nova = Nova(host=nova_api)
    cell = nova.cell()
    controller = await cell.controller("ur")

    actions = [
        # from the default script for ur10
        cartesian_ptp((-91.4, -662.0, 851.3, 2.14, 2.14, -0.357)),
        linear((-160.4, -652.0, 851.3, 2.14, 2.14, -0.357)),
        cartesian_ptp((-91.4, -462.0, 851.3, 2.14, 2.14, -0.357)),
        linear((-60.4, -652.0, 851.3, 2.14, 2.14, -0.357)),
        cartesian_ptp((-91.4, -662.0, 851.3, 2.14, 2.14, -0.357)),
    ] * 5

    async with controller:
        motion_group = controller[0]
        tcp = "Flange"
        state = await motion_group.get_state(tcp)
        assert state is not None

        active_tcp_name = await motion_group.active_tcp_name()
        assert active_tcp_name == "Flange"

        await motion_group.plan_and_execute(actions, tcp)
        assert True


@pytest.mark.asyncio
async def test_empty_list():
    assert split_actions_into_batches([]) == []


@pytest.mark.asyncio
async def test_only_actions():
    # Create only normal actions.
    a1 = linear((-60.4, -652.0, 851.3, 2.14, 2.14, -0.357))
    a2 = cartesian_ptp((-91.4, -462.0, 851.3, 2.14, 2.14, -0.357))
    a3 = linear((10, 20, 30, 1, 2, 3))
    # Expect a single batch containing all the actions.
    assert split_actions_into_batches([a1, a2, a3]) == [[a1, a2, a3]]


@pytest.mark.asyncio
async def test_only_collision_free():
    # Create only collision free motions.
    cfm1 = CollisionFreeMotion(target=Pose(1, 2, 3, 4, 5, 6))
    cfm2 = CollisionFreeMotion(target=Pose(7, 8, 9, 10, 11, 12))
    # Each collision free motion should be yielded immediately.
    assert split_actions_into_batches([cfm1, cfm2]) == [[cfm1], [cfm2]]


@pytest.mark.asyncio
async def test_collision_free_first():
    # Collision free motion comes first.
    cfm1 = CollisionFreeMotion(target=Pose(1, 2, 3, 4, 5, 6))
    a1 = linear((0, 0, 0, 0, 0, 0))
    a2 = cartesian_ptp((1, 1, 1, 1, 1, 1))
    # Expect: first the collision free motion, then the batch of actions.
    assert split_actions_into_batches([cfm1, a1, a2]) == [[cfm1], [a1, a2]]


@pytest.mark.asyncio
async def test_collision_free_last():
    # Collision free motion comes last.
    a1 = linear((0, 0, 0, 0, 0, 0))
    a2 = cartesian_ptp((1, 1, 1, 1, 1, 1))
    cfm1 = CollisionFreeMotion(target=Pose(1, 2, 3, 4, 5, 6))
    # Expect: first a batch of actions, then the collision free motion.
    assert split_actions_into_batches([a1, a2, cfm1]) == [[a1, a2], [cfm1]]


@pytest.mark.asyncio
async def test_interleaved():
    # Test interleaved actions and collision free motions.
    a1 = linear((0, 0, 0, 0, 0, 0))
    a2 = cartesian_ptp((1, 1, 1, 1, 1, 1))
    a3 = linear((2, 2, 2, 2, 2, 2))
    cfm1 = CollisionFreeMotion(target=Pose(10, 20, 30, 40, 50, 60))
    cfm2 = CollisionFreeMotion(target=Pose(70, 80, 90, 100, 110, 120))

    actions = [a1, cfm1, a2, cfm2, a3]
    expected = [[a1], [cfm1], [a2], [cfm2], [a3]]
    assert split_actions_into_batches(actions) == expected


@pytest.mark.asyncio
async def test_multiple_collision_free_in_row():
    # Sequence: [action, collision free, collision free, action]
    a1 = linear((0, 0, 0, 0, 0, 0))
    a2 = cartesian_ptp((1, 1, 1, 1, 1, 1))
    cfm1 = CollisionFreeMotion(target=Pose(10, 20, 30, 40, 50, 60))
    cfm2 = CollisionFreeMotion(target=Pose(70, 80, 90, 100, 110, 120))
    # Simulation:
    # - a1 → batch = [a1]
    # - cfm1 with non-empty batch → yield [a1], defer cfm1.
    # - Next, before processing cfm2, yield deferred cfm1.
    # - Process cfm2 (batch empty) → yield it immediately.
    # - a2 → batch = [a2]
    # - End → yield remaining batch [a2]
    actions = [a1, cfm1, cfm2, a2]
    expected = [[a1], [cfm1], [cfm2], [a2]]
    assert split_actions_into_batches(actions) == expected


@pytest.mark.asyncio
async def test_complex_sequence():
    # A more complex sequence mixing several patterns:
    # Sequence: [a1, cfm1, cfm2, a2, a3, cfm3]
    a1 = linear((0, 0, 0, 0, 0, 0))
    a2 = cartesian_ptp((1, 1, 1, 1, 1, 1))
    a3 = linear((2, 2, 2, 2, 2, 2))
    cfm1 = CollisionFreeMotion(target=Pose(10, 20, 30, 40, 50, 60))
    cfm2 = CollisionFreeMotion(target=Pose(70, 80, 90, 100, 110, 120))
    cfm3 = CollisionFreeMotion(target=Pose(130, 140, 150, 160, 170, 180))

    actions = [a1, cfm1, cfm2, a2, a3, cfm3]
    expected = [[a1], [cfm1], [cfm2], [a2, a3], [cfm3]]
    assert split_actions_into_batches(actions) == expected


def mock_collision_scene():
    colliders = MagicMock(spec=dict)
    colliders.__eq__.side_effect = lambda other, self=colliders: other is self
    colliders.__ne__.side_effect = lambda other, self=colliders: other is not self

    motion_groups = MagicMock(spec=dict)
    motion_groups.__eq__.side_effect = lambda other, self=motion_groups: other is self
    motion_groups.__ne__.side_effect = lambda other, self=motion_groups: other is not self

    return wb.models.CollisionScene.model_construct(
        colliders=colliders, motion_groups=motion_groups
    )


@pytest.mark.asyncio
async def test_compare_collision_scene():
    collision_scene_1 = mock_collision_scene()
    collision_scene_2 = mock_collision_scene()

    assert compare_collision_scenes(collision_scene_1, collision_scene_2) is False, (
        "Collision scenes should not be equal"
    )


@pytest.mark.asyncio
async def test_split_and_verify_collision_scene():
    def split_and_verify(actions: list[Action]):
        for batch in split_actions_into_batches(actions):
            validate_collision_scenes(actions=batch)

    # A complex mixture of actions
    collision_scene_1 = mock_collision_scene()
    split_and_verify(
        [
            linear(target=(0, 0, 0, 0, 0, 0), collision_scene=collision_scene_1),
            io_write("digital", 0),
            linear(target=(0, 0, 0, 0, 0, 0), collision_scene=collision_scene_1),
            CollisionFreeMotion(
                target=Pose(1, 2, 3, 4, 5, 6),
                collision_scene=MagicMock(spec=wb.models.CollisionScene),
            ),
            wait(1),
            linear(
                target=(1, 2, 3, 4, 5, 6), collision_scene=MagicMock(spec=wb.models.CollisionScene)
            ),
            CollisionFreeMotion(
                target=Pose(7, 8, 9, 10, 11, 12),
                collision_scene=MagicMock(spec=wb.models.CollisionScene),
            ),
            linear(target=(0, 0, 0, 0, 0, 0)),
            CollisionFreeMotion(target=Pose(7, 8, 9, 10, 11, 12)),
        ]
    )

    # This should fail because two consecutive linear motions should't have different collision scenes
    with pytest.raises(Exception):
        split_and_verify(
            [
                linear(target=(0, 0, 0, 0, 0, 0), collision_scene=mock_collision_scene()),
                linear(target=(1, 2, 3, 4, 5, 6), collision_scene=mock_collision_scene()),
            ]
        )

    with pytest.raises(Exception):
        split_and_verify(
            [
                linear(target=(0, 0, 0, 0, 0, 0), collision_scene=mock_collision_scene()),
                linear(target=(1, 2, 3, 4, 5, 6)),
            ]
        )


@pytest.fixture
def mock_motion_group():
    """Create a MotionGroup instance for testing."""
    mock_api_gateway = MagicMock(spec=ApiGateway)
    mock_api_gateway.virtual_robot_setup_api = AsyncMock()
    return MotionGroup(
        api_gateway=mock_api_gateway, cell="test_cell", motion_group_id="0@test-controller"
    )


@pytest.mark.asyncio
async def test_ensure_virtual_tcp_creates_new_tcp(mock_motion_group):
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
async def test_ensure_virtual_tcp_returns_existing_identical_tcp(mock_motion_group):
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
async def test_ensure_virtual_tcp_updates_different_tcp(mock_motion_group):
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
async def test_ensure_virtual_tcp_different_rotation_types(mock_motion_group, rotation_type):
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
async def test_ensure_virtual_tcp_different_rotation_types_not_equal(mock_motion_group):
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
