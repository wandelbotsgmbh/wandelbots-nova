from unittest.mock import AsyncMock, MagicMock

import pytest

from nova import api
from nova.actions import cartesian_ptp, io_write, joint_ptp, linear, wait
from nova.actions.base import Action
from nova.cell.motion_group import MotionGroup, split_actions_into_batches
from nova.core.gateway import ApiGateway
from nova.exceptions import InconsistentCollisionScenes
from nova.types import Pose
from nova.utils.collision_setup import compare_collision_setups, validate_collision_setups


@pytest.mark.asyncio
async def test_empty_list():
    assert split_actions_into_batches([]) == []


def create_collision_setup(
    *, radius: float = 1.0, collider_id: str | None = None
) -> api.models.CollisionSetup:
    collider_id = collider_id or f"test_collider_{radius}"
    collider = api.models.Collider(
        id=collider_id,
        shape=api.models.Sphere(radius=radius, position=api.models.Vector3d([radius, 0, 0])),
    )
    return api.models.CollisionSetup(
        colliders=api.models.ColliderDictionary({collider_id: collider})
    )


@pytest.fixture
def collision_setup():
    return create_collision_setup()


@pytest.mark.asyncio
async def test_only_actions():
    # Create only normal actions.
    a1 = linear((-60.4, -652.0, 851.3, 2.14, 2.14, -0.357))
    a2 = cartesian_ptp((-91.4, -462.0, 851.3, 2.14, 2.14, -0.357))
    a3 = linear((10, 20, 30, 1, 2, 3))
    # Expect a single batch containing all the actions.
    assert split_actions_into_batches([a1, a2, a3]) == [[a1, a2, a3]]


@pytest.mark.asyncio
async def test_only_collision_free(collision_setup):
    # Create only collision free motions.
    cfm1 = joint_ptp((1, 2, 3, 4, 5, 6), collision_setup=collision_setup)
    cfm2 = joint_ptp((7, 8, 9, 10, 11, 12), collision_setup=collision_setup)
    # Each collision free motion should be yielded immediately.
    assert split_actions_into_batches([cfm1, cfm2]) == [[cfm1], [cfm2]]


@pytest.mark.asyncio
async def test_collision_free_first(collision_setup):
    # Collision free motion comes first.
    cfm1 = joint_ptp((1, 2, 3, 4, 5, 6), collision_setup=collision_setup)
    a1 = linear((0, 0, 0, 0, 0, 0))
    a2 = cartesian_ptp((1, 1, 1, 1, 1, 1))
    # Expect: first the collision free motion, then the batch of actions.
    assert split_actions_into_batches([cfm1, a1, a2]) == [[cfm1], [a1, a2]]


@pytest.mark.asyncio
async def test_collision_free_last(collision_setup):
    # Collision free motion comes last.
    a1 = linear((0, 0, 0, 0, 0, 0))
    a2 = cartesian_ptp((1, 1, 1, 1, 1, 1))
    cfm1 = joint_ptp((1, 2, 3, 4, 5, 6), collision_setup=collision_setup)
    # Expect: first a batch of actions, then the collision free motion.
    assert split_actions_into_batches([a1, a2, cfm1]) == [[a1, a2], [cfm1]]


@pytest.mark.asyncio
async def test_interleaved(collision_setup):
    # Test interleaved actions and collision free motions.
    a1 = linear((0, 0, 0, 0, 0, 0))
    a2 = cartesian_ptp((1, 1, 1, 1, 1, 1))
    a3 = linear((2, 2, 2, 2, 2, 2))
    cfm1 = joint_ptp((10, 20, 30, 40, 50, 60), collision_setup=collision_setup)
    cfm2 = joint_ptp((70, 80, 90, 100, 110, 120), collision_setup=collision_setup)

    actions = [a1, cfm1, a2, cfm2, a3]
    expected = [[a1], [cfm1], [a2], [cfm2], [a3]]
    assert split_actions_into_batches(actions) == expected


@pytest.mark.asyncio
async def test_multiple_collision_free_in_row(collision_setup):
    # Sequence: [action, collision free, collision free, action]
    a1 = linear((0, 0, 0, 0, 0, 0))
    a2 = cartesian_ptp((1, 1, 1, 1, 1, 1))
    cfm1 = joint_ptp((10, 20, 30, 40, 50, 60), collision_setup=collision_setup)
    cfm2 = joint_ptp((70, 80, 90, 100, 110, 120), collision_setup=collision_setup)
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
async def test_complex_sequence(collision_setup):
    # A more complex sequence mixing several patterns:
    # Sequence: [a1, cfm1, cfm2, a2, a3, cfm3]
    a1 = linear((0, 0, 0, 0, 0, 0))
    a2 = cartesian_ptp((1, 1, 1, 1, 1, 1))
    a3 = linear((2, 2, 2, 2, 2, 2))
    cfm1 = joint_ptp((10, 20, 30, 40, 50, 60), collision_setup=collision_setup)
    cfm2 = joint_ptp((70, 80, 90, 100, 110, 120), collision_setup=collision_setup)
    cfm3 = joint_ptp((130, 140, 150, 160, 170, 180), collision_setup=collision_setup)

    actions = [a1, cfm1, cfm2, a2, a3, cfm3]
    expected = [[a1], [cfm1], [cfm2], [a2, a3], [cfm3]]
    assert split_actions_into_batches(actions) == expected


@pytest.mark.asyncio
async def test_compare_collision_setup():
    collision_setup_1 = create_collision_setup(radius=1)
    collision_setup_2 = create_collision_setup(radius=1)
    collision_setup_3 = create_collision_setup(radius=2)

    assert compare_collision_setups(collision_setup_1, collision_setup_2) is True
    assert compare_collision_setups(collision_setup_1, collision_setup_3) is False


@pytest.mark.asyncio
async def test_split_and_verify_collision_setup():
    def split_and_verify(actions: list[Action]):
        for batch in split_actions_into_batches(actions):
            validate_collision_setups(actions=batch)

    # A complex mixture of actions
    collision_scene_1 = create_collision_setup(radius=1)
    collision_scene_2 = create_collision_setup(radius=1)
    collision_scene_3 = create_collision_setup(radius=1)
    collision_scene_4 = create_collision_setup(radius=1)
    split_and_verify(
        [
            linear(target=(0, 0, 0, 0, 0, 0), collision_setup=collision_scene_1),
            io_write("digital", 0),
            linear(target=(0, 0, 0, 0, 0, 0), collision_setup=collision_scene_1),
            joint_ptp((1, 2, 3, 4, 5, 6), collision_setup=collision_scene_2),
            wait(1),
            linear(Pose((1, 2, 3, 4, 5, 6))),
            joint_ptp((7, 8, 9, 10, 11, 12), collision_setup=collision_scene_3),
            linear(target=(0, 0, 0, 0, 0, 0)),
            joint_ptp((7, 8, 9, 10, 11, 12), collision_setup=collision_scene_4),
        ]
    )

    # This should fail because two consecutive linear motions should't have different collision scenes
    with pytest.raises(InconsistentCollisionScenes):
        split_and_verify(
            [
                linear(target=(0, 0, 0, 0, 0, 0), collision_setup=create_collision_setup(radius=5)),
                linear(target=(1, 2, 3, 4, 5, 6), collision_setup=create_collision_setup(radius=6)),
            ]
        )

    with pytest.raises(Exception):
        split_and_verify(
            [
                linear(target=(0, 0, 0, 0, 0, 0), collision_setup=create_collision_setup(radius=7)),
                linear(target=(1, 2, 3, 4, 5, 6)),
            ]
        )


@pytest.fixture
def mock_motion_group():
    """Create a MotionGroup instance for testing."""
    mock_api_client = MagicMock(spec=ApiGateway)
    mock_api_client.virtual_controller_api = MagicMock()
    mock_api_client.virtual_controller_api.add_virtual_controller_tcp = AsyncMock()
    return MotionGroup(
        api_client=mock_api_client,
        cell="test_cell",
        controller_id="test-controller",
        motion_group_id="0@test-controller",
    )


@pytest.mark.asyncio
async def test_ensure_virtual_tcp_creates_new_tcp(mock_motion_group):
    """Test that ensure_virtual_tcp creates a new TCP when it doesn't exist."""
    tcp = api.models.RobotTcp(
        id="test_tcp",
        position=api.models.Vector3d([0, 0, 150]),
        orientation=api.models.Orientation([0, 0, 0]),
        orientation_type=api.models.OrientationType.ROTATION_VECTOR,
    )

    mock_motion_group.tcps = AsyncMock(side_effect=[[], [tcp]])

    result = await mock_motion_group.ensure_virtual_tcp(tcp)

    assert result == tcp
    mock_motion_group._api_client.virtual_controller_api.add_virtual_controller_tcp.assert_called_once_with(
        cell="test_cell",
        controller="test-controller",
        motion_group="0@test-controller",
        tcp="test_tcp",
        robot_tcp_data=api.models.RobotTcpData(
            name=tcp.name,
            position=tcp.position,
            orientation=tcp.orientation,
            orientation_type=tcp.orientation_type,
        ),
    )


@pytest.mark.asyncio
async def test_ensure_virtual_tcp_returns_existing_identical_tcp(mock_motion_group):
    """Test that ensure_virtual_tcp returns existing TCP when configurations are identical."""
    tcp = api.models.RobotTcp(
        id="test_tcp",
        position=api.models.Vector3d([0, 0, 150]),
        orientation=api.models.Orientation([0, 0, 0]),
        orientation_type=api.models.OrientationType.ROTATION_VECTOR,
    )

    existing_tcp = api.models.RobotTcp(
        id="test_tcp",
        position=api.models.Vector3d([0, 0, 150]),
        orientation=api.models.Orientation([0, 0, 0]),
        orientation_type=api.models.OrientationType.ROTATION_VECTOR,
    )

    mock_motion_group.tcps = AsyncMock(side_effect=[[existing_tcp], [tcp]])

    result = await mock_motion_group.ensure_virtual_tcp(tcp)

    assert result == existing_tcp
    mock_motion_group._api_client.virtual_controller_api.add_virtual_controller_tcp.assert_not_called()


@pytest.mark.asyncio
async def test_ensure_virtual_tcp_updates_different_tcp(mock_motion_group):
    """Test that ensure_virtual_tcp updates TCP when configurations differ."""
    tcp = api.models.RobotTcp(
        id="test_tcp",
        position=api.models.Vector3d([0, 0, 150]),
        orientation=api.models.Orientation([0, 0, 0]),
        orientation_type=api.models.OrientationType.EULER_ANGLES_EXTRINSIC_XYZ,
    )

    existing_tcp = api.models.RobotTcp(
        id="test_tcp",
        position=api.models.Vector3d([10, 0, 150]),
        orientation=api.models.Orientation([0, 0, 0]),
        orientation_type=api.models.OrientationType.ROTATION_VECTOR,
    )

    mock_motion_group.tcps = AsyncMock(side_effect=[[existing_tcp], [tcp]])

    result = await mock_motion_group.ensure_virtual_tcp(tcp)

    assert result == tcp
    mock_motion_group._api_client.virtual_controller_api.add_virtual_controller_tcp.assert_called_once()


@pytest.mark.parametrize(
    "orientation_type",
    [
        api.models.OrientationType.EULER_ANGLES_EXTRINSIC_XYZ,
        api.models.OrientationType.EULER_ANGLES_INTRINSIC_XYZ,
        api.models.OrientationType.QUATERNION,
        api.models.OrientationType.ROTATION_VECTOR,
    ],
)
@pytest.mark.asyncio
async def test_ensure_virtual_tcp_different_rotation_types(mock_motion_group, orientation_type):
    """Test that ensure_virtual_tcp works with different rotation types."""
    angles = (
        [0, 0, 0] if orientation_type != api.models.OrientationType.QUATERNION else [0, 0, 0, 1]
    )

    tcp = api.models.RobotTcp(
        id="test_tcp",
        position=api.models.Vector3d([0, 0, 150]),
        orientation=api.models.Orientation(angles),
        orientation_type=orientation_type,
    )

    mock_motion_group.tcps = AsyncMock(side_effect=[[], [tcp]])

    result = await mock_motion_group.ensure_virtual_tcp(tcp)

    assert result == tcp
    mock_motion_group._api_client.virtual_controller_api.add_virtual_controller_tcp.assert_called_once()


@pytest.mark.asyncio
async def test_ensure_virtual_tcp_different_rotation_types_not_equal(mock_motion_group):
    """Test that TCPs with different rotation types are not considered equal."""
    tcp = api.models.RobotTcp(
        id="test_tcp",
        position=api.models.Vector3d([0, 0, 150]),
        orientation=api.models.Orientation([0, 0, 0]),
        orientation_type=api.models.OrientationType.EULER_ANGLES_EXTRINSIC_XYZ,
    )

    existing_tcp = api.models.RobotTcp(
        id="test_tcp",
        position=api.models.Vector3d([0, 0, 150]),
        orientation=api.models.Orientation([0, 0, 0]),
        orientation_type=api.models.OrientationType.EULER_ANGLES_INTRINSIC_XYZ,
    )

    mock_motion_group.tcps = AsyncMock(side_effect=[[existing_tcp], [tcp]])

    result = await mock_motion_group.ensure_virtual_tcp(tcp)

    assert result == tcp
    mock_motion_group._api_client.virtual_controller_api.add_virtual_controller_tcp.assert_called_once()
