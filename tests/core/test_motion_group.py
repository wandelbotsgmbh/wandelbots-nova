import pytest

from nova import Nova
from nova.actions import cartesian_ptp, linear
from nova.actions.motions import CollisionFreeMotion
from nova.core.motion_group import split_actions_into_batches
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
