import pytest

from nova import Nova
from nova.actions import cartesian_ptp, linear
from nova.core.motion_group import split_actions_into_batches


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
