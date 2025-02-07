import pytest

from nova import Nova
from nova.actions import lin, ptp


@pytest.mark.skip
@pytest.mark.asyncio
async def test_motion_group(nova_api):
    nova = Nova(host=nova_api)
    cell = nova.cell()
    controller = await cell.controller("ur")

    actions = [
        # from the default script for ur10
        ptp((-91.4, -662.0, 851.3, 2.14, 2.14, -0.357)),
        lin((-160.4, -652.0, 851.3, 2.14, 2.14, -0.357)),
        ptp((-91.4, -462.0, 851.3, 2.14, 2.14, -0.357)),
        lin((-60.4, -652.0, 851.3, 2.14, 2.14, -0.357)),
        ptp((-91.4, -662.0, 851.3, 2.14, 2.14, -0.357)),
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
