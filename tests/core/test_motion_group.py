import pytest
from nova import Nova, lin, ptp


@pytest.mark.asyncio
@pytest.mark.skip
async def test_motion_group():
    nova = Nova(host="172.30.1.65")
    cell = nova.cell()
    controller = await cell.controller("ur")

    path = [
        # from the default script for ur10
        ptp((-91.4, -662.0, 851.3, 2.14, 2.14, -0.357)),
        lin((-160.4, -652.0, 851.3, 2.14, 2.14, -0.357)),
        ptp((-91.4, -462.0, 851.3, 2.14, 2.14, -0.357)),
        lin((-60.4, -652.0, 851.3, 2.14, 2.14, -0.357)),
        ptp((-91.4, -662.0, 851.3, 2.14, 2.14, -0.357)),
    ] * 5

    async with controller:
        motion_group = controller[0]
        state = await motion_group.get_state("Flange")
        print(state)

        motion_iter = motion_group.stream_move(path=path, tcp="Flange")
        async for motion_state in motion_iter:
            print(motion_state)
        assert True
