import pytest
from wandelbots.nova.instance import use_nova_api
from wandelbots.nova.controller import Controller
from wandelbots.types.trajectory import MotionTrajectory
from wandelbots.types.motion import lin, ptp


@pytest.mark.asyncio
async def test_motion_group():
    path = MotionTrajectory(
        items=[
            # from the default script for ur10
            ptp((-91.4, -662.0, 851.3, 2.14, 2.14, -0.357)),
            lin((-60.4, -652.0, 851.3, 2.14, 2.14, -0.357)),
            ptp((-91.4, -662.0, 851.3, 2.14, 2.14, -0.357)),
            lin((-60.4, -652.0, 851.3, 2.14, 2.14, -0.357)),
            ptp((-91.4, -662.0, 851.3, 2.14, 2.14, -0.357)),
        ] * 5
    )

    nova_api_client = use_nova_api("172.30.0.124")
    controller = Controller(api_client=nova_api_client, cell="cell", controller_host="ur")

    async with controller:
        motion_group = controller[0]
        state = await motion_group.get_state("Flange")
        print(state)

        motion_iter = motion_group.planned_motion(path=path, tcp="Flange")
        async for motion_state in motion_iter:
            print(motion_state)
        assert True
