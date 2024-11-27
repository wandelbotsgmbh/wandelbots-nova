import pytest
from wandelbots.core.controller import Controller
from wandelbots.types.trajectory import MotionTrajectory
from wandelbots.types.motion import lin, ptp
from wandelbots import use_nova


@pytest.mark.asyncio
async def test_motion_group():
    nova = use_nova("172.30.1.65")

    path = MotionTrajectory(
        items=[
            # from the default script for ur10
            ptp((-91.4, -662.0, 851.3, 2.14, 2.14, -0.357)),
            lin((-160.4, -652.0, 851.3, 2.14, 2.14, -0.357)),
            ptp((-91.4, -462.0, 851.3, 2.14, 2.14, -0.357)),
            lin((-60.4, -652.0, 851.3, 2.14, 2.14, -0.357)),
            ptp((-91.4, -662.0, 851.3, 2.14, 2.14, -0.357)),
        ]
        * 5
    )

    controller = Controller(nova, cell="cell", controller_host="ur")

    async with controller:
        motion_group = controller[0]
        state = await motion_group.get_state("Flange")
        print(state)

        motion_iter = motion_group.stream_move(path=path, tcp="Flange")
        async for motion_state in motion_iter:
            print(motion_state)
        assert True
