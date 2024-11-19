import pytest
from wandelbots.nova.instance import use_nova_api
from wandelbots.nova.controller import Controller
from wandelbots.types.trajectory import MotionTrajectory
from wandelbots.types.motion import lin, ptp


@pytest.mark.asyncio
async def test_motion_group():
    path = MotionTrajectory(
        items=[
            ptp((100, 100, 0, 0, 3.41, 0)),
            lin((100, 50, 0, 0, 3.41, 0)),
            lin((100, 50, 100, 0, 3.41, 0)),
            ptp((100, 100, 0, 0, 3.41, 0)),
        ]
    )

    nova_api_client = use_nova_api("172.30.1.111")
    controller = Controller(api_client=nova_api_client, cell="cell", controller_host="ur10e")

    async with controller:
        motion_group = controller[0]
        state = await motion_group.get_state("Flange")
        print(state)

        motion_iter = motion_group.planned_motion(path=path, tcp="Flange")
        async for motion_state in motion_iter:
            print(motion_state)
        assert False
