import asyncio

import pytest

from nova.cell.robot_cell import RobotCell, Timer
from nova.cell.simulation import SimulatedRobot
from wandelscript import Program


@pytest.mark.asyncio
@pytest.mark.skip("broken")
async def test_stepwise_execution():
    robot_cell = RobotCell(timer=Timer(), robot=SimulatedRobot())
    code = """
print("start")
a = 3
b = 13
c = a * b
wait 1000
print("end")
wait 1000
print("end2")
wait 1000
print("end2")
wait 1000
print("end2")
move via p2p() to (0, 0, 0, 1, 1, 1)
"""
    program = Program.from_code(code)

    class ProgramStop(Exception):
        pass

    store = program.make_store(robot_cell)

    async def callback():
        if store.event and store.event.is_set():
            raise ProgramStop

    store.environment.interceptor = callback

    store.event = asyncio.Event()

    async with robot_cell:

        async def fm():
            try:
                await program.body(store=store)
            except ProgramStop:
                pass

        # from async_generator import aclosing

        m = fm()
        boo = asyncio.ensure_future(m)  # noqa: F841

        async def f():
            await asyncio.sleep(1)
            print("a")
            store.event.set()
            # m.close()

        baa = asyncio.ensure_future(f())  # noqa: F841
    # loop = asyncio.get_event_loop()
    # loop.run_forever()

    # assert runner.store["c"] == 39
