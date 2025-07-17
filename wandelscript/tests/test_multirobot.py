import pytest

from nova.cell.simulation import SimulatedRobotCell
from wandelscript.exception import NestedSyncError, WrongRobotError
from wandelscript.metamodel import run_program


@pytest.mark.asyncio
async def test_raise_using_sync_in_robotcontext():
    cell = SimulatedRobotCell()
    code = """
Flange = frame("Flange")
do with controller[0]:
    move Flange via p2p() to (100, 200, 300, 0.1, 0.2, 0.3)
    sync
"""
    with pytest.raises(NestedSyncError):
        await run_program(code, cell)


@pytest.mark.asyncio
async def test_raise_if_using_wrong_robot():
    cell = SimulatedRobotCell()
    cell.get_controller("controller")[0].configuration.tools.update(
        {"flange_0": {"position": [0, 0, 0], "orientation": [0, 0, 0]}}
    )
    cell.get_controller("controller")[1].configuration.tools.update(
        {"flange_1": {"position": [0, 0, 0], "orientation": [0, 0, 0]}}
    )
    code = """
flange_0 = frame("flange_0")
flange_1 = frame("flange_1")
do with controller[0]:
    move flange_0 via p2p() to (100, 200, 300, 0.1, 0.2, 0.3)
and do with controller[1]:
    move flange_0 via p2p() to (100, 200, 300, 0.1, 0.2, 0.3)
    move flange_1 via p2p() to (100, 200, 300, 0.1, 0.2, 0.3)
result = "value_bar"
"""
    with pytest.raises(WrongRobotError):
        await run_program(code, cell)
