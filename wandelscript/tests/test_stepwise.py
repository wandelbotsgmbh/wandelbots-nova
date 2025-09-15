import pytest

from nova.cell.robot_cell import RobotCell
from nova.cell.simulation import SimulatedRobot
from wandelscript.metamodel import Program


@pytest.mark.skip("broken")
@pytest.mark.asyncio
async def test_stepwise():
    a = RobotCell(robot=SimulatedRobot())
    code = """
print("That")
print("is")
print("a")
print("test")
"""
    program = Program.from_code(code)
    steps = 0
    async for _ in program.stepwise(a):
        steps += 1

    assert steps > 30
