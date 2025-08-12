import builtins

import pytest

import wandelscript
from nova.cell.robot_cell import AbstractController, RobotCellKeyError
from nova.cell.simulation import SimulatedRobotCell
from nova.program.runner import ProgramRunState


@pytest.mark.parametrize(
    "index, exception",
    [
        (-100, builtins.IndexError),
        (-7, builtins.IndexError),
        (0, None),
        (6, builtins.IndexError),
        (100, builtins.IndexError),
        ("'foo'", builtins.TypeError),
        (0.001, builtins.TypeError),
    ],
)
def test_pose_index_access_error(index, exception):
    code = f"""
pose = (0, 0, 10, pi, 0, 0)
a = pose[{index}]
print(a)
"""
    robot_cell = SimulatedRobotCell()
    if exception is None:
        runner = wandelscript.run(program_id="test", program=code, robot_cell_override=robot_cell)
        assert runner.state is ProgramRunState.COMPLETED
    else:
        with pytest.raises(exception):
            wandelscript.run(program_id="test", program=code, robot_cell_override=robot_cell)


@pytest.mark.parametrize(
    "index, exception",
    [
        (-100, builtins.IndexError),
        (-7, builtins.IndexError),
        (6, builtins.IndexError),
        (100, builtins.IndexError),
        ("'foo'", builtins.TypeError),
        (0.001, builtins.TypeError),
    ],
)
def test_assoc_error(index, exception):
    code = f"""
pose = (0, 0, 10, pi, 0, 0)
new_pose = assoc(pose, {index}, 42)
"""
    robot_cell = SimulatedRobotCell()
    with pytest.raises(exception):
        runner = wandelscript.run(program_id="test", program=code, robot_cell_override=robot_cell)
        assert runner.state is ProgramRunState.FAILED


@pytest.mark.asyncio
async def test_get_controller():
    """Assert the happy path of `get_controller()`. I.e., test that it retrieves a controller."""
    robot_cell = SimulatedRobotCell()

    c = robot_cell.get_controller("controller")
    assert isinstance(c, AbstractController)


@pytest.mark.asyncio
async def test_get_controller_with_wrong_name_fails():
    """Assert that retrieving a controller with an unknown name fails with a `RobotCellKeyError`."""
    robot_cell = SimulatedRobotCell()

    with pytest.raises(RobotCellKeyError, match="my-non-existent-controller"):
        robot_cell.get_controller("my-non-existent-controller")


@pytest.mark.asyncio
async def test_get_controller_on_not_controller_fails():
    """Assert that retrieving an object that is not a controller fails with a `ValueError`."""
    robot_cell = SimulatedRobotCell()

    assert not isinstance(robot_cell.timer, AbstractController)

    with pytest.raises(ValueError, match='Found no controller with name "timer".'):
        robot_cell.get_controller("timer")
