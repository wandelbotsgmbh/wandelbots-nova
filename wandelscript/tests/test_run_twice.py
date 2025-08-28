import sys
from datetime import datetime

from icecream import ic

from nova.cell.simulation import SimulatedRobotCell, get_robot_controller
from nova.program.runner import ProgramRunState
from wandelscript import run
from wandelscript.utils.runtime import Tee

robot_cell = SimulatedRobotCell(controller=get_robot_controller())

ic.configureOutput(prefix=lambda: f"{datetime.now().time()} | ", includeContext=True)


def test_run_code_twice():
    code = """
a = 4 + 5
move via p2p() to (0, 0, 400, 0, pi, 0)
print("print something")
move via line() to (0, 100, 400, 0, pi, 0)
"""
    for i in range(2):
        runner = run(
            program_id="test", program=code, robot_cell_override=robot_cell, default_tcp="Flange"
        )
        assert runner.program_run.output_data["a"] == 9
        assert runner.program_run.state is ProgramRunState.COMPLETED
        assert "print something" in runner.program_run.logs
        assert not isinstance(sys.stdout, Tee)
