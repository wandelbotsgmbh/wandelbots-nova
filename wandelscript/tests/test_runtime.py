import asyncio

import pytest

from nova.actions import cartesian_ptp, circular, io_write, linear
from nova.actions.container import ActionLocation
from nova.cell.robot_cell import RobotCell
from nova.cell.simulation import (
    SimulatedController,
    SimulatedRobotCell,
    get_simulated_robot_configs,
)
from nova.types import Pose
from nova.types.state import MotionState, RobotState
from wandelscript import exception as wsexception
from wandelscript.exception import NameError_
from wandelscript.metamodel import register_debug_func, run_program
from wandelscript.runner import run
from wandelscript.runtime import ActionQueue, ExecutionContext, Store


@pytest.mark.asyncio
async def test_runtime_error():
    code = "a = 1\na = b"
    with pytest.raises(NameError_) as error:
        await run_program(code)
    assert error.value.location.start.line == 2
    assert error.value.location.start.column == 4


def test_debug():
    test_a = None

    @register_debug_func
    def debug(context):
        nonlocal test_a
        test_a = context.store.data["a"]
        # Also the buffered path can be accessed via
        # print(store.environment._record)

    code = """
a = 1
b = 2
move via p2p() to (1, 2, 3, 4, 5, 6)
move via line() to (1, 0, 0, 1, 1, 1)
debug()
"""

    robot_cell = SimulatedRobotCell()
    run(
        program_id="test",
        program=code,
        robot_cell_override=robot_cell,
        default_robot="0@controller",
        default_tcp="Flange",
    )
    assert test_a == 1


@pytest.mark.parametrize(
    "code, num_robots, expected_exception",
    [
        (
            """
move via p2p() to (1, 2, 3, 4, 5, 6)
""",
            1,
            None,
        ),
        (
            """
move via p2p() to (1, 2, 3, 4, 5, 6)
""",
            2,
            wsexception.WrongRobotError,
        ),
        (
            """
move via p2p() to (1, 2, 3, 4, 5, 6)
do with controller[0]:
    move via p2p() to (1, 2, 3, 4, 5, 6)
""",
            1,
            None,
        ),
        (
            """
move via p2p() to (1, 2, 3, 4, 5, 6)
do with controller[0]:
    move via p2p() to (1, 2, 3, 4, 5, 6)
""",
            2,
            wsexception.WrongRobotError,
        ),
    ],
    ids=[
        "top_level_one_robot",
        "top_level_multi_robot",
        "robot_context_one_robot",
        "robot_context_multi_robot",
    ],
)
def test_robot_code_execution(code, num_robots, expected_exception):
    robot_cell = SimulatedRobotCell(
        controller=SimulatedController(
            SimulatedController.Configuration(
                robots=get_simulated_robot_configs(num_robots=num_robots)
            )
        )
    )
    if expected_exception:
        with pytest.raises(expected_exception):
            run(
                program_id="test",
                program=code,
                robot_cell_override=robot_cell,
                default_tcp="Flange",
            )
    else:
        result = run(
            program_id="test", program=code, robot_cell_override=robot_cell, default_tcp="Flange"
        )
        assert result is not None


def test_store_data_dict():
    store = Store()
    store["int"] = 4
    store["float"] = 10.0
    store["str"] = "string"
    store["pose"] = Pose((0, 0, 0, 0, 0, 0))
    assert store.data_dict == {
        "int": 4,
        "float": 10.0,
        "str": "string",
        "pose": Pose((0, 0, 0, 0, 0, 0)).model_dump(),
    }


@pytest.mark.asyncio
async def test_trigger_actions():
    async def motion_iterator():
        yield MotionState(
            motion_group_id="0", path_parameter=0, state=RobotState(pose=Pose((0, 0, 0, 0, 0, 0)))
        )
        yield MotionState(
            motion_group_id="0", path_parameter=1, state=RobotState(pose=Pose((0, 0, 0, 0, 0, 0)))
        )
        yield MotionState(
            motion_group_id="0", path_parameter=2, state=RobotState(pose=Pose((0, 0, 0, 0, 0, 0)))
        )
        yield MotionState(
            motion_group_id="0", path_parameter=3, state=RobotState(pose=Pose((0, 0, 0, 0, 0, 0)))
        )
        yield MotionState(
            motion_group_id="0", path_parameter=4, state=RobotState(pose=Pose((0, 0, 0, 0, 0, 0)))
        )
        yield MotionState(
            motion_group_id="0", path_parameter=5, state=RobotState(pose=Pose((0, 0, 0, 0, 0, 0)))
        )

    actions = [
        ActionLocation(
            path_parameter=0, action=io_write(device_id="controller", key="some_io", value=0.5)
        ),
        ActionLocation(
            path_parameter=3, action=io_write(device_id="controller", key="some_io", value=3.3)
        ),
        ActionLocation(
            path_parameter=5, action=io_write(device_id="controller", key="some_io", value=5.71)
        ),
        ActionLocation(
            path_parameter=5, action=io_write(device_id="controller", key="some_other_io", value=11)
        ),
        ActionLocation(
            path_parameter=19, action=io_write(device_id="controller", key="some_io", value=190)
        ),
    ]

    cell = RobotCell(controller=SimulatedController())
    queue = ActionQueue(ExecutionContext(cell, asyncio.Event()))
    motion_states_with_actions_triggered = [
        motion_state async for motion_state in queue.trigger_actions(motion_iterator(), actions)
    ]
    assert len(motion_states_with_actions_triggered) == 6
    assert await cell["controller"].read("some_io") == 5.71
    assert await cell["controller"].read("some_other_io") == 11


@pytest.mark.asyncio
async def test_run():
    # push first, or use WS
    controller = SimulatedController()
    robot = controller[0]
    cell = RobotCell(controller=controller)
    execution_context = ExecutionContext(cell, asyncio.Event())
    queue = ActionQueue(execution_context)
    motions = [
        linear((400, 0, 0, 0, 0, 0)),
        circular((500, 0, 0), (0, 0, 0)),
        cartesian_ptp((500, 0, 0)),
    ]
    for motion in motions:
        queue.push(motion, tool="Flange", motion_group_id=robot.id)

    await queue._run()
    assert (await robot.get_state("Flange")).pose == Pose((500, 0, 0, 0, 0, 0))
    assert queue.last_pose(robot.id) == Pose((500, 0, 0, 0, 0, 0))
    assert queue._last_motions[robot.id] == motions[-1]
    assert len(queue._record) == 0
