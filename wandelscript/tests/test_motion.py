import numpy as np
import pytest

import wandelscript
from nova.actions.motions import CartesianPTP, JointPTP, Linear
from nova.cell.robot_cell import RobotCell
from nova.cell.simulation import (
    SimulatedController,
    SimulatedRobot,
    SimulatedRobotCell,
    get_robot_cell,
)
from nova.types import Pose
from wandelscript.exception import ProgramRuntimeError


def test_forbidden_tcp_change_in_one_motion_example():
    code = """
move via ptp() to (0, 0, 0, 0, 0, 0)
move frame("Flange") to (1, 2, 0)
move frame("tool") to (2, 2, 0)
"""
    robot_configuration = SimulatedRobot.Configuration(
        id="0@controller",
        tools={"Flange": Pose((0, 0, 0, 0, np.pi, 0)), "tool": Pose((2, 0, 0, 0, np.pi, 0))},
    )
    controller = SimulatedController(
        SimulatedController.Configuration(robots=[robot_configuration])
    )
    with pytest.raises(ProgramRuntimeError):
        wandelscript.run(
            program_id="test",
            program=code,
            robot_cell_override=SimulatedRobotCell(controller=controller),
            default_robot="0@controller",
        )


def test_no_robot():
    code = """print("hello world")"""
    wandelscript.run(program_id="test", program=code, robot_cell_override=RobotCell())


def test_motion_type_p2p_line():
    code = """
move via ptp() to (1, 0, 626, 0, 0, 0)
move via line() to (2, 0, 1111, 0, 0, 0)
move via ptp() to (3, 0, 626, 0, 0, 0)
sync
move via ptp() to (11, 0, 626, 0, 0, 0)
move via line() to (12, 0, 1111, 0, 0, 0)
sync
move via line() to (21, 0, 1111, 0, 0, 0)
move via ptp() to (23, 0, 626, 0, 0, 0)
"""
    expected_motion_types = [
        [CartesianPTP, Linear, CartesianPTP],
        [CartesianPTP, Linear],
        [Linear, CartesianPTP],
    ]

    cell = get_robot_cell()
    runner = wandelscript.run(
        program_id="test", program=code, robot_cell_override=cell, default_tcp="Flange"
    )

    record_of_commands = runner.execution_context.robot_cell.get_motion_group(
        "0@controller"
    ).record_of_commands

    assert len(record_of_commands) == 3

    for j, path in enumerate(record_of_commands):
        for i, motion in enumerate(path):
            assert isinstance(motion, expected_motion_types[j][i]), (
                f"The point has a wrong motion type - {expected_motion_types[j][i]=} == {type(motion)=}"
            )


@pytest.mark.skip("JointPTP not supported by simulated robot")
def test_motion_type_joint_p2p():
    code = """
move via joint_p2p() to [1, 0, 626, 0, 0, 0]
move via line() to (2, 0, 1111, 0, 0, 0)
move via ptp() to (3, 0, 626, 0, 0, 0)
move via joint_p2p() to [4, 0, 626, 0, 0, 0]
move via joint_p2p() to [5, 0, 626, 0, 0, 0]
move via joint_p2p() to [6, 0, 626, 0, 0, 0]
sync
move via joint_p2p() to [11, 0, 626, 0, 0, 0]
move via line() to (12, 0, 1111, 0, 0, 0)
sync
move via line() to (21, 0, 1111, 0, 0, 0)
move via joint_p2p() to [23, 0, 626, 0, 0, 0]
sync
move via joint_p2p() to [31, 0, 626, 0, 0, 0]
"""
    expected_joint_values = [
        [
            (1, 0, 626, 0, 0, 0),
            None,
            None,
            (4, 0, 626, 0, 0, 0),
            (5, 0, 626, 0, 0, 0),
            (6, 0, 626, 0, 0, 0),
        ],
        [
            # After a sync there will always first be a CartesianPTP motion added (this point has joints of a previous point)
            (6, 0, 626, 0, 0, 0),
            (11, 0, 626, 0, 0, 0),
            None,
        ],
        [
            # After a sync there will always first be a CartesianPTP motion added
            None,
            None,
            (23, 0, 626, 0, 0, 0),
        ],
        [
            # After a sync there will always first be a CartesianPTP motion added
            (23, 0, 626, 0, 0, 0),
            (31, 0, 626, 0, 0, 0),
        ],
    ]

    expected_motion_types = [
        [JointPTP, Linear, CartesianPTP, JointPTP, JointPTP, JointPTP],
        [
            # After a sync there will always first be a CartesianPTP motion added
            JointPTP,
            JointPTP,
            Linear,
        ],
        [
            # After a sync there will always first be a CartesianPTP motion added
            Linear,
            Linear,
            JointPTP,
        ],
        [
            # After a sync there will always first be a CartesianPTP motion added
            JointPTP,
            JointPTP,
        ],
    ]
    # Create a robot cell:
    cell = get_robot_cell()
    # Execute code:
    runner = wandelscript.run(code, robot_cell_override=cell)

    record_of_commands = runner.execution_context.robot_cell.get_motion_group(
        "0@controller"
    ).record_of_commands

    assert len(record_of_commands) == 4

    for j, path in enumerate(record_of_commands):
        for i, motion in enumerate(path):
            # Check the motion type:
            assert isinstance(motion, expected_motion_types[j][i]), (
                f"The point has a wrong motion type - {expected_motion_types[j][i]=} == {type(path[i])=}"
            )

            if isinstance(path[i], JointPTP):
                assert motion.target == expected_joint_values[j][i], (
                    f"The joint values don't match - {expected_joint_values[j][i]=} == {path[i].target=}"
                )


@pytest.mark.skip("JointPTP not supported by simulated robot")
def test_joint_p2p_on_io_write():
    code = """
joints = read(controller[0], "joints")
print(joints)
move via joint_p2p() to joints
write(controller, "10010#0001", True)
move via joint_p2p() to joints
"""
    cell = get_robot_cell()
    runner = wandelscript.run(code, robot_cell_override=cell)
    print(runner.execution_context.store)

    path = runner.execution_context.robot_cell.robot.record_of_commands[0]
    assert path[1].callback is not None
    assert isinstance(path[1], JointPTP)
