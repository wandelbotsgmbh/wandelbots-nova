import pytest

import nova
from nova import Nova, api, run_program
from nova.actions import cartesian_ptp, joint_ptp
from nova.cell import virtual_controller
from nova.program import ProgramPreconditions
from nova.types import Pose


@pytest.mark.integration
async def test_program_runner_with_unrelated_controller_in_estop():
    preconditions = ProgramPreconditions(
        controllers=[
            virtual_controller(
                name="kuka-no-estop",
                manufacturer=api.models.Manufacturer.KUKA,
                type=api.models.VirtualControllerTypes.KUKA_KR16_R1610_2,
            )
        ],
        cleanup_controllers=False,
    )

    @nova.program(preconditions=preconditions)
    async def test_program():
        async with Nova() as nova:
            cell = nova.cell()
            controller = await cell.controller("kuka-no-estop")

            async with controller[0] as motion_group:
                home_joints = await motion_group.joints()
                tcp_names = await motion_group.tcp_names()
                tcp = tcp_names[0]
                target_pose = await motion_group.tcp_pose(tcp)

                actions = [
                    joint_ptp(home_joints),
                    cartesian_ptp(target_pose @ Pose((100, 0, 0, 0, 0, 0))),
                    joint_ptp(home_joints),
                ]
                await motion_group.plan_and_execute(actions, tcp)

    # Set up another controller in the cell in estop before running the program
    nova_instance = Nova()
    await nova_instance.connect()
    cell = nova_instance.cell()
    controller_in_estop = await cell.ensure_controller(
        virtual_controller(
            name="ur10e-estop",
            manufacturer=api.models.Manufacturer.UNIVERSALROBOTS,
            type=api.models.VirtualControllerTypes.UNIVERSALROBOTS_UR10E,
        )
    )
    # Set the controller in estop
    await controller_in_estop.set_estop(active=True)

    # Check if the program starts even if one (unused) controller is in estop
    runner = run_program(test_program)
    assert runner.state == api.models.ProgramRunState.COMPLETED
    assert runner.program_run.error is None

    # Check if the program runner has the correct preconditions & robot cell
    assert runner._preconditions == preconditions
    # Timer, cycle timer and the kuka controller
    assert len(runner.execution_context.robot_cell.devices) == 3
    assert "kuka-no-estop" in runner.execution_context.robot_cell.devices
    assert "cycle" in runner.execution_context.robot_cell.devices
    assert "timer" in runner.execution_context.robot_cell.devices
    assert "ur10e-estop" not in runner.execution_context.robot_cell.devices
