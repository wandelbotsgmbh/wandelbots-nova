import asyncio

import pytest

from nova import Nova, api
from nova import command_routines as cr
from nova import datasets as ds
from nova.actions import joint_ptp
from nova.cell.controllers import virtual_controller
from nova.command_routines import (
    UnsupportedRoutineCommand,
    at_path_fraction,
    command_routine,
    marker,
    motion,
    move_joint_ptp,
    move_linear,
    path_joint_ptp,
    set_io,
    wait_for_time,
)
from nova.types import Pose


def _inline_pose(pose: Pose, tcp: str = "Flange") -> api.models.InlinePose:
    return api.models.InlinePose(
        pose=pose.to_api_model(),
        frame=api.models.WorldFrameReference(),
        tcp=tcp,
        kinematic_configuration=api.models.UnspecifiedKinematicConfiguration(),
    )


async def _ur10e(nova: Nova, name: str):
    cell = nova.cell()
    await cell.ensure_controller(
        virtual_controller(
            name=name,
            manufacturer=api.models.Manufacturer.UNIVERSALROBOTS,
            type="universalrobots-ur10e",
        )
    )
    return await cell.controller(name)


class TestMotionRoutine:
    """A motion-only routine runs to completion on a virtual controller."""

    @pytest.mark.asyncio
    @pytest.mark.integration
    async def test_motion_routine_runs_to_completion_on_virtual_controller(self):
        async with Nova() as nova:
            controller = await _ur10e(nova, "ur-routine-test")
            async with controller[0] as mg:
                # GIVEN
                start = await mg.joints()
                start_pose = await mg.tcp_pose("Flange")
                routine = command_routine(
                    "motion-only",
                    motion_group=mg.id,
                    tcp="Flange",
                    motion_group_setup=await mg.get_setup("Flange"),
                    start_joint_position=list(start),
                    commands=[
                        move_linear(_inline_pose(start_pose @ Pose((50, 0, 0)))),
                        move_joint_ptp(list(start)),
                    ],
                )

                # WHEN
                await cr.execute(nova, routine)

                # THEN
                end_joints = await mg.joints()
                assert end_joints == pytest.approx(start, abs=1e-2)


class TestDatasetPoses:
    """A routine stored in a Dataset is resolved by NOVA and runs on a virtual controller."""

    @pytest.mark.asyncio
    @pytest.mark.integration
    async def test_dataset_routine_is_resolved_and_runs_on_virtual_controller(self):
        """A pose taught 50 mm along x of a frame placed at the start TCP pose ends there in world."""
        dataset = "sdk-routine-test"
        async with Nova() as nova:
            controller = await _ur10e(nova, "ur-routine-dataset-test")
            async with controller[0] as mg:
                # GIVEN
                start = await mg.joints()
                start_pose = await mg.tcp_pose("Flange")
                routine = command_routine(
                    "dataset-pose",
                    motion_group=mg.id,
                    tcp="Flange",
                    motion_group_setup=await mg.get_setup("Flange"),
                    start_joint_position=list(start),
                    commands=[move_linear("ahead")],
                )
                cell = nova.cell().cell_id
                await nova.api.datasets_api.create_dataset(
                    cell=cell,
                    create_dataset_request=api.models.CreateDatasetRequest(
                        dataset=dataset,
                        frames=[
                            api.models.DatasetFrame(
                                frame="start", dataset=dataset, pose=start_pose.to_api_model()
                            )
                        ],
                        poses=[
                            api.models.DatasetPose(
                                dataset_pose="ahead",
                                dataset=dataset,
                                pose=Pose((50, 0, 0)).to_api_model(),
                                frame=api.models.DatasetFrameReference(frame="start"),
                                tcp="Flange",
                                kinematic_configuration=api.models.UnspecifiedKinematicConfiguration(),
                            )
                        ],
                        command_routines=[routine],
                    ),
                )
                try:
                    # WHEN
                    resolved = await ds.resolve_command_routine(
                        nova, dataset, "dataset-pose", cell=cell
                    )
                    await cr.execute(nova, resolved, cell=cell)
                finally:
                    await nova.api.datasets_api.delete_dataset(cell=cell, dataset=dataset)

                # THEN
                expected = start_pose @ Pose((50, 0, 0))
                reached = await mg.tcp_pose("Flange")
                assert tuple(reached.position.to_tuple()) == pytest.approx(
                    tuple(expected.position.to_tuple()), abs=1.0
                )
                await mg.plan_and_execute([joint_ptp(tuple(start))], "Flange")


class TestSetIO:
    """set_io switches a controller output while the routine runs."""

    @pytest.mark.asyncio
    @pytest.mark.integration
    async def test_set_io_switches_output_during_run_on_virtual_controller(self):
        async with Nova() as nova:
            controller = await _ur10e(nova, "ur-routine-io-test")
            async with controller[0] as mg:
                # GIVEN
                await controller.write("tool_out[0]", False)
                start = list(await mg.joints())
                moved = [start[0] + 0.3, *start[1:]]
                routine = command_routine(
                    "io",
                    motion_group=mg.id,
                    tcp="Flange",
                    motion_group_setup=await mg.get_setup("Flange"),
                    start_joint_position=start,
                    commands=[
                        move_joint_ptp(moved),
                        set_io("tool_out[0]", True),
                        move_joint_ptp(start),
                    ],
                )

                # WHEN
                observed: list[bool] = []

                async def watch():
                    while True:
                        observed.append(bool(await controller.read("tool_out[0]")))
                        await asyncio.sleep(0.05)

                watcher = asyncio.create_task(watch())
                try:
                    await cr.execute(nova, routine)
                finally:
                    watcher.cancel()

                # THEN
                assert observed[0] is False
                assert await controller.read("tool_out[0]") is True
                await controller.write("tool_out[0]", False)


class TestAllCommands:
    """Waits, markers and triggered set_io run on NOVA alongside the motions."""

    @pytest.mark.asyncio
    @pytest.mark.integration
    async def test_routine_with_wait_marker_and_triggered_set_io_completes(self):
        async with Nova() as nova:
            controller = await _ur10e(nova, "ur-routine-all-test")
            async with controller[0] as mg:
                # GIVEN
                await controller.write("tool_out[0]", True)
                start = list(await mg.joints())
                moved = [start[0] + 0.3, *start[1:]]
                routine = command_routine(
                    "all-commands",
                    motion_group=mg.id,
                    tcp="Flange",
                    motion_group_setup=await mg.get_setup("Flange"),
                    start_joint_position=start,
                    commands=[
                        move_joint_ptp(moved),
                        wait_for_time(200),
                        marker("half-way"),
                        set_io("tool_out[0]", False, at=at_path_fraction(0.5)),
                        move_joint_ptp(start),
                    ],
                )

                # WHEN
                await cr.execute(nova, routine)

                # THEN
                assert await controller.read("tool_out[0]") is False
                assert await mg.joints() == pytest.approx(start, abs=1e-2)


class TestUnsupportedCommands:
    """What NOVA cannot execute yet is rejected naming the command, before the robot moves."""

    @pytest.mark.asyncio
    @pytest.mark.integration
    async def test_unsupported_command_is_rejected_before_motion_on_virtual_controller(self):
        async with Nova() as nova:
            controller = await _ur10e(nova, "ur-routine-unsupported-test")
            async with controller[0] as mg:
                # GIVEN
                start = list(await mg.joints())
                moved = [start[0] + 0.3, *start[1:]]
                pose = _inline_pose(await mg.tcp_pose("Flange"))
                routine = command_routine(
                    "unsupported",
                    motion_group=mg.id,
                    tcp="Flange",
                    motion_group_setup=await mg.get_setup("Flange"),
                    start_joint_position=start,
                    commands=[move_joint_ptp(moved), motion(pose, path_joint_ptp())],
                )

                # WHEN
                with pytest.raises(UnsupportedRoutineCommand) as error:
                    await cr.execute(nova, routine)

                # THEN
                assert "Command 1 (motion_command)" in str(error.value)
                assert await mg.joints() == pytest.approx(start, abs=1e-3)
