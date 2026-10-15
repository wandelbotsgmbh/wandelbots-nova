import asyncio
from datetime import datetime, timezone

import pytest

from nova import Nova, api
from nova.cell.controllers import virtual_controller
from nova.command_routines import (
    command_routine,
    marker,
    move_joint_ptp,
    move_linear,
    set_io,
    wait_for_time,
)
from nova.datasets import Dataset, DatasetFrame, DatasetPose
from nova.exceptions import UnsupportedRoutineCommand
from nova.types import Pose


class TestMotionRoutine:
    """A motion-only routine runs to completion on a virtual controller."""

    @pytest.mark.asyncio
    @pytest.mark.integration
    async def test_motion_routine_runs_to_completion_on_virtual_controller(self):
        controller_name = "ur-routine-test"
        async with Nova() as nova:
            cell = nova.cell()
            await cell.ensure_controller(
                virtual_controller(
                    name=controller_name,
                    manufacturer=api.models.Manufacturer.UNIVERSALROBOTS,
                    type="universalrobots-ur10e",
                )
            )
            controller = await cell.controller(controller_name)
            async with controller[0] as mg:
                # GIVEN
                start = await mg.joints()
                start_pose = await mg.tcp_pose()
                target = start_pose.position.x + 50.0

                routine = command_routine(
                    "motion-only",
                    motion_group=mg.id,
                    tcp="Flange",
                    motion_group_setup=await mg.get_setup("Flange"),
                    start_joint_position=list(start),
                    commands=[
                        move_linear(
                            api.models.InlinePoseReference(
                                pose=api.models.Pose(
                                    position=[target, start_pose.position.y, start_pose.position.z],
                                    orientation=list(start_pose.orientation.to_tuple()),
                                )
                            )
                        ),
                        move_joint_ptp(list(start)),
                    ],
                )

                # WHEN
                await mg.plan_and_execute_routine(routine)

                # THEN
                end_joints = await mg.joints()
                assert end_joints == pytest.approx(start, abs=1e-2)


class TestDatasetPoses:
    """Dataset poses are resolved to world poses on a virtual controller."""

    @pytest.mark.asyncio
    @pytest.mark.integration
    async def test_dataset_pose_is_resolved_to_world_on_virtual_controller(self):
        """A pose taught 50 mm along x of a frame placed at the start TCP pose ends there in world."""
        controller_name = "ur-routine-dataset-test"
        async with Nova() as nova:
            cell = nova.cell()
            await cell.ensure_controller(
                virtual_controller(
                    name=controller_name,
                    manufacturer=api.models.Manufacturer.UNIVERSALROBOTS,
                    type="universalrobots-ur10e",
                )
            )
            controller = await cell.controller(controller_name)
            async with controller[0] as mg:
                # GIVEN
                start = await mg.joints()
                start_pose = await mg.tcp_pose("Flange")

                now = datetime(2026, 1, 1, tzinfo=timezone.utc)
                dataset = Dataset(
                    dataset="d",
                    revision=1,
                    created_at=now,
                    updated_at=now,
                    frames={"start": DatasetFrame(frame="start", dataset="d", pose=start_pose)},
                    poses={
                        "ahead": DatasetPose(
                            dataset_pose="ahead", dataset="d", frame="start", pose=Pose((50, 0, 0))
                        )
                    },
                )
                dataset.attach_frame_tree()
                routine = command_routine(
                    "dataset-pose",
                    motion_group=mg.id,
                    tcp="Flange",
                    motion_group_setup=await mg.get_setup("Flange"),
                    start_joint_position=list(start),
                    commands=[move_linear("ahead")],
                )

                # WHEN
                await mg.plan_and_execute_routine(routine, dataset)

                # THEN

                expected = start_pose @ Pose((50, 0, 0))
                reached = await mg.tcp_pose("Flange")
                assert tuple(reached.position.to_tuple()) == pytest.approx(
                    tuple(expected.position.to_tuple()), abs=1.0
                )
                await mg.plan_and_execute_routine(
                    command_routine(
                        "back",
                        motion_group=mg.id,
                        tcp="Flange",
                        motion_group_setup=await mg.get_setup("Flange"),
                        commands=[move_joint_ptp(list(start))],
                    )
                )


class TestSetIO:
    """set_io switches a controller output while the routine runs."""

    @pytest.mark.asyncio
    @pytest.mark.integration
    async def test_set_io_switches_output_during_run_on_virtual_controller(self):
        controller_name = "ur-routine-io-test"
        async with Nova() as nova:
            cell = nova.cell()
            await cell.ensure_controller(
                virtual_controller(
                    name=controller_name,
                    manufacturer=api.models.Manufacturer.UNIVERSALROBOTS,
                    type="universalrobots-ur10e",
                )
            )
            controller = await cell.controller(controller_name)
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
                    await mg.plan_and_execute_routine(routine)
                finally:
                    watcher.cancel()

                # THEN
                assert observed[0] is False
                assert await controller.read("tool_out[0]") is True
                await controller.write("tool_out[0]", False)


class TestExampleDatasetRoutine:
    """A Dataset routine with explicit motions and set_io, with and without trigger, completes."""

    @pytest.mark.asyncio
    @pytest.mark.integration
    async def test_dataset_routine_with_set_io_runs_on_virtual_controller(self):
        from pathlib import Path

        from nova import datasets as ds
        from nova.actions import joint_ptp
        from nova.command_routines import at_path_fraction, move_cartesian_ptp

        controller_name = "kuka-kr16-r2010"
        dataset = await ds.read("examples/example_dataset.json", base_dir=Path(__file__).parents[4])
        async with Nova() as nova:
            cell = nova.cell()
            await cell.ensure_controller(
                virtual_controller(
                    name=controller_name,
                    manufacturer=api.models.Manufacturer.KUKA,
                    type="kuka-kr16_r2010_2",
                )
            )
            controller = await cell.controller(controller_name)
            async with controller[0] as mg:
                # GIVEN
                tcp = (await mg.tcp_names())[0]
                home = tuple(await mg.joints())
                routine = command_routine(
                    "example",
                    tcp=tcp,
                    motion_group_setup=await mg.get_setup(tcp),
                    start_joint_position=list(home),
                    commands=[
                        move_cartesian_ptp("pick"),
                        set_io("OUT#1", True),
                        move_linear("place"),
                        set_io("OUT#1", False, at=at_path_fraction(0.5)),
                        move_cartesian_ptp("pick"),
                    ],
                )

                # WHEN
                await mg.plan_and_execute_routine(routine, dataset)

                # THEN
                assert await controller.read("OUT#1") is False
                await mg.plan_and_execute([joint_ptp(home)], tcp)


class TestUnsupportedCommands:
    """Unsupported commands are rejected with a message naming them, before the robot moves."""

    @pytest.mark.asyncio
    @pytest.mark.integration
    @pytest.mark.parametrize(
        "command, command_type", [(wait_for_time(500), "wait_for_time"), (marker("here"), "marker")]
    )
    async def test_unsupported_command_is_rejected_before_motion_on_virtual_controller(
        self, command, command_type
    ):
        controller_name = "ur-routine-unsupported-test"
        async with Nova() as nova:
            cell = nova.cell()
            await cell.ensure_controller(
                virtual_controller(
                    name=controller_name,
                    manufacturer=api.models.Manufacturer.UNIVERSALROBOTS,
                    type="universalrobots-ur10e",
                )
            )
            controller = await cell.controller(controller_name)
            async with controller[0] as mg:
                # GIVEN
                start = list(await mg.joints())
                moved = [start[0] + 0.3, *start[1:]]
                routine = command_routine(
                    "unsupported",
                    motion_group=mg.id,
                    tcp="Flange",
                    motion_group_setup=await mg.get_setup("Flange"),
                    start_joint_position=start,
                    commands=[move_joint_ptp(moved), command],
                )

                # WHEN
                with pytest.raises(UnsupportedRoutineCommand) as error:
                    await mg.plan_and_execute_routine(routine)

                # THEN
                assert command_type in str(error.value)
                assert "not supported yet" in str(error.value)
                assert await mg.joints() == pytest.approx(start, abs=1e-3)
