from datetime import datetime, timezone

import pytest

from nova import Nova, api
from nova.cell.controllers import virtual_controller
from nova.command_routines import command_routine, move_joint_ptp, move_linear
from nova.datasets import Dataset, DatasetFrame, DatasetPose
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
