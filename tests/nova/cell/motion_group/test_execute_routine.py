"""Running CommandRoutines through ``MotionGroup`` on a virtual KUKA controller.

NOVA's own scenarios (``tests/command_routines/scenarios.py``) run against a real instance:
every feature must reach ``COMPLETED``, every error must be answered as documented.
``plan_and_execute`` returns the started run as soon as NOVA has accepted it, so the tests follow
the run by polling the API themselves; the library does not wait (that will use NATS).
"""

import asyncio

import pytest

from nova import Nova, api
from nova import datasets as ds
from nova.actions import joint_ptp
from nova.cell.controllers import virtual_controller
from nova.command_routines import (
    CommandRoutineError,
    UnsupportedRoutineCommand,
    command_routine,
    move_joint_ptp,
    move_linear,
)
from nova.command_routines.exceptions import error_from_api_exception
from nova.types import Pose
from tests.command_routines.scenarios import Context, errors, features

pytestmark = [pytest.mark.integration, pytest.mark.asyncio]

_State = api.models.CommandRoutineRunState
_FINAL = (_State.COMPLETED, _State.FAILED, _State.STOPPED)
_DUMMY = Context(start=[0.0] * 6, tcp_pose=Pose((0, 0, 0, 0, 0, 0)))

_NEEDS_BUS_IO = "needs the bus IO service (virtual profinet) with the IOs of the routine"
_INPUT_TO_SET = {"04-wait-for-io": "IN#1"}
_SKIPPED = {"05-wait-for-io-expression": _NEEDS_BUS_IO, "11-io-only-bus": _NEEDS_BUS_IO}
# These routines are I/O-only: `motion_group_setup: null` is what declares them, and the
# generated client drops explicit nulls from the request, so NOVA cannot tell.
_IO_ONLY_ERRORS = {
    "e04-io-only-with-motion-group",
    "e05-io-only-controller-io",
    "e06-io-only-with-motion",
    "e14-io-only-at-trigger",
    "e15-io-only-pause-on-io",
}


def _kuka(name: str):
    return virtual_controller(
        name=name, manufacturer=api.models.Manufacturer.KUKA, type="kuka-kr16_r2010_2"
    )


async def _context(mg) -> Context:
    """The routine context of this robot, as it stands now."""
    tcp = (await mg.tcp_names())[0]
    return Context(
        start=list(await mg.joints()),
        tcp_pose=await mg.tcp_pose(tcp),
        setup=await mg.get_setup(tcp),
        tcp=tcp,
        motion_group=mg.id,
    )


async def _final(nova: Nova, run: api.models.CommandRoutineRun, seen: list | None = None):
    """Follow the run until it ends; ``seen`` collects the commands it was on."""
    routines = nova.api.command_routines_api
    async with asyncio.timeout(90):
        while run.state not in _FINAL:
            if seen is not None:
                seen.append(run.current_command_index)
            await asyncio.sleep(0.1)
            run = await routines.get_command_routine_run(cell=nova.cell().cell_id, run=run.run)
    return run


@pytest.fixture
async def robot():
    async with Nova() as nova:
        controller = await nova.cell().ensure_controller(_kuka("kuka-routine-test"))
        async with controller[0] as mg:
            yield nova, controller, mg


class TestFeatures:
    @pytest.mark.parametrize(
        "name",
        [
            pytest.param(name, marks=pytest.mark.skip(reason=_SKIPPED[name]))
            if name in _SKIPPED
            else name
            for name in features(_DUMMY)
        ],
    )
    async def test_feature_routine_completes(self, robot, name):
        nova, controller, mg = robot
        routine = features(await _context(mg))[name]

        run = await mg.plan_and_execute(routine)
        if name in _INPUT_TO_SET:
            await asyncio.sleep(1.5)  # the routine waits for this input
            await controller.write(_INPUT_TO_SET[name], True)
        final = await _final(nova, run)

        assert final.state == _State.COMPLETED, final.error

    async def test_set_io_is_written_where_the_robot_stands_and_on_the_way(self, robot):
        nova, controller, mg = robot
        ctx = await _context(mg)
        await controller.write(ctx.io_out, False)
        await controller.write(ctx.io_out_2, False)

        run = await mg.plan_and_execute(features(ctx)["02-set-io"])
        final = await _final(nova, run)

        assert final.state == _State.COMPLETED, final.error
        assert await controller.read(ctx.io_out) is True
        assert await controller.read(ctx.io_out_2) is True
        await controller.write(ctx.io_out, False)
        await controller.write(ctx.io_out_2, False)

    async def test_wait_for_time_holds_the_run_on_the_wait(self, robot):
        nova, _, mg = robot
        routine = features(await _context(mg))["03-wait-for-time"]
        seen: list = []

        run = await mg.plan_and_execute(routine)
        final = await _final(nova, run, seen)

        assert final.state == _State.COMPLETED, final.error
        assert 1 in seen  # command 1 is the 1.5 s wait

    async def test_plan_returns_the_trajectory_without_moving_the_robot(self, robot):
        _, _, mg = robot
        ctx = await _context(mg)

        trajectory = await mg.plan(features(ctx)["01-motion-types"])

        assert len(trajectory.joint_positions) > 1
        assert await mg.joints() == pytest.approx(ctx.start, abs=1e-3)

    async def test_a_run_can_be_stopped(self, robot):
        nova, _, mg = robot
        routine = features(await _context(mg))["03-wait-for-time"]

        run = await mg.plan_and_execute(routine)
        stopped = await mg.stop_routine(run)

        assert stopped.state == _State.STOPPED
        assert (await _final(nova, run)).state == _State.STOPPED


class TestDatasetPoses:
    async def test_a_routine_of_a_stored_dataset_is_resolved_by_nova_and_runs(self, robot):
        """A pose taught 50 mm along x of a frame at the start TCP pose ends there in world."""
        nova, _, mg = robot
        ctx = await _context(mg)
        cell, dataset = nova.cell().cell_id, "sdk-routine-test"
        stored = command_routine(
            "dataset-pose",
            motion_group=mg.id,
            tcp=ctx.tcp,
            motion_group_setup=ctx.setup,
            start_joint_position=ctx.start,
            commands=[move_linear("ahead")],
        )
        await nova.api.datasets_api.create_dataset(
            cell=cell,
            create_dataset_request=api.models.CreateDatasetRequest(
                dataset=dataset,
                frames=[
                    api.models.DatasetFrame(
                        frame="start", dataset=dataset, pose=ctx.tcp_pose.to_api_model()
                    )
                ],
                poses=[
                    api.models.DatasetPose(
                        dataset_pose="ahead",
                        dataset=dataset,
                        pose=Pose((50, 0, 0)).to_api_model(),
                        frame=api.models.DatasetFrameReference(frame="start"),
                        tcp=ctx.tcp,
                        kinematic_configuration=api.models.UnspecifiedKinematicConfiguration(),
                    )
                ],
                command_routines=[stored],
            ),
        )
        try:
            routine = await ds.resolve_command_routine(nova, dataset, "dataset-pose", cell=cell)
            run = await mg.plan_and_execute(routine)
            final = await _final(nova, run)
        finally:
            await nova.api.datasets_api.delete_dataset(cell=cell, dataset=dataset)

        assert final.state == _State.COMPLETED, final.error
        expected = ctx.tcp_pose @ Pose((50, 0, 0))
        reached = await mg.tcp_pose(ctx.tcp)
        assert tuple(reached.position.to_tuple()) == pytest.approx(
            tuple(expected.position.to_tuple()), abs=1.0
        )
        await mg.plan_and_execute([joint_ptp(tuple(ctx.start))], ctx.tcp)


class TestOtherMotionGroup:
    async def test_a_routine_runs_on_a_motion_group_other_than_the_routines(self):
        async with Nova() as nova:
            cell = nova.cell()
            first = await cell.ensure_controller(_kuka("kuka-routine-test"))
            second = await cell.ensure_controller(_kuka("kuka-routine-other"))
            async with first[0] as mg_named, second[0] as mg_used:
                ctx = await _context(mg_named)  # the routine names the first motion group
                moved = [ctx.start[0] + 0.2, *ctx.start[1:]]
                routine = command_routine(
                    "other-motion-group",
                    motion_group=mg_named.id,
                    tcp=ctx.tcp,
                    motion_group_setup=ctx.setup,
                    start_joint_position=ctx.start,
                    commands=[move_joint_ptp(moved)],
                )
                assert await mg_used.joints() == pytest.approx(ctx.start, abs=1e-2)

                run = await mg_used.plan_and_execute(routine)
                final = await _final(nova, run)

                assert final.state == _State.COMPLETED, final.error
                assert await mg_used.joints() == pytest.approx(moved, abs=1e-2)
                assert await mg_named.joints() == pytest.approx(ctx.start, abs=1e-2)
                await mg_used.plan_and_execute([joint_ptp(tuple(ctx.start))], ctx.tcp)


class TestErrors:
    @pytest.mark.parametrize(
        "name",
        [
            pytest.param(
                name,
                marks=pytest.mark.xfail(
                    reason="the client drops `motion_group_setup: null`, so NOVA does not see an "
                    "I/O-only routine; it answers for another reason",
                    strict=False,
                ),
            )
            if name in _IO_ONLY_ERRORS
            else name
            for name in errors(_DUMMY)
        ],
    )
    async def test_error_is_answered_as_documented(self, robot, name):
        nova, controller, mg = robot
        ctx = await _context(mg)
        failure = errors(ctx)[name]
        await controller.write(ctx.io_in_never, False)

        if failure.status is None:
            run = await mg.plan_and_execute(failure.routine)  # NOVA accepts the start ...
            final = await _final(nova, run)  # ... and the run fails
            assert final.state == _State.FAILED
            assert "Command 1" in (final.error or "")
            return

        # Sent as authored, without the motion group of the call as the one to run on: the
        # I/O-only errors are about what the routine itself names (or leaves out).
        request = api.models.StartCommandRoutineRunRequest(command_routine=failure.routine)
        with pytest.raises(api.ApiException) as raised:
            await nova.api.command_routines_api.start_command_routine_run(
                cell=nova.cell().cell_id, start_command_routine_run_request=request
            )

        expected = UnsupportedRoutineCommand if failure.status == 501 else CommandRoutineError
        assert raised.value.status == failure.status
        assert type(error_from_api_exception(raised.value)) is expected
        assert await mg.joints() == pytest.approx(ctx.start, abs=1e-3)  # nothing moved
