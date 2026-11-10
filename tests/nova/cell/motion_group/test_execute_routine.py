"""Running CommandRoutines on a ``MotionGroup`` with ``CommandRoutine`` on a virtual KUKA controller.

Every test defines the routine it runs, so what it does is visible where it is asserted.
``CommandRoutine.execute`` returns the started run as soon as NOVA has accepted it, so the tests follow
the run by polling the API themselves; the library does not wait (that will use NATS).
"""

import asyncio
import math
from contextlib import asynccontextmanager
from dataclasses import dataclass

import pytest

from nova import Nova, api
from nova.actions import joint_ptp
from nova.cell.controllers import virtual_controller
from nova.command_routines import (
    CommandRoutine,
    CommandRoutineError,
    UnsupportedRoutineCommand,
    at_path_fraction,
    command_routine,
    io_value,
    marker,
    move_cartesian_ptp,
    move_circular,
    move_joint_ptp,
    move_linear,
    set_io,
    wait_for_io,
    wait_for_time,
)
from nova.types import Pose

pytestmark = [pytest.mark.integration, pytest.mark.asyncio]

_State = api.models.CommandRoutineRunState
_FINAL = (_State.COMPLETED, _State.FAILED, _State.STOPPED)

OUT_1, OUT_2, IN_1 = "OUT#1", "OUT#2", "IN#1"
HOME = (0.0, -math.pi / 2, math.pi / 2, 0.0, math.pi / 2, math.pi)
"""Where every test starts: the position a virtual KUKA has when it is created."""


@dataclass(frozen=True)
class Context:
    """The robot a routine is written for, as it stands now."""

    start: list[float]
    """Joint position the robot is at, in radians."""
    moved: list[float]
    """Joint position a little away from ``start``."""
    tcp_pose: Pose
    """Pose of the TCP at ``start``; motion targets are offsets from it."""
    setup: api.models.MotionGroupSetup
    tcp: str
    motion_group: str


def _kuka(name: str):
    return virtual_controller(
        name=name, manufacturer=api.models.Manufacturer.KUKA, type="kuka-kr16_r2010_2"
    )


async def _context(mg) -> Context:
    tcp = (await mg.tcp_names())[0]
    start = list(await mg.joints())
    return Context(
        start=start,
        moved=[start[0] + 0.2, *start[1:]],
        tcp_pose=await mg.tcp_pose(tcp),
        setup=await mg.get_setup(tcp),
        tcp=tcp,
        motion_group=mg.id,
    )


def _routine(ctx: Context, name: str, *commands, **settings) -> CommandRoutine:
    """A routine written for the robot of ``ctx``, with its start position, TCP and setup."""
    return command_routine(
        name,
        motion_group=ctx.motion_group,
        tcp=ctx.tcp,
        motion_group_setup=ctx.setup,
        start_joint_position=ctx.start,
        commands=list(commands),
        **settings,
    )


def _pose(ctx: Context, dx: float = 0.0, dy: float = 0.0, *, tcp: str | None = None):
    """An inline world pose ``dx``/``dy`` millimetres from the start pose."""
    pose = ctx.tcp_pose @ Pose((dx, dy, 0, 0, 0, 0))
    return api.models.InlinePose(
        pose=pose.to_api_model(),
        frame=api.models.WorldFrameReference(),
        tcp=tcp or ctx.tcp,
        kinematic_configuration=api.models.UnspecifiedKinematicConfiguration(),
    )


def _input_is_true(io: str) -> api.models.IOConditionExpression:
    return api.models.IOConditionExpression(
        io=io_value(io, True),
        comparator=api.models.Comparator.COMPARATOR_EQUALS,
        io_origin=api.models.IOOrigin.CONTROLLER,
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


@asynccontextmanager
async def _watch_runs(nova: Nova):
    """Collect every change of every run NOVA publishes on NATS, in order.

    Polling the run misses commands the robot only passes (a marker lasts an instant), NATS
    carries one message per change.
    """
    updates: list[api.models.CommandRoutineRun] = []

    async def collect(message):
        updates.append(api.models.CommandRoutineRun.model_validate_json(message.data))

    subscription = await nova.nats.subscribe(
        f"nova.v2.cells.{nova.cell().cell_id}.command-routines.runs.*", cb=collect
    )
    try:
        yield updates
    finally:
        await subscription.unsubscribe()


def _markers_passed(
    updates: list[api.models.CommandRoutineRun],
    run: api.models.CommandRoutineRun,
    routine: CommandRoutine,
) -> list[str]:
    """Names of the markers the run was on, in order. A marker's payload is not published."""
    indices = [u.current_command_index for u in updates if u.run == run.run]
    return [
        routine.commands[index].name
        for index in dict.fromkeys(i for i in indices if i is not None)  # each command once
        if isinstance(routine.commands[index], api.models.MarkerCommand)
    ]


async def _on_command(nova: Nova, run: api.models.CommandRoutineRun, index: int):
    """Follow the run until it is executing or waiting on the command ``index``."""
    routines = nova.api.command_routines_api
    async with asyncio.timeout(30):
        while run.current_command_index != index:
            await asyncio.sleep(0.05)
            run = await routines.get_command_routine_run(cell=nova.cell().cell_id, run=run.run)
    return run


async def _go_home(mg):
    """Bring the robot to ``HOME``; an earlier test may have left it elsewhere."""
    tcp = (await mg.tcp_names())[0]
    await mg.plan_and_execute([joint_ptp(HOME)], tcp)


async def _stop_active_runs(nova: Nova):
    """A run that is left active keeps its motion group busy for every later test."""
    routines = nova.api.command_routines_api
    cell = nova.cell().cell_id
    for run in (await routines.list_command_routine_runs(cell=cell)).runs:
        if run.state not in _FINAL:
            await routines.stop_command_routine_run(cell=cell, run=run.run)


@pytest.fixture
async def robot():
    async with Nova() as nova:
        controller = await nova.cell().ensure_controller(_kuka("kuka-routine-test"))
        async with controller[0] as mg:
            await _stop_active_runs(nova)
            await _go_home(mg)
            try:
                yield nova, controller, mg
            finally:
                await _stop_active_runs(nova)
                await _go_home(mg)


class TestRun:
    async def test_every_motion_type_runs(self, robot):
        # GIVEN
        nova, _, mg = robot
        ctx = await _context(mg)
        routine = _routine(
            ctx,
            "motion-types",
            move_joint_ptp(ctx.moved),
            move_cartesian_ptp(_pose(ctx, 20)),
            move_linear(_pose(ctx, 40)),
            move_circular(_pose(ctx, 60), via_pose=_pose(ctx, 50, 20)),
        )

        # WHEN
        run = await routine.execute(mg)
        final = await _final(nova, run)

        # THEN
        assert final.state == _State.COMPLETED, final.error

    async def test_set_io_is_written_where_the_robot_stands_and_on_the_way(self, robot):
        # GIVEN
        nova, controller, mg = robot
        ctx = await _context(mg)
        await controller.write(OUT_1, False)
        await controller.write(OUT_2, False)
        routine = _routine(
            ctx,
            "set-io",
            set_io(OUT_1, True),  # where the robot stands: written by the service
            move_joint_ptp(ctx.moved),
            set_io(OUT_2, True),  # between two motions: written by the controller
            move_joint_ptp(ctx.start),
        )

        # WHEN
        run = await routine.execute(mg)
        final = await _final(nova, run)

        # THEN
        assert final.state == _State.COMPLETED, final.error
        assert await controller.read(OUT_1) is True
        assert await controller.read(OUT_2) is True
        await controller.write(OUT_1, False)
        await controller.write(OUT_2, False)

    async def test_wait_for_time_holds_the_run_on_the_wait(self, robot):
        # GIVEN
        nova, _, mg = robot
        ctx = await _context(mg)
        routine = _routine(
            ctx,
            "wait-for-time",
            move_joint_ptp(ctx.moved),
            wait_for_time(1500),  # command 1
            move_joint_ptp(ctx.start),
        )
        seen: list = []

        # WHEN
        run = await routine.execute(mg)
        final = await _final(nova, run, seen)

        # THEN
        assert final.state == _State.COMPLETED, final.error
        assert 1 in seen

    async def test_wait_for_io_continues_when_the_input_becomes_true(self, robot):
        # GIVEN
        nova, controller, mg = robot
        ctx = await _context(mg)
        await controller.write(IN_1, False)
        routine = _routine(
            ctx,
            "wait-for-io",
            move_joint_ptp(ctx.moved),
            wait_for_io(_input_is_true(IN_1), timeout_ms=30_000),
            move_joint_ptp(ctx.start),
        )

        # WHEN
        run = await routine.execute(mg)
        await asyncio.sleep(1.5)  # the routine waits for this input
        await controller.write(IN_1, True)
        final = await _final(nova, run)

        # THEN
        assert final.state == _State.COMPLETED, final.error

    async def test_markers_are_passed_in_order(self, robot):
        # GIVEN
        nova, _, mg = robot
        ctx = await _context(mg)
        routine = _routine(
            ctx,
            "markers",
            marker("start", payload={"stage": "idle"}),  # where the robot stands
            move_joint_ptp(ctx.moved),
            marker("passed"),  # passed while the robot moves on
            move_joint_ptp(ctx.start),
            marker("end", payload={"count": 3}),
        )

        # WHEN
        async with _watch_runs(nova) as updates:
            run = await routine.execute(mg)
            final = await _final(nova, run)
            await asyncio.sleep(0.5)  # the last messages are still on their way

        # THEN
        assert final.state == _State.COMPLETED, final.error
        assert _markers_passed(updates, run, routine) == ["start", "passed", "end"]

    async def test_default_and_per_motion_speed_settings_run(self, robot):
        # GIVEN
        nova, _, mg = robot
        ctx = await _context(mg)
        default = api.models.MotionSettings(
            blending=api.models.BlendingPosition(position_zone_radius=5.0),
            limits_override=api.models.LimitsOverride(tcp_velocity_limit=100.0),
        )
        slower = api.models.MotionSettings(
            limits_override=api.models.LimitsOverride(tcp_velocity_limit=25.0)
        )
        routine = _routine(
            ctx,
            "motion-settings",
            move_cartesian_ptp(_pose(ctx, 20)),  # the routine's default settings
            move_cartesian_ptp(_pose(ctx, 40), settings=slower),  # overridden speed
            default_motion_settings=default,
        )

        # WHEN
        run = await routine.execute(mg)
        final = await _final(nova, run)

        # THEN
        assert final.state == _State.COMPLETED, final.error

    async def test_a_run_can_be_stopped(self, robot):
        # GIVEN
        nova, _, mg = robot
        ctx = await _context(mg)
        routine = _routine(
            ctx, "stop", move_joint_ptp(ctx.moved), wait_for_time(1500), move_joint_ptp(ctx.start)
        )
        run = await routine.execute(mg)

        # WHEN
        run = await _on_command(nova, run, 1)  # the robot has moved and now waits
        stopped = await CommandRoutine.stop(mg, run)

        # THEN
        assert stopped.state == _State.STOPPED
        assert (await _final(nova, run)).state == _State.STOPPED


class TestPlan:
    async def test_returns_the_trajectory_without_moving_the_robot(self, robot):
        # GIVEN
        _, _, mg = robot
        ctx = await _context(mg)
        routine = _routine(ctx, "plan", move_joint_ptp(ctx.moved), move_joint_ptp(ctx.start))

        # WHEN
        plan = await routine.plan(mg)

        # THEN
        assert len(plan.joint_trajectory.joint_positions) > 1
        assert await mg.joints() == pytest.approx(ctx.start, abs=1e-3)


class TestAtTrigger:
    async def test_set_io_at_a_fraction_of_the_motion_is_written_during_that_motion(self, robot):
        # GIVEN
        nova, controller, mg = robot
        ctx = await _context(mg)
        await controller.write(OUT_1, False)
        routine = _routine(
            ctx,
            "set-io-at",
            move_joint_ptp(ctx.moved),  # command 0: from location 0 to location 1
            set_io(OUT_1, True, at=at_path_fraction(0.5)),  # command 1: halfway along command 0
            move_joint_ptp(ctx.start),  # command 2: from location 1 to location 2
        )

        # WHEN
        plan = await routine.plan(mg)
        run = await routine.execute(mg, plan)  # what was planned is what runs
        final = await _final(nova, run)

        # THEN
        [placed] = [c for c in plan.commands if c.command_index == 1]
        assert placed.location == pytest.approx(0.5, abs=0.05)  # inside the first motion
        assert final.state == _State.COMPLETED, final.error
        assert await controller.read(OUT_1) is True
        await controller.write(OUT_1, False)


class TestDatasetPoses:
    async def test_a_routine_of_a_stored_dataset_is_resolved_by_nova_and_runs(self, robot):
        """A pose taught 50 mm along x of a frame at the start TCP pose ends there in world."""
        # GIVEN
        nova, _, mg = robot
        ctx = await _context(mg)
        cell, dataset = nova.cell().cell_id, "sdk-routine-test"
        stored = _routine(ctx, "dataset-pose", move_linear("ahead"))
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

        # WHEN
        try:
            routine = await CommandRoutine.resolve(nova, dataset, "dataset-pose", cell=cell)
            run = await routine.execute(mg)
            final = await _final(nova, run)
        finally:
            await nova.api.datasets_api.delete_dataset(cell=cell, dataset=dataset)

        # THEN
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
                # GIVEN
                await _stop_active_runs(nova)
                await _go_home(mg_named)
                await _go_home(mg_used)
                ctx = await _context(mg_named)  # the routine names the first motion group
                routine = _routine(ctx, "other-motion-group", move_joint_ptp(ctx.moved))
                assert await mg_used.joints() == pytest.approx(ctx.start, abs=1e-2)

                # WHEN
                run = await routine.execute(mg_used)
                final = await _final(nova, run)

                # THEN
                assert final.state == _State.COMPLETED, final.error
                assert await mg_used.joints() == pytest.approx(ctx.moved, abs=1e-2)
                assert await mg_named.joints() == pytest.approx(ctx.start, abs=1e-2)
                await mg_used.plan_and_execute([joint_ptp(tuple(ctx.start))], ctx.tcp)


class TestErrors:
    async def test_an_unresolved_pose_is_a_command_routine_error(self, robot):
        # GIVEN
        _, _, mg = robot
        ctx = await _context(mg)
        routine = _routine(ctx, "unresolved", move_linear("slot"))  # a local pose id

        # WHEN
        # THEN
        with pytest.raises(CommandRoutineError) as raised:
            await routine.execute(mg)
        assert type(raised.value) is CommandRoutineError
        assert await mg.joints() == pytest.approx(ctx.start, abs=1e-3)  # nothing moved

    async def test_a_pose_of_another_tcp_is_unsupported(self, robot):
        # GIVEN
        _, _, mg = robot
        ctx = await _context(mg)
        routine = _routine(ctx, "other-tcp", move_linear(_pose(ctx, 20, tcp="OtherTcp")))

        # WHEN
        # THEN
        with pytest.raises(UnsupportedRoutineCommand):
            await routine.execute(mg)
        assert await mg.joints() == pytest.approx(ctx.start, abs=1e-3)  # nothing moved
