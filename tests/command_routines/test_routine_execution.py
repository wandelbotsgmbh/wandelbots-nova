from datetime import datetime, timezone
from unittest.mock import AsyncMock, MagicMock, Mock

import pytest

from nova import api
from nova.cell.motion_group import MotionGroup
from nova.command_routines import (
    CommandRoutine,
    CommandRoutineError,
    UnsupportedRoutineCommand,
    command_routine,
    move_joint_ptp,
)
from nova.core.nova import Nova
from nova.datasets import DatasetError, DatasetNotFoundError

pytestmark = pytest.mark.asyncio


def _run(state=api.models.CommandRoutineRunState.RUNNING) -> api.models.CommandRoutineRun:
    return api.models.CommandRoutineRun(
        run="run-1",
        command_routine="routine",
        state=state,
        created_at=datetime(2026, 1, 1, tzinfo=timezone.utc),
    )


@pytest.fixture
def routines_api():
    routines_api = MagicMock()
    routines_api.plan_command_routine = AsyncMock(
        return_value=api.models.PlanCommandRoutineResponse(
            joint_trajectory=api.models.JointTrajectory(
                joint_positions=[[0.0] * 6, [0.1] * 6], times=[0.0, 1.0], locations=[0.0, 1.0]
            )
        )
    )
    routines_api.start_command_routine_run = AsyncMock(return_value=_run())
    routines_api.get_command_routine_run = AsyncMock()
    routines_api.stop_command_routine_run = AsyncMock(
        return_value=_run(api.models.CommandRoutineRunState.STOPPED)
    )
    return routines_api


@pytest.fixture
def motion_group(routines_api):
    api_client = MagicMock()
    api_client.command_routines_api = routines_api
    return MotionGroup(
        api_client=api_client,
        cell="test_cell",
        controller_id="test-controller",
        motion_group_id="0@test-controller",
    )


def _routine(**kwargs) -> CommandRoutine:
    return command_routine("routine", commands=[move_joint_ptp([0.0] * 6)], **kwargs)


def _start_request(routines_api) -> api.models.StartCommandRoutineRunRequest:
    return routines_api.start_command_routine_run.await_args.kwargs[
        "start_command_routine_run_request"
    ]


class TestPlan:
    async def test_routine_of_another_motion_group_is_planned_on_this_one(
        self, motion_group, routines_api
    ):
        # GIVEN a routine written for another motion group
        routine = _routine(motion_group="0@other")

        # WHEN
        trajectory = await routine.plan(motion_group)

        # THEN NOVA plans it for this controller and motion group, and the routine stays as authored
        request = routines_api.plan_command_routine.await_args.kwargs[
            "plan_command_routine_request"
        ]
        assert routines_api.plan_command_routine.await_args.kwargs["cell"] == "test_cell"
        assert request.controller == "test-controller"
        assert request.motion_group_override == api.models.MotionGroupReference(
            id="0@test-controller"
        )
        assert request.command_routine == routine
        assert trajectory.times == [0.0, 1.0]
        routines_api.start_command_routine_run.assert_not_awaited()

    async def test_io_only_routine_has_nothing_to_plan(self, motion_group, routines_api):
        # GIVEN
        routines_api.plan_command_routine.return_value = api.models.PlanCommandRoutineResponse()

        # WHEN
        # THEN
        with pytest.raises(ValueError, match="I/O-only"):
            await _routine().plan(motion_group)


class TestExecute:
    async def test_routine_of_this_motion_group_is_not_overridden(self, motion_group, routines_api):
        # GIVEN a routine that names this motion group
        routine = _routine(motion_group="0@test-controller")

        # WHEN
        await routine.execute(motion_group)

        # THEN NOVA runs it with its own setup: only the controller is passed
        request = _start_request(routines_api)
        assert request.controller == "test-controller"
        assert request.motion_group_override is None

    async def test_routine_of_another_motion_group_runs_on_this_one(
        self, motion_group, routines_api
    ):
        # GIVEN
        routine = _routine(motion_group="0@other")

        # WHEN
        await routine.execute(motion_group)

        # THEN
        request = _start_request(routines_api)
        assert request.motion_group_override == api.models.MotionGroupReference(
            id="0@test-controller"
        )
        assert request.command_routine == routine

    async def test_returns_the_started_run_without_waiting(self, motion_group, routines_api):
        # GIVEN
        # WHEN
        run = await _routine().execute(motion_group)

        # THEN the run NOVA accepted is returned, nothing is planned, polled or stopped
        assert run.run == "run-1"
        assert run.state == api.models.CommandRoutineRunState.RUNNING
        routines_api.plan_command_routine.assert_not_awaited()
        routines_api.get_command_routine_run.assert_not_awaited()
        routines_api.stop_command_routine_run.assert_not_awaited()

    @pytest.mark.parametrize(
        "status, error", [(501, UnsupportedRoutineCommand), (422, CommandRoutineError)]
    )
    async def test_nova_rejection_names_the_command(
        self, motion_group, routines_api, status, error
    ):
        # GIVEN
        routines_api.start_command_routine_run.side_effect = api.ApiException(
            status=status, data=api.models.Error(message="command 0 'spline' is rejected")
        )

        # WHEN
        # THEN
        with pytest.raises(error, match="command 0"):
            await _routine().execute(motion_group)


class TestStop:
    @pytest.mark.parametrize("by_id", [False, True])
    async def test_stops_the_run_given_or_its_id(self, motion_group, routines_api, by_id):
        # GIVEN
        run = _run()

        # WHEN
        stopped = await CommandRoutine.stop(motion_group, run.run if by_id else run)

        # THEN
        routines_api.stop_command_routine_run.assert_awaited_once_with(
            cell="test_cell", run="run-1"
        )
        assert stopped.state == api.models.CommandRoutineRunState.STOPPED


def _nova_mock(**api_returns) -> Nova:
    nova = Mock(spec=Nova)
    datasets_api = Mock()
    for name, value in api_returns.items():
        mock = (
            AsyncMock(side_effect=value)
            if isinstance(value, Exception)
            else AsyncMock(return_value=value)
        )
        setattr(datasets_api, name, mock)
    nova.api = Mock(datasets_api=datasets_api)
    return nova


class TestResolve:
    async def test_returns_the_routine_resolved_by_the_instance(self):
        # GIVEN
        resolved = command_routine("pick-and-place", commands=[move_joint_ptp([0.0] * 6)])
        nova = _nova_mock(resolve_dataset_command_routine=resolved)

        # WHEN
        routine = await CommandRoutine.resolve(
            nova, "default", "pick-and-place", cell="cell", revision=2
        )

        # THEN
        assert routine == resolved
        nova.api.datasets_api.resolve_dataset_command_routine.assert_awaited_once_with(
            cell="cell", dataset="default", command_routine="pick-and-place", revision=2
        )

    async def test_missing_routine_raises_not_found(self):
        # GIVEN
        nova = _nova_mock(
            resolve_dataset_command_routine=api.exceptions.NotFoundException(
                status=404, reason="not found"
            )
        )

        # WHEN
        # THEN
        with pytest.raises(DatasetNotFoundError):
            await CommandRoutine.resolve(nova, "default", "missing")

    async def test_unresolvable_pose_raises_dataset_error_naming_it(self):
        # GIVEN
        nova = _nova_mock(
            resolve_dataset_command_routine=api.exceptions.UnprocessableEntityException(
                status=422, body='{"message": "pose \'slot\' cannot be resolved"}'
            )
        )

        # WHEN
        # THEN
        with pytest.raises(DatasetError, match="slot"):
            await CommandRoutine.resolve(nova, "default", "pick-and-place")
