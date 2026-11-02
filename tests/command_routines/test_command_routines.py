"""Unit tests for planning and executing a CommandRoutine through `nova.command_routines`.

Behaviour is asserted at the public functions only: what NOVA's command routine execution API
was asked to plan, run and stop, and the errors the caller sees. The API is a double.
"""

import asyncio
from unittest.mock import AsyncMock, Mock

import pytest

from nova import api
from nova import command_routines as cr
from nova.command_routines import (
    CommandRoutineError,
    RoutineRunFailed,
    UnsupportedRoutineCommand,
    at_path_fraction,
    command_routine,
    marker,
    move_joint_ptp,
    set_io,
    wait_for_time,
)
from nova.core.nova import Nova

_State = api.models.CommandRoutineRunState
_TRAJECTORY = api.models.JointTrajectory(
    joint_positions=[[0.0] * 6, [0.1] * 6], times=[0.0, 1.0], locations=[0.0, 1.0]
)


def _routine(*commands) -> api.models.CommandRoutine:
    return command_routine(
        "r",
        motion_group="0@ctrl",
        tcp="Flange",
        start_joint_position=[0.0] * 6,
        commands=list(commands) or [move_joint_ptp([0.1] * 6)],
    )


def _run(state: _State, error: str | None = None) -> api.models.CommandRoutineRun:
    return api.models.CommandRoutineRun(run="run-1", state=state, error=error)


def _nova_mock(*run_states: api.models.CommandRoutineRun) -> Nova:
    """A Nova whose run reports ``run_states`` one per poll, then stays in the last."""
    states = list(run_states) or [_run(_State.COMPLETED)]
    routines = Mock()
    routines.plan_command_routine = AsyncMock(
        side_effect=lambda cell, command_routine, controller: api.models.PlannedCommandRoutine(
            command_routine=command_routine, joint_trajectory=_TRAJECTORY
        )
    )
    routines.start_command_routine_run = AsyncMock(return_value=_run(_State.PREPARING))
    routines.get_command_routine_run = AsyncMock(
        side_effect=lambda cell, run: states.pop(0) if len(states) > 1 else states[0]
    )
    routines.stop_command_routine_run = AsyncMock(return_value=_run(_State.STOPPED))
    nova = Mock(spec=Nova)
    nova.api = Mock(command_routine_execution_api=routines)
    return nova


def _routines(nova: Nova):
    return nova.api.command_routine_execution_api


def _api_error(status: int, message: str) -> api.ApiException:
    return api.ApiException(status=status, data=api.models.Error(message=message))


class TestPlan:
    async def test_plans_the_whole_routine_on_the_instance(self):
        nova = _nova_mock()
        routine = _routine(move_joint_ptp([0.1] * 6), set_io("valve", True), marker("done"))

        planned = await cr.plan(nova, routine, cell="cell", controller="ur")

        assert planned.joint_trajectory == _TRAJECTORY
        _routines(nova).plan_command_routine.assert_awaited_once_with(
            cell="cell", command_routine=routine, controller="ur"
        )
        _routines(nova).start_command_routine_run.assert_not_awaited()

    async def test_a_feature_nova_cannot_execute_yet_raises_unsupported_routine_command(self):
        nova = _nova_mock()
        message = "Command 0 (motion_command): a joint PTP to a pose target is not implemented yet"
        _routines(nova).plan_command_routine.side_effect = _api_error(501, message)

        with pytest.raises(UnsupportedRoutineCommand, match=r"Command 0 \(motion_command\)"):
            await cr.plan(nova, _routine())

    async def test_a_rejected_routine_raises_command_routine_error_naming_the_command(self):
        nova = _nova_mock()
        message = "Command 0 (motion_command): target is the local pose 'x'"
        _routines(nova).plan_command_routine.side_effect = _api_error(422, message)

        with pytest.raises(CommandRoutineError, match="local pose 'x'"):
            await cr.plan(nova, _routine())


class TestExecute:
    async def test_runs_the_routine_until_it_completes(self):
        nova = _nova_mock(_run(_State.RUNNING), _run(_State.RUNNING), _run(_State.COMPLETED))
        routine = _routine(
            move_joint_ptp([0.1] * 6),
            wait_for_time(500),
            set_io("valve", True, at=at_path_fraction(0.5)),
            move_joint_ptp([0.0] * 6),
        )

        async with asyncio.timeout(5):
            run = await cr.execute(nova, routine, cell="cell")

        assert run.state == _State.COMPLETED
        _routines(nova).start_command_routine_run.assert_awaited_once_with(
            cell="cell", command_routine=routine, controller=None
        )
        assert _routines(nova).get_command_routine_run.await_count == 3

    @pytest.mark.parametrize("state", [_State.FAILED, _State.STOPPED])
    async def test_a_run_that_does_not_complete_raises(self, state):
        nova = _nova_mock(_run(_State.RUNNING), _run(state, "IO timed out"))

        with pytest.raises(RoutineRunFailed, match=state.value) as error:
            async with asyncio.timeout(5):
                await cr.execute(nova, _routine())

        assert error.value.run.state == state

    async def test_a_rejected_start_raises_before_anything_runs(self):
        nova = _nova_mock()
        message = "Command 1 (motion_command): a circle via a joint position is not implemented yet"
        _routines(nova).start_command_routine_run.side_effect = _api_error(501, message)

        with pytest.raises(UnsupportedRoutineCommand, match=r"Command 1"):
            await cr.execute(nova, _routine())

        _routines(nova).get_command_routine_run.assert_not_awaited()
        _routines(nova).stop_command_routine_run.assert_not_awaited()


class TestStop:
    async def test_cancelling_the_call_stops_the_run(self):
        nova = _nova_mock(_run(_State.RUNNING))
        task = asyncio.create_task(cr.execute(nova, _routine(), cell="cell"))
        async with asyncio.timeout(5):
            while not _routines(nova).get_command_routine_run.await_count:
                await asyncio.sleep(0.01)

            task.cancel()
            with pytest.raises(asyncio.CancelledError):
                await task

        _routines(nova).stop_command_routine_run.assert_awaited_once_with(cell="cell", run="run-1")

    async def test_a_run_starting_while_the_call_is_cancelled_is_stopped(self):
        nova = _nova_mock(_run(_State.RUNNING))
        requested = asyncio.Event()

        async def slow_start(cell, command_routine, controller):
            requested.set()
            await asyncio.sleep(0.05)  # NOVA plans before the run starts
            return _run(_State.PREPARING)

        _routines(nova).start_command_routine_run.side_effect = slow_start
        task = asyncio.create_task(cr.execute(nova, _routine(), cell="cell"))
        async with asyncio.timeout(5):
            await requested.wait()
            task.cancel()
            with pytest.raises(asyncio.CancelledError):
                await task

        _routines(nova).stop_command_routine_run.assert_awaited_once_with(cell="cell", run="run-1")
        _routines(nova).get_command_routine_run.assert_not_awaited()

    async def test_a_run_that_can_no_longer_be_followed_is_stopped(self):
        nova = _nova_mock()
        unavailable = api.exceptions.ServiceException(status=503, reason="unavailable")
        _routines(nova).get_command_routine_run.side_effect = unavailable

        with pytest.raises(api.exceptions.ServiceException):
            async with asyncio.timeout(5):
                await cr.execute(nova, _routine(), cell="cell")

        _routines(nova).stop_command_routine_run.assert_awaited_once_with(cell="cell", run="run-1")
