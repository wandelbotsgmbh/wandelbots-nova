"""Planning and running a CommandRoutine through ``MotionGroup.plan`` / ``plan_and_execute``.

The robot branches on what it receives: an action list takes the usual path, a CommandRoutine goes
to NOVA's command routines API as it is. Behaviour is asserted at the public entry points only:
what NOVA was asked, what comes back, and the errors the caller sees. The API is a double; the
routines of NOVA's own scenarios (``tests/command_routines/scenarios.py``) are the test data.
"""

from datetime import datetime, timezone
from unittest.mock import AsyncMock, MagicMock, Mock

import pytest

from nova import api
from nova.cell.motion_group import MotionGroup
from nova.cell.simulation import get_robot_controller
from nova.command_routines import CommandRoutineError, UnsupportedRoutineCommand
from nova.core.gateway import ApiGateway
from nova.types import Pose
from tests.command_routines.scenarios import Context, errors, features

pytestmark = pytest.mark.asyncio

_State = api.models.CommandRoutineRunState
_TRAJECTORY = api.models.JointTrajectory(
    joint_positions=[[0.0] * 6, [0.1] * 6], times=[0.0, 1.0], locations=[0.0, 1.0]
)
_SETUP = api.models.MotionGroupSetup(
    motion_group_model="KUKA_KR16_R2010_2",
    cycle_time=8,
    mounting=api.models.Pose(position=[0.0] * 3, orientation=[0.0] * 3),
    tcp_offset=api.models.Pose(position=[0.0] * 3, orientation=[0.0] * 3),
    global_limits=api.models.LimitSet(),
)
_CTX = Context(
    start=[0.0, -1.57, 1.57, 0.0, 1.57, 0.0],
    tcp_pose=Pose((700, 0, 600, 0, 3.14, 0)),
    setup=_SETUP,
    motion_group="0@other",
)
_FEATURES = features(_CTX)
_ERRORS = errors(_CTX)
_ROUTINE = _FEATURES["01-motion-types"]  # written for 0@other, not for the motion group below


def _run(state: _State = _State.RUNNING) -> api.models.CommandRoutineRun:
    return api.models.CommandRoutineRun(
        run="run-1",
        command_routine="r",
        state=state,
        created_at=datetime(2026, 1, 1, tzinfo=timezone.utc),
    )


def _motion_group() -> MotionGroup:
    routines = MagicMock()
    routines.plan_command_routine = AsyncMock(
        return_value=api.models.PlanCommandRoutineResponse(joint_trajectory=_TRAJECTORY)
    )
    routines.start_command_routine_run = AsyncMock(return_value=_run())
    routines.get_command_routine_run = AsyncMock()
    routines.stop_command_routine_run = AsyncMock(return_value=_run(_State.STOPPED))
    gateway = MagicMock(spec=ApiGateway)
    gateway.command_routines_api = routines
    return MotionGroup(
        api_client=gateway, cell="cell", controller_id="ctrl", motion_group_id="0@ctrl"
    )


def _routines(motion_group: MotionGroup):
    return motion_group._api_client.command_routines_api


def _plan_request(motion_group: MotionGroup):
    return _routines(motion_group).plan_command_routine.await_args.kwargs[
        "plan_command_routine_request"
    ]


def _start_request(motion_group: MotionGroup):
    return _routines(motion_group).start_command_routine_run.await_args.kwargs[
        "start_command_routine_run_request"
    ]


def _api_error(status: int, message: str) -> api.ApiException:
    return api.ApiException(status=status, data=api.models.Error(message=message))


class TestPlan:
    async def test_plans_the_routine_on_this_motion_group(self):
        motion_group = _motion_group()

        trajectory = await motion_group.plan(_ROUTINE)

        assert trajectory == _TRAJECTORY
        assert _routines(motion_group).plan_command_routine.await_args.kwargs["cell"] == "cell"
        assert _plan_request(motion_group).command_routine == _ROUTINE
        _routines(motion_group).start_command_routine_run.assert_not_awaited()

    async def test_plans_on_a_motion_group_other_than_the_routines(self):
        motion_group = _motion_group()
        assert _ROUTINE.motion_group.id == "0@other"

        await motion_group.plan(_ROUTINE)

        request = _plan_request(motion_group)
        assert request.motion_group == api.models.MotionGroupReference(id="0@ctrl")
        assert request.command_routine.motion_group.id == "0@other"  # the routine stays as it is

    async def test_an_io_only_routine_has_no_trajectory(self):
        motion_group = _motion_group()
        _routines(
            motion_group
        ).plan_command_routine.return_value = api.models.PlanCommandRoutineResponse(
            joint_trajectory=None
        )

        with pytest.raises(ValueError, match="I/O-only"):
            await motion_group.plan(_FEATURES["11-io-only-bus"])

    async def test_action_lists_do_not_use_the_routines_api(self):
        motion_group = _motion_group()

        with pytest.raises(ValueError, match="No actions provided"):
            await motion_group.plan([])

        _routines(motion_group).plan_command_routine.assert_not_awaited()


class TestPlanAndExecute:
    async def test_starts_the_routine_on_this_motion_group_and_returns_its_run(self):
        motion_group = _motion_group()

        run = await motion_group.plan_and_execute(_ROUTINE)

        assert run == _run()
        _routines(motion_group).start_command_routine_run.assert_awaited_once()
        assert _routines(motion_group).start_command_routine_run.await_args.kwargs["cell"] == "cell"
        assert _start_request(motion_group).command_routine == _ROUTINE
        _routines(motion_group).plan_command_routine.assert_not_awaited()

    async def test_does_not_wait_for_the_run(self):
        motion_group = _motion_group()

        await motion_group.plan_and_execute(_ROUTINE)

        _routines(motion_group).get_command_routine_run.assert_not_awaited()
        _routines(motion_group).stop_command_routine_run.assert_not_awaited()

    async def test_runs_on_a_motion_group_other_than_the_routines(self):
        motion_group = _motion_group()
        assert _ROUTINE.motion_group.id == "0@other"

        await motion_group.plan_and_execute(_ROUTINE)

        request = _start_request(motion_group)
        assert request.motion_group == api.models.MotionGroupReference(id="0@ctrl")
        assert request.command_routine.motion_group.id == "0@other"

    async def test_a_routine_without_a_motion_group_runs_on_this_one(self):
        motion_group = _motion_group()
        routine = _ROUTINE.model_copy(update={"motion_group": None})

        await motion_group.plan_and_execute(routine)

        assert _start_request(motion_group).motion_group.id == "0@ctrl"

    async def test_the_routines_speed_settings_reach_nova_as_authored(self):
        motion_group = _motion_group()
        routine = _FEATURES["10-motion-settings"]

        await motion_group.plan_and_execute(routine)

        sent = _start_request(motion_group).command_routine
        assert sent.default_motion_settings.limits_override.tcp_velocity_limit == 100.0
        assert sent.commands[1].motion_settings.limits_override.tcp_velocity_limit == 25.0
        assert sent.commands[0].motion_settings is None  # inherits the routine's default

    async def test_the_routines_own_tcp_may_be_repeated(self):
        motion_group = _motion_group()

        await motion_group.plan_and_execute(_ROUTINE, _ROUTINE.tcp)

        _routines(motion_group).start_command_routine_run.assert_awaited_once()


class TestStopRoutine:
    async def test_stops_a_run_given_as_run_or_id(self):
        motion_group = _motion_group()

        by_run = await motion_group.stop_routine(_run())
        await motion_group.stop_routine("run-1")

        assert by_run.state == _State.STOPPED
        assert _routines(motion_group).stop_command_routine_run.await_args_list == [
            ((), {"cell": "cell", "run": "run-1"}),
            ((), {"cell": "cell", "run": "run-1"}),
        ]

    async def test_a_missing_run_raises_command_routine_error(self):
        motion_group = _motion_group()
        _routines(motion_group).stop_command_routine_run.side_effect = _api_error(
            404, "run not found"
        )

        with pytest.raises(CommandRoutineError, match="run not found"):
            await motion_group.stop_routine("gone")


class TestScenarios:
    """Every scenario of NOVA's routines service reaches the API as authored."""

    @pytest.mark.parametrize("name", _FEATURES)
    async def test_feature_routine_is_sent_unchanged(self, name):
        motion_group = _motion_group()

        await motion_group.plan(_FEATURES[name])
        await motion_group.plan_and_execute(_FEATURES[name])

        assert _plan_request(motion_group).command_routine == _FEATURES[name]
        assert _start_request(motion_group).command_routine == _FEATURES[name]

    @pytest.mark.parametrize("name", [n for n, failure in _ERRORS.items() if failure.status])
    async def test_rejection_is_raised_naming_the_command(self, name):
        failure = _ERRORS[name]
        motion_group = _motion_group()
        error = _api_error(failure.status, f"Command 0 (motion_command): rejected for {name}")
        _routines(motion_group).plan_command_routine.side_effect = error
        _routines(motion_group).start_command_routine_run.side_effect = error
        expected = UnsupportedRoutineCommand if failure.status == 501 else CommandRoutineError

        for call in (motion_group.plan, motion_group.plan_and_execute):
            with pytest.raises(expected, match=name) as raised:
                await call(failure.routine)
            assert type(raised.value) is expected  # 501 is the only "unsupported"


class TestMotionSettings:
    """The speed overrides of a routine travel to NOVA as authored."""

    def _wire(self, routine: api.models.CommandRoutine) -> dict:
        request = api.models.StartCommandRoutineRunRequest(command_routine=routine)
        return api.ApiClient().sanitize_for_serialization(request)["command_routine"]

    def test_default_and_per_motion_speed_are_sent(self):
        wire = self._wire(_FEATURES["10-motion-settings"])

        limits = [
            command.get("motion_settings", {}).get("limits_override", {}).get("tcp_velocity_limit")
            for command in wire["commands"]
        ]
        assert wire["default_motion_settings"]["limits_override"]["tcp_velocity_limit"] == 100.0
        assert limits == [None, 25.0, None]  # the first motion inherits the default

    @pytest.mark.xfail(
        strict=True,
        reason="the generated client drops explicit nulls, so `blending: null` cannot clear the "
        "routine's default blending",
    )
    def test_blending_null_clears_the_default(self):
        wire = self._wire(_FEATURES["10-motion-settings"])

        assert wire["commands"][2]["motion_settings"] == {"blending": None}

    @pytest.mark.xfail(
        strict=True,
        reason="the generated client drops explicit nulls, so an I/O-only routine loses "
        "`motion_group_setup: null` that declares it",
    )
    def test_io_only_routine_keeps_motion_group_setup_null(self):
        wire = self._wire(_FEATURES["11-io-only-bus"])

        assert wire["motion_group_setup"] is None


class TestRejection:
    """What a routine cannot take is rejected before NOVA is called."""

    async def test_rejects_arguments_that_override_the_routines_context(self):
        motion_group = _motion_group()

        with pytest.raises(ValueError, match="start_joint_position"):
            await motion_group.plan(_ROUTINE, start_joint_position=(0.0,) * 6)
        with pytest.raises(ValueError, match="motion_group_setup"):
            await motion_group.plan(_ROUTINE, motion_group_setup=_SETUP)
        with pytest.raises(ValueError, match="tcp"):
            await motion_group.plan(_ROUTINE, "Other")
        with pytest.raises(ValueError, match="movement_controller"):
            await motion_group.plan_and_execute(_ROUTINE, movement_controller=Mock())
        with pytest.raises(ValueError, match="state_stream_rate_msecs"):
            await motion_group.plan_and_execute(_ROUTINE, state_stream_rate_msecs=10)

        _routines(motion_group).plan_command_routine.assert_not_awaited()
        _routines(motion_group).start_command_routine_run.assert_not_awaited()

    async def test_execute_and_the_streams_do_not_take_a_routine(self):
        motion_group = _motion_group()

        with pytest.raises(ValueError, match="plan_and_execute"):
            await motion_group.execute(_TRAJECTORY, "Flange", _ROUTINE)
        with pytest.raises(ValueError, match="plan_and_execute"):
            async for _ in motion_group.stream_execute(_TRAJECTORY, "Flange", _ROUTINE):
                pass
        with pytest.raises(ValueError, match="plan_and_execute"):
            async for _ in motion_group.stream_plan_and_execute(_ROUTINE):
                pass

        _routines(motion_group).start_command_routine_run.assert_not_awaited()


class TestRobotsWithoutNova:
    """A robot that does not talk to NOVA's routines API says so."""

    async def test_a_simulated_robot_cannot_plan_or_run_routines(self):
        robot = next(iter(get_robot_controller().get_motion_groups().values()))

        with pytest.raises(NotImplementedError, match="cannot plan"):
            await robot.plan(_ROUTINE)
        with pytest.raises(NotImplementedError, match="cannot run"):
            await robot.plan_and_execute(_ROUTINE)
