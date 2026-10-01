"""``PauseResumeStrategy.CONTROLLER``: the controller resumes IO pauses itself.

The frame sequences replay the semantics of robotics/wbr!2384 (``PauseOnIO.auto_resume``,
read from the MR, no instance runs it yet): the robot brakes on path while the controller
still reports ``RUNNING``, is then held as ``WAIT_FOR_IO`` (not ``PAUSED_ON_IO``), and the
controller resumes with ``RUNNING`` once the condition clears — no start from the client.
A condition that already holds at the start keeps the robot in ``WAIT_FOR_IO`` before it
ever moves.

The behaviour tests replace only the injection of the flag (``with_auto_resume``): the
installed API client may predate ``auto_resume``. The wire test runs once it has the field.
"""

from __future__ import annotations

import asyncio
import logging

import pytest

from nova import api
from nova.cell.io_condition import supports_auto_resume, with_auto_resume
from nova.cell.movement_controller.move_forward import move_forward
from nova.cell.movement_controller.policy import (
    ExecutionPolicy,
    MissedAutoResume,
    PauseResumeStrategy,
    set_default_execution_policy,
)
from nova.cell.movement_controller.standstill import StandstillConfig
from nova.cell.movement_controller.trajectory_state_machine import (
    PauseReason,
    TrajectoryExecutionMachine,
)
from nova.cell.trajectory_executor import GroupArgs, _session_policy
from nova.exceptions import ResumeNotTakenUp
from tests.cell.test_robust_execution import (
    _collect,
    _FedStates,
    _pause_condition,
    _Signal,
    _start_count,
    _starts_become,
    _state,
    _supervised_context,
    ended,
    parked,
    running,
    wait_for_io,
)

CONTROLLER = ExecutionPolicy(
    pause_resume=PauseResumeStrategy.CONTROLLER, resume_detect_s=0.05, resume_window_s=0.3
)


@pytest.fixture
def flag_injection(monkeypatch):
    """Stand in for the client field: record which conditions were sent with auto_resume."""
    sent: list[api.models.PauseOnIO] = []

    def inject(pause_on_io):
        sent.append(pause_on_io)
        return pause_on_io

    monkeypatch.setattr("nova.cell.movement_controller.trajectory_cursor.with_auto_resume", inject)
    return sent


async def _republish(states: _FedStates, frame, done, timeout: float = 5.0) -> None:
    async with asyncio.timeout(timeout):
        while not done():
            states.feed(frame)
            await asyncio.sleep(0.005)


# ---------------------------------------------------------------------------
# Policy
# ---------------------------------------------------------------------------


class TestPolicy:
    def test_defaults_keep_the_sdk_resume_and_fail_on_a_missed_auto_resume(self):
        policy = ExecutionPolicy()
        assert policy.pause_resume is PauseResumeStrategy.SDK
        assert not policy.controller_resumes
        assert policy.missed_auto_resume is MissedAutoResume.FAIL

    def test_environment_selects_both_knobs(self):
        policy = ExecutionPolicy.from_env(
            {"NOVA_PAUSE_RESUME": "Controller", "NOVA_MISSED_AUTO_RESUME": "start"}
        )
        assert policy.controller_resumes
        assert policy.effective_missed_auto_resume is MissedAutoResume.START

    def test_an_unknown_value_is_rejected(self):
        with pytest.raises(ValueError, match="NOVA_PAUSE_RESUME"):
            ExecutionPolicy.from_env({"NOVA_PAUSE_RESUME": "plc"})

    def test_strict_always_fails_on_a_missed_auto_resume(self):
        policy = ExecutionPolicy.from_env(
            {"NOVA_EXECUTION_POLICY": "strict", "NOVA_MISSED_AUTO_RESUME": "start"}
        )
        assert policy.missed_auto_resume is MissedAutoResume.START
        assert policy.effective_missed_auto_resume is MissedAutoResume.FAIL


# ---------------------------------------------------------------------------
# Wire
# ---------------------------------------------------------------------------


@pytest.mark.skipif(supports_auto_resume(), reason="the installed client has auto_resume")
def test_without_client_support_the_controller_strategy_fails_loudly():
    with pytest.raises(RuntimeError, match="auto_resume"):
        with_auto_resume(_pause_condition())


@pytest.mark.skipif(not supports_auto_resume(), reason="client predates PauseOnIO.auto_resume")
async def test_only_the_controller_strategy_sends_auto_resume():
    # With the field in the client, the SDK strategy sends its default (false).
    for policy, expected in ((CONTROLLER, True), (ExecutionPolicy(), False)):
        states, signal, requests = _FedStates(), _Signal(), []
        signal.set(pausing=False)
        run = asyncio.create_task(
            _collect(move_forward(_supervised_context(states, signal, policy)), requests)
        )
        states.feed(_state(True))
        await _starts_become(requests, 1)
        start = next(r for r in requests if isinstance(r, api.models.StartMovementRequest))
        assert start.pause_on_io is not None
        assert getattr(start.pause_on_io, "auto_resume", None) is expected
        assert ('"auto_resume":true' in start.model_dump_json(exclude_none=True)) is bool(expected)
        states.feed(running(1.0), ended(3.0), ended(3.0), ended(3.0))
        async with asyncio.timeout(5):
            await run


def test_the_callers_condition_object_is_left_untouched():
    if not supports_auto_resume():
        pytest.skip("client predates PauseOnIO.auto_resume")
    condition = _pause_condition()
    sent = with_auto_resume(condition)
    assert sent.auto_resume is True  # type: ignore[attr-defined]
    assert getattr(condition, "auto_resume", False) is False


# ---------------------------------------------------------------------------
# State machine
# ---------------------------------------------------------------------------


def _armed_machine(*, auto_resume: bool) -> TrajectoryExecutionMachine:
    machine = TrajectoryExecutionMachine(StandstillConfig.passthrough(), strict=True)
    machine.arm(pause_on_io_armed=True, auto_resume=auto_resume)
    return machine


def _feed(machine: TrajectoryExecutionMachine, *frames) -> None:
    for frame in frames:
        machine.process_motion_state(frame)


class TestMachine:
    def test_wait_for_io_after_motion_is_an_io_pause_the_controller_resumes(self):
        machine = _armed_machine(auto_resume=True)
        _feed(machine, running(0.5), running(0.8))  # braking is still reported RUNNING
        _feed(machine, wait_for_io(0.9))
        assert machine.is_paused
        assert machine.pause_reason is PauseReason.IO
        assert machine.is_paused_on_io

        _feed(machine, wait_for_io(0.9), wait_for_io(0.9))  # the hold is re-published
        assert machine.is_paused_on_io

        _feed(machine, running(1.0))  # the controller resumed on its own
        assert machine.is_executing
        _feed(machine, ended(3.0))
        assert machine.is_ended

    def test_a_hold_settling_while_braking_pauses_on_standstill(self):
        machine = _armed_machine(auto_resume=True)
        _feed(machine, running(0.5))
        machine.process_motion_state(_state(False, wait_for_io(0.9).execute))
        assert machine.is_pausing
        _feed(machine, wait_for_io(0.9))
        assert machine.is_paused_on_io

    def test_a_condition_holding_at_the_start_keeps_the_start_armed(self):
        machine = _armed_machine(auto_resume=True)
        _feed(machine, parked(), wait_for_io(0.0), wait_for_io(0.0))
        assert machine.is_armed
        _feed(machine, running(0.1))
        assert machine.is_executing

    def test_the_condition_turning_true_after_the_end_is_no_contradiction(self):
        machine = _armed_machine(auto_resume=True)
        _feed(machine, running(1.0), ended(3.0))
        assert machine.is_ended
        _feed(machine, wait_for_io(3.0), wait_for_io(3.0), ended(3.0))
        assert machine.is_ended
        assert not machine.is_error

    def test_without_auto_resume_wait_for_io_keeps_executing(self):
        machine = _armed_machine(auto_resume=False)
        _feed(machine, running(0.5), wait_for_io(0.9))
        assert machine.is_executing


# ---------------------------------------------------------------------------
# One-shot execution
# ---------------------------------------------------------------------------


async def _paused_by_the_controller(states, signal, requests) -> None:
    """Moving, then the signal pauses: braking (RUNNING) and the WAIT_FOR_IO hold."""
    states.feed(_state(True))  # the cursor dispatches once the stream is live
    await _starts_become(requests, 1)
    states.feed(running(0.5))
    signal.set(pausing=True)
    states.feed(running(0.8), wait_for_io(0.9), wait_for_io(0.9))


async def test_the_controller_resumes_and_no_resume_start_is_sent(flag_injection):
    states, signal, requests = _FedStates(), _Signal(), []
    signal.set(pausing=False)
    run = asyncio.create_task(
        _collect(move_forward(_supervised_context(states, signal, CONTROLLER)), requests)
    )
    await _paused_by_the_controller(states, signal, requests)
    await asyncio.sleep(0.1)
    assert not run.done(), "an IO hold must not end the execution"

    signal.set(pausing=False)
    states.feed(wait_for_io(0.9), running(1.2), running(2.0), ended(3.0), ended(3.0), ended(3.0))
    async with asyncio.timeout(5):
        await run

    assert _start_count(requests) == 1, "the controller resumed; the SDK sent no start"
    assert flag_injection, "the start carried the condition with auto_resume"


async def test_a_condition_holding_at_the_start_waits_without_a_start(flag_injection):
    states, signal, requests = _FedStates(), _Signal(), []  # signal pausing from the start
    run = asyncio.create_task(
        _collect(move_forward(_supervised_context(states, signal, CONTROLLER)), requests)
    )
    states.feed(_state(True))
    await _starts_become(requests, 1)
    states.feed(parked(), wait_for_io(0.0), wait_for_io(0.0))
    await asyncio.sleep(0.1)
    assert not run.done()

    signal.set(pausing=False)
    states.feed(running(0.2), ended(3.0), ended(3.0), ended(3.0))
    async with asyncio.timeout(5):
        await run
    assert _start_count(requests) == 1


async def test_a_missed_auto_resume_fails_by_default(flag_injection):
    states, signal, requests = _FedStates(), _Signal(), []
    signal.set(pausing=False)
    run = asyncio.create_task(
        _collect(move_forward(_supervised_context(states, signal, CONTROLLER)), requests)
    )
    await _paused_by_the_controller(states, signal, requests)
    await asyncio.sleep(0.02)

    signal.set(pausing=False)  # released, but the controller keeps holding
    feeder = asyncio.create_task(_republish(states, wait_for_io(0.9), run.done))
    with pytest.raises(ResumeNotTakenUp, match="did not resume the IO pause"):
        async with asyncio.timeout(5):
            await run
    await feeder
    assert _start_count(requests) == 1, "an upstream defect is not glossed over with a start"


async def test_a_missed_auto_resume_is_started_from_the_sdk_when_configured(flag_injection, caplog):
    policy = ExecutionPolicy(
        pause_resume=PauseResumeStrategy.CONTROLLER,
        missed_auto_resume=MissedAutoResume.START,
        resume_detect_s=0.05,
        resume_window_s=0.3,
    )
    states, signal, requests = _FedStates(), _Signal(), []
    signal.set(pausing=False)
    run = asyncio.create_task(
        _collect(move_forward(_supervised_context(states, signal, policy)), requests)
    )
    await _paused_by_the_controller(states, signal, requests)
    await asyncio.sleep(0.02)

    with caplog.at_level(logging.WARNING):
        signal.set(pausing=False)
        await _republish(states, wait_for_io(0.9), lambda: _start_count(requests) >= 2)
    assert "sending the start from the SDK" in caplog.text

    states.feed(running(1.2), ended(3.0), ended(3.0), ended(3.0))
    async with asyncio.timeout(5):
        await run
    assert _start_count(requests) == 2


async def test_strict_fails_a_missed_auto_resume_even_when_start_is_configured(flag_injection):
    policy = ExecutionPolicy(
        strict=True,
        standstill=StandstillConfig.passthrough(),
        pause_resume=PauseResumeStrategy.CONTROLLER,
        missed_auto_resume=MissedAutoResume.START,
        resume_detect_s=0.05,
        resume_window_s=0.3,
    )
    states, signal, requests = _FedStates(), _Signal(), []
    signal.set(pausing=False)
    run = asyncio.create_task(
        _collect(move_forward(_supervised_context(states, signal, policy)), requests)
    )
    await _paused_by_the_controller(states, signal, requests)
    await asyncio.sleep(0.02)
    signal.set(pausing=False)
    feeder = asyncio.create_task(_republish(states, wait_for_io(0.9), run.done))
    with pytest.raises(ResumeNotTakenUp):
        async with asyncio.timeout(5):
            await run
    await feeder
    assert _start_count(requests) == 1


async def test_a_condition_back_before_the_check_is_an_ordinary_hold(flag_injection):
    states, signal, requests = _FedStates(), _Signal(), []
    signal.set(pausing=False)
    run = asyncio.create_task(
        _collect(move_forward(_supervised_context(states, signal, CONTROLLER)), requests)
    )
    await _paused_by_the_controller(states, signal, requests)
    await asyncio.sleep(0.02)

    signal.set(pausing=False)
    await asyncio.sleep(0.01)
    signal.set(pausing=True)  # chattering signal: holds again before the controller moved
    for _ in range(20):
        states.feed(wait_for_io(0.9))
        await asyncio.sleep(0.01)
    assert not run.done(), "a signal that holds again is no missed resume"

    signal.set(pausing=False)
    states.feed(running(1.2), ended(3.0), ended(3.0), ended(3.0))
    async with asyncio.timeout(5):
        await run
    assert _start_count(requests) == 1


# ---------------------------------------------------------------------------
# Synchronized sessions
# ---------------------------------------------------------------------------


def test_synchronized_sessions_keep_the_sdk_strategy(caplog):
    set_default_execution_policy(ExecutionPolicy(pause_resume=PauseResumeStrategy.CONTROLLER))
    with caplog.at_level(logging.WARNING):
        policy = _session_policy({"a": GroupArgs(pause_on_io=_pause_condition())})
    assert policy.pause_resume is PauseResumeStrategy.SDK
    assert "not supported in synchronized sessions" in caplog.text

    caplog.clear()
    assert _session_policy(None).pause_resume is PauseResumeStrategy.SDK
    assert "not supported" not in caplog.text, "no warning without a pause condition"


async def test_the_bus_loss_guard_still_pauses_and_resumes_from_the_sdk(flag_injection):
    """The controller keeps using a stale bus value when the bus-IO service is gone
    (wbr!2384 too), so the SDK guard pauses with a user pause — which only a start
    resumes, under either strategy."""
    states, signal, requests = _FedStates(), _Signal(), []
    signal.set(pausing=False)
    loss = asyncio.Event()

    async def wait_for_signal_loss():
        await loss.wait()
        loss.clear()

    context = _supervised_context(states, signal, CONTROLLER).model_copy(
        update={"wait_for_pause_signal_loss": wait_for_signal_loss}
    )
    run = asyncio.create_task(_collect(move_forward(context), requests))
    states.feed(_state(True))
    await _starts_become(requests, 1)
    states.feed(running(0.5))
    await asyncio.sleep(0.01)

    signal.set(pausing=True)  # the bus is gone; the signal is unknown — treat as pausing
    loss.set()
    async with asyncio.timeout(5):
        while not any(isinstance(r, api.models.PauseMovementRequest) for r in requests):
            await asyncio.sleep(0)
    states.feed(_state(False, parked(0.8).execute), parked(0.8), parked(0.8), parked(0.8))
    await asyncio.sleep(0.05)
    assert _start_count(requests) == 1

    signal.set(pausing=False)  # bus back, signal allows motion
    await _starts_become(requests, 2)
    states.feed(parked(0.8), running(1.2), ended(3.0), ended(3.0), ended(3.0))
    async with asyncio.timeout(5):
        await run
    assert _start_count(requests) == 2
