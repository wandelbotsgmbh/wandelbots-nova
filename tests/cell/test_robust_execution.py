"""Execution under an unreliable ``standstill`` flag (the ``robust`` execution policy).

Pins the cases seen on the real cell (arg3-longterm-tests, 2026-09-29/30):

* a standstill flicker on the parked frame before the controller took up the start
  concluded a user pause, and the ``RUNNING`` that followed failed the execution with
  ``UnexpectedTrajectoryState`` (combined_loop, synchronized linear legs);
* a resume out of an IO pause the controller did not take up kept the machine
  ``armed`` on the old ``PAUSED_ON_IO`` forever (the ``pause`` app).

And the policy contract around them: the strict preset keeps failing on contradictions,
the robust one warns and follows the controller, the diagnose preset fails on jitter.
"""

from __future__ import annotations

import asyncio
from datetime import datetime, timezone
from typing import AsyncIterator

import pytest

from nova import api
from nova.actions.container import CombinedActions, MovementControllerContext
from nova.cell.movement_controller.move_forward import move_forward
from nova.cell.movement_controller.policy import ExecutionPolicy
from nova.cell.movement_controller.standstill import StandstillConfig
from nova.cell.movement_controller.trajectory_cursor import TrajectoryCursor
from nova.cell.movement_controller.trajectory_state_machine import (
    PauseReason,
    TrajectoryExecutionMachine,
)
from nova.exceptions import ResumeNotTakenUp

# ---------------------------------------------------------------------------
# Frames
# ---------------------------------------------------------------------------


def _state(
    standstill: bool, execute: api.models.Execute | None = None
) -> api.models.MotionGroupState:
    return api.models.MotionGroupState(
        timestamp=datetime.now(timezone.utc),
        sequence_number=1,
        description_revision=0,
        motion_group="mg-0",
        controller="ctrl-0",
        joint_position=[0.0] * 6,
        joint_limit_reached=api.models.MotionGroupStateJointLimitReached(limit_reached=[False] * 6),
        standstill=standstill,
        execute=execute,
    )


def _execute(trajectory_state, location: float) -> api.models.Execute:
    return api.models.Execute(
        joint_position=[0.0] * 6,
        details=api.models.TrajectoryDetails(
            trajectory="traj-1", location=location, state=trajectory_state
        ),
    )


def _rail(
    trajectory_state, location: float, position_mm: float, *, standstill: bool = True
) -> api.models.MotionGroupState:
    state = _state(standstill, _execute(trajectory_state, location))
    return state.model_copy(update={"joint_position": [position_mm]})


def running(location: float, *, standstill: bool = False) -> api.models.MotionGroupState:
    return _state(standstill, _execute(api.models.TrajectoryRunning(time_to_end=1000), location))


def parked(location: float = 0.0, *, standstill: bool = True) -> api.models.MotionGroupState:
    return _state(standstill, _execute(api.models.TrajectoryPausedByUser(), location))


def paused_on_io(location: float, *, standstill: bool = True) -> api.models.MotionGroupState:
    return _state(standstill, _execute(api.models.TrajectoryPausedOnIO(), location))


def ended(location: float, *, standstill: bool = True) -> api.models.MotionGroupState:
    return _state(standstill, _execute(api.models.TrajectoryEnded(), location))


def wait_for_io(location: float = 0.0) -> api.models.MotionGroupState:
    return _state(True, _execute(api.models.TrajectoryWaitForIO(), location))


def _robust_machine() -> TrajectoryExecutionMachine:
    return TrajectoryExecutionMachine(StandstillConfig.robust(), strict=False)


def _feed(machine: TrajectoryExecutionMachine, *frames: api.models.MotionGroupState) -> None:
    for frame in frames:
        machine.process_motion_state(frame)


# The observed failure: armed on the parked frame, one standstill flicker, the
# parked frame again, then the controller takes up the start.
_FLICKER_BEFORE_TAKE_UP = (
    parked(0.0),
    parked(0.0, standstill=False),
    parked(0.0),
    running(0.0, standstill=True),
)


# ---------------------------------------------------------------------------
# State machine
# ---------------------------------------------------------------------------


class TestStandstillFlickerBeforeTheStart:
    def test_the_strict_machine_executes_the_observed_sequence_too(self):
        """Commanded progress, not the flag, proves a start was taken up: even the raw
        flag (strict preset) no longer concludes a pause from the parked flicker."""
        machine = TrajectoryExecutionMachine()
        machine.arm()

        _feed(machine, *_FLICKER_BEFORE_TAKE_UP)

        assert machine.is_executing
        assert machine.failure_reason is None

    def test_a_long_flicker_without_commanded_progress_is_no_pause(self):
        machine = _robust_machine()
        machine.arm()

        _feed(
            machine,
            parked(0.0),
            *[parked(0.0, standstill=False)] * 5,
            parked(0.0),
            parked(0.0),
            parked(0.0),
        )

        assert machine.is_armed
        _feed(machine, running(0.0, standstill=True))
        assert machine.is_executing

    def test_a_stale_frame_of_another_trajectory_is_no_progress(self):
        machine = _robust_machine()
        machine.arm()
        other = _state(
            True,
            api.models.Execute(
                joint_position=[0.0] * 6,
                details=api.models.TrajectoryDetails(
                    trajectory="previous", location=1.96, state=api.models.TrajectoryPausedByUser()
                ),
            ),
        )

        _feed(machine, other, parked(0.0), parked(0.0), parked(0.0))

        assert machine.is_armed

    def test_the_robust_machine_ignores_the_flicker_and_executes(self):
        machine = _robust_machine()
        machine.arm()

        _feed(machine, *_FLICKER_BEFORE_TAKE_UP)

        assert machine.is_executing
        assert machine.failure_reason is None

    def test_a_real_motion_start_is_seen_at_once_when_the_location_moves(self):
        machine = _robust_machine()
        machine.arm()

        _feed(machine, parked(0.0), parked(0.01, standstill=False))

        assert machine.last_reading is not None and machine.last_reading.at_rest is False
        # A user pause after that motion is a real pause.
        _feed(machine, parked(0.02), parked(0.02))
        assert machine.is_paused
        assert machine.pause_reason is PauseReason.USER

    def test_the_diagnose_preset_fails_on_the_flicker(self):
        machine = TrajectoryExecutionMachine(StandstillConfig.robust().with_strict(True))
        machine.arm()

        _feed(machine, parked(0.0), parked(0.0, standstill=False), parked(0.0))

        assert machine.is_error
        assert "flickered" in (machine.failure_reason or "")


class TestContradictionsAreFollowedWhenNotStrict:
    def test_running_while_paused_by_user_resumes_tracking(self, caplog):
        machine = _robust_machine()
        machine.arm()
        _feed(machine, running(0.5), parked(0.8), parked(0.8), parked(0.8))
        assert machine.is_paused and machine.pause_reason is PauseReason.USER

        _feed(machine, running(0.9))

        assert machine.is_executing
        assert "following the controller" in caplog.text

    def test_running_after_the_end_resumes_tracking(self):
        machine = _robust_machine()
        machine.arm()
        _feed(machine, running(0.5), ended(1.0), ended(1.0))
        assert machine.is_ended

        _feed(machine, running(0.5))

        assert machine.is_executing

    def test_strict_still_fails(self):
        machine = TrajectoryExecutionMachine(StandstillConfig.robust(), strict=True)
        machine.arm()
        _feed(machine, running(0.5), ended(1.0), ended(1.0), running(0.5))

        assert machine.is_error


class TestPendingStart:
    def test_frames_before_the_start_was_sent_conclude_nothing(self):
        machine = TrajectoryExecutionMachine()
        machine.arm()
        _feed(machine, running(0.5), ended(3.0))
        assert machine.is_ended

        machine.expect_start()
        _feed(machine, ended(3.0), ended(3.0, standstill=False), ended(3.0))

        assert machine.is_pending

    def test_the_stale_filter_survives_pending(self):
        machine = TrajectoryExecutionMachine()
        machine.arm(pause_on_io_armed=True)
        _feed(machine, running(0.5), paused_on_io(1.0))
        assert machine.is_paused_on_io

        machine.expect_start(pause_on_io_armed=True)
        _feed(machine, paused_on_io(1.0))
        machine.arm()
        _feed(machine, paused_on_io(1.0), paused_on_io(1.0))

        assert machine.is_waiting_on_stale_terminal
        assert machine.resuming_from is PauseReason.IO
        _feed(machine, running(1.2))
        assert machine.is_executing

    def test_a_pause_requested_while_pending_concludes_on_the_parked_frame(self):
        machine = TrajectoryExecutionMachine()
        machine.expect_start()
        machine.request_pause()
        machine.arm()

        _feed(machine, parked(0.0))

        assert machine.is_paused


class TestPauseOnIOIntent:
    def test_an_io_pause_without_a_condition_fails_when_strict(self):
        machine = TrajectoryExecutionMachine()
        machine.arm(pause_on_io_armed=False)

        _feed(machine, running(0.5), paused_on_io(1.0))

        assert machine.is_error
        assert "no pause_on_io" in (machine.failure_reason or "")

    def test_an_io_pause_without_a_condition_is_followed_otherwise(self):
        machine = _robust_machine()
        machine.arm(pause_on_io_armed=False)

        _feed(machine, running(0.5), paused_on_io(1.0), paused_on_io(1.0))

        assert machine.is_paused_on_io

    def test_unknown_intent_accepts_the_io_pause(self):
        machine = TrajectoryExecutionMachine()
        machine.arm()

        _feed(machine, running(0.5), paused_on_io(1.0))

        assert machine.is_paused_on_io


def test_abandon_start_returns_to_the_io_pause_and_filters_again():
    machine = TrajectoryExecutionMachine()
    machine.arm(pause_on_io_armed=True)
    _feed(machine, running(0.5), paused_on_io(1.0))
    machine.arm(pause_on_io_armed=True)
    _feed(machine, paused_on_io(1.0))

    machine.abandon_start()

    assert machine.is_paused_on_io
    machine.arm(pause_on_io_armed=True)
    _feed(machine, paused_on_io(1.0))
    assert machine.is_waiting_on_stale_terminal


def test_abandon_start_needs_a_stale_io_resume():
    machine = TrajectoryExecutionMachine()
    machine.arm()
    with pytest.raises(RuntimeError):
        machine.abandon_start()


# ---------------------------------------------------------------------------
# Cursor: resume not taken up
# ---------------------------------------------------------------------------


class _Frames:
    def __init__(self):
        self._queue: asyncio.Queue[api.models.MotionGroupState] = asyncio.Queue()

    def feed(self, *states: api.models.MotionGroupState) -> None:
        for state in states:
            self._queue.put_nowait(state)

    async def stream(self) -> AsyncIterator[api.models.MotionGroupState]:
        while True:
            yield await self._queue.get()


async def _responses() -> AsyncIterator[api.models.ExecuteTrajectoryResponse]:
    yield api.models.InitializeMovementResponse()
    while True:
        yield api.models.StartMovementResponse()
        await asyncio.sleep(0)


def _pause_condition() -> api.models.PauseOnIO:
    return api.models.PauseOnIO(
        io=api.models.IOBooleanValue(io="hold", value=True),
        comparator=api.models.Comparator.COMPARATOR_EQUALS,
        io_origin=api.models.IOOrigin.BUS_IO,
    )


def _joint_trajectory() -> api.models.JointTrajectory:
    return api.models.JointTrajectory(
        joint_positions=[[0.0] * 6] * 4, times=[0.0, 1.0, 2.0, 3.0], locations=[0.0, 1.0, 2.0, 3.0]
    )


class _Clock:
    def __init__(self):
        self.now = 100.0

    def __call__(self) -> float:
        return self.now


async def _starts_become(requests: list, count: int) -> None:
    async with asyncio.timeout(5):
        while sum(isinstance(r, api.models.StartMovementRequest) for r in requests) < count:
            await asyncio.sleep(0)


@pytest.mark.asyncio
async def test_cursor_reports_a_resume_the_controller_did_not_take_up():
    frames = _Frames()
    clock = _Clock()
    frames.feed(_state(True), parked())
    cursor = TrajectoryCursor(
        motion_id="traj-1",
        motion_group_state_stream=frames.stream(),
        joint_trajectory=_joint_trajectory(),
        detach_on_standstill=True,
        emit_motion_events=False,
        pause_on_io=_pause_condition(),
        policy=ExecutionPolicy.robust(),
        clock=clock,
    )
    requests: list = []

    async def consume():
        async for request in cursor.cntrl(_responses()):
            requests.append(request)

    consumer = asyncio.create_task(consume())
    try:
        operation = cursor.forward()
        frames.feed(running(0.5), paused_on_io(1.0), paused_on_io(1.0))
        async with asyncio.timeout(5):
            first = await operation
        assert first.paused_on_io and not first.resume_not_taken_up

        resume = cursor.forward()
        await _starts_become(requests, 2)
        frames.feed(paused_on_io(1.0), paused_on_io(1.0))
        await asyncio.sleep(0.05)
        assert not resume.done(), "the stale pause alone must not conclude before the limit"

        clock.now += 0.6
        frames.feed(paused_on_io(1.0))
        async with asyncio.timeout(5):
            result = await resume

        assert result.resume_not_taken_up and result.paused_on_io
        assert result.final_location == 1.0
        assert not consumer.done(), "the cursor stays attached for another start"
    finally:
        cursor.detach()
        async with asyncio.timeout(5):
            await consumer


# ---------------------------------------------------------------------------
# move_forward: resume supervision
# ---------------------------------------------------------------------------


class _Signal:
    """A pause signal driven by the test, observed through push-style waiters."""

    def __init__(self):
        self.pausing = True
        self._changed = asyncio.Event()
        self.release_waits = 0

    def set(self, pausing: bool) -> None:
        self.pausing = pausing
        self._changed.set()
        self._changed = asyncio.Event()

    async def _until(self, pausing: bool) -> None:
        while self.pausing != pausing:
            await self._changed.wait()

    async def wait_for_release(self) -> None:
        self.release_waits += 1
        await self._until(False)

    async def wait_for_hold(self) -> None:
        await self._until(True)


class _FedStates:
    def __init__(self):
        self._queue: asyncio.Queue = asyncio.Queue()

    def feed(self, *states):
        for state in states:
            self._queue.put_nowait(state)

    def gen(self):
        async def _gen():
            while (state := await self._queue.get()) is not None:
                yield state

        return _gen

    def close(self):
        self._queue.put_nowait(None)


_FAST = ExecutionPolicy(resume_detect_s=0.05, resume_window_s=0.3)


def _supervised_context(states: _FedStates, signal: _Signal, policy: ExecutionPolicy):
    return MovementControllerContext(
        combined_actions=CombinedActions(items=()),
        motion_id="traj-1",
        motion_group_state_stream_gen=states.gen(),
        pause_on_io=_pause_condition(),
        wait_for_pause_on_io_release=signal.wait_for_release,
        wait_for_pause_on_io_hold=signal.wait_for_hold,
        execution_policy=policy,
    )


async def _collect(controller_fn, requests: list) -> None:
    async def responses():
        yield api.models.InitializeMovementResponse()
        while True:
            yield api.models.StartMovementResponse()
            await asyncio.sleep(0)

    async for request in controller_fn(responses()):
        requests.append(request)


async def _republish_until(states: _FedStates, frame, done) -> None:
    """Level-based controller: re-publish ``frame`` every few ms until ``done()``."""
    async with asyncio.timeout(5):
        while not done():
            states.feed(frame)
            await asyncio.sleep(0.005)


def _start_count(requests: list) -> int:
    return sum(isinstance(r, api.models.StartMovementRequest) for r in requests)


async def _paused_on_io_once(states: _FedStates, signal: _Signal, requests: list) -> None:
    states.feed(_state(True), running(0.5), paused_on_io(1.0), paused_on_io(1.0))
    async with asyncio.timeout(5):
        while signal.release_waits < 1:
            await asyncio.sleep(0)
    assert _start_count(requests) == 1


@pytest.mark.asyncio
async def test_an_ignored_resume_is_started_once_more_within_the_window(caplog):
    states, signal, requests = _FedStates(), _Signal(), []
    run = asyncio.create_task(
        _collect(move_forward(_supervised_context(states, signal, _FAST)), requests)
    )
    await _paused_on_io_once(states, signal, requests)

    signal.set(pausing=False)
    await _starts_become(requests, 2)
    # The controller ignores the resume: only the old pause is re-published.
    await _republish_until(states, paused_on_io(1.0), lambda: _start_count(requests) >= 3)
    assert "sending the start once more" in caplog.text

    # The second start is taken up.
    states.feed(running(1.5), ended(3.0), ended(3.0), ended(3.0))
    async with asyncio.timeout(5):
        await run
    assert _start_count(requests) == 3


@pytest.mark.asyncio
async def test_no_start_is_sent_after_the_window_without_a_new_edge(caplog):
    states, signal, requests = _FedStates(), _Signal(), []
    run = asyncio.create_task(
        _collect(move_forward(_supervised_context(states, signal, _FAST)), requests)
    )
    await _paused_on_io_once(states, signal, requests)

    signal.set(pausing=False)
    await _starts_become(requests, 2)
    await _republish_until(
        states, paused_on_io(1.0), lambda: "release it again to continue" in caplog.text
    )
    assert _start_count(requests) == 3, "one extra start, then none"

    # Still ignored, time passes: nothing is sent while the signal stays released.
    for _ in range(20):
        states.feed(paused_on_io(1.0))
        await asyncio.sleep(0.01)
    assert _start_count(requests) == 3
    assert not run.done()

    # A new edge: the operator sets the signal to pause and releases it again.
    signal.set(pausing=True)
    await asyncio.sleep(0.01)
    signal.set(pausing=False)
    await _starts_become(requests, 4)
    states.feed(paused_on_io(1.0), running(1.5), ended(3.0), ended(3.0), ended(3.0))
    async with asyncio.timeout(5):
        await run


@pytest.mark.asyncio
async def test_a_condition_that_holds_again_is_an_ordinary_pause(caplog):
    states, signal, requests = _FedStates(), _Signal(), []
    run = asyncio.create_task(
        _collect(move_forward(_supervised_context(states, signal, _FAST)), requests)
    )
    await _paused_on_io_once(states, signal, requests)

    signal.set(pausing=False)
    await _starts_become(requests, 2)
    signal.set(pausing=True)  # the condition is back before the controller moved
    await _republish_until(
        states, paused_on_io(1.0), lambda: "pause condition holds again" in caplog.text
    )
    assert _start_count(requests) == 2, "no extra start while the signal pauses"

    signal.set(pausing=False)
    await _starts_become(requests, 3)
    states.feed(paused_on_io(1.0), running(1.5), ended(3.0), ended(3.0), ended(3.0))
    async with asyncio.timeout(5):
        await run


@pytest.mark.asyncio
async def test_an_ignored_resume_fails_in_strict_mode():
    states, signal, requests = _FedStates(), _Signal(), []
    strict = ExecutionPolicy(
        strict=True,
        standstill=StandstillConfig.passthrough(),
        resume_detect_s=0.05,
        resume_window_s=0.3,
    )
    run = asyncio.create_task(
        _collect(move_forward(_supervised_context(states, signal, strict)), requests)
    )
    await _paused_on_io_once(states, signal, requests)

    signal.set(pausing=False)
    await _starts_become(requests, 2)
    feeder = asyncio.create_task(_republish_until(states, paused_on_io(1.0), run.done))

    with pytest.raises(ResumeNotTakenUp):
        async with asyncio.timeout(5):
            await run
    await feeder
    assert _start_count(requests) == 2


@pytest.mark.asyncio
async def test_a_pause_superseding_a_pending_start_is_concluded_by_the_parked_frame():
    """forward() then pause() before any command went out: the machine, still
    pending, must expect the pause — the parked frame concludes it."""
    frames = _Frames()
    cursor = TrajectoryCursor(
        motion_id="traj-1",
        motion_group_state_stream=frames.stream(),
        joint_trajectory=_joint_trajectory(),
        emit_motion_events=False,
        policy=ExecutionPolicy.robust(),
    )
    cursor.forward()
    pause = cursor.pause()
    frames.feed(_state(True), parked(), parked(), parked())
    requests: list = []

    async def consume():
        async for request in cursor.cntrl(_responses()):
            requests.append(request)

    consumer = asyncio.create_task(consume())
    try:
        async with asyncio.timeout(5):
            result = await pause
        assert result.final_location == 0.0
        assert not any(isinstance(r, api.models.StartMovementRequest) for r in requests)
    finally:
        cursor.detach()
        async with asyncio.timeout(5):
            await consumer


class TestRailAtRest:
    """Replays the second failure (combined_loop, 2026-09-30, >85 % playback): the rail's
    encoder noise at rest was larger than a 1e-3 threshold meant in rad, so a flicker
    counted as corroborated motion, the parked frames concluded paused(USER) and the
    rail's operation completed at location 0.0 before the rail had moved."""

    _NOISY_PARKED_RAIL = (
        _rail(api.models.TrajectoryPausedByUser(), 0.0, -1500.000),
        _rail(api.models.TrajectoryPausedByUser(), 0.0, -1500.004, standstill=False),
        _rail(api.models.TrajectoryPausedByUser(), 0.0, -1500.001),
        _rail(api.models.TrajectoryPausedByUser(), 0.0, -1500.003),
        _rail(api.models.TrajectoryPausedByUser(), 0.0, -1500.002),
    )

    def test_noise_on_a_prismatic_joint_is_no_motion(self):
        machine = TrajectoryExecutionMachine(
            StandstillConfig.robust(), strict=False, prismatic_joints=(True,)
        )
        machine.arm()

        _feed(machine, *self._NOISY_PARKED_RAIL)

        assert machine.is_armed
        assert machine.last_reading is not None and machine.last_reading.at_rest
        _feed(machine, _rail(api.models.TrajectoryRunning(time_to_end=1000), 0.0, -1500.002))
        assert machine.is_executing

    def test_a_rad_threshold_on_the_rail_would_see_motion_but_no_longer_pauses(self):
        """The old unit mistake, forced: joint evidence fires on the noise, yet without
        commanded progress the parked frames still conclude nothing."""
        machine = TrajectoryExecutionMachine(
            StandstillConfig.robust(), strict=False, prismatic_joints=(False,)
        )
        machine.arm()

        _feed(machine, *self._NOISY_PARKED_RAIL)

        assert machine.is_armed
