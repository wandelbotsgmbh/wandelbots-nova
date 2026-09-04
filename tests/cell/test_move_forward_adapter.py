"""Regression tests for move_forward as a TrajectoryCursor adapter.

Since the movement-controller merge
(docs/architecture/adr/001-merge-movement-controllers-into-trajectory-cursor.md),
``move_forward`` no longer implements the ``executeTrajectory`` protocol itself:
it configures a :class:`TrajectoryCursor` for one-shot execution and starts it.
The behavioural contract of the old implementation is pinned by the untouched
``move_forward`` test files; these tests pin what is new at the adapter seam:

- the start command must reach the wire even against a state stream that runs
  to its end in a single scheduling slice (the first-dispatch gate);
- the context's IO overlay and IO gates must arrive on the emitted start.
"""

import asyncio
from datetime import datetime, timezone

import pytest

from nova import api
from nova.actions.container import CombinedActions, MovementControllerContext
from nova.actions.io import io_write
from nova.actions.motions import lin
from nova.cell.movement_controller.move_forward import move_forward
from nova.types import Pose

pytestmark = pytest.mark.asyncio


def _state(
    standstill: bool, execute: api.models.Execute | None = None
) -> api.models.MotionGroupState:
    return api.models.MotionGroupState(
        timestamp=datetime.now(timezone.utc),
        sequence_number=1,
        motion_group="mg-0",
        controller="ctrl-0",
        joint_position=[0.0] * 6,
        joint_limit_reached=api.models.MotionGroupStateJointLimitReached(limit_reached=[False] * 6),
        standstill=standstill,
        execute=execute,
        description_revision=1,
    )


def _execute(location: float, state=None) -> api.models.Execute:
    return api.models.Execute(
        joint_position=[0.0] * 6,
        details=api.models.TrajectoryDetails(
            trajectory="traj-1",
            location=location,
            state=state or api.models.TrajectoryRunning(time_to_end=0),
        ),
    )


def _finite_states():
    """A state stream that ends, as the movement-controller unit fixtures do.

    Without the cursor's first-dispatch gate this stream is consumed to its end
    in one scheduling slice, tearing the cursor down before the adapter's queued
    start had a turn — the measured D12/D16 artefact from the transition plan.
    """

    async def gen():
        yield _state(False, _execute(0.5))
        yield _state(True, _execute(1.0, api.models.TrajectoryEnded()))
        yield _state(True)

    return gen


async def _responses():
    yield api.models.InitializeMovementResponse()
    yield api.models.StartMovementResponse()
    await asyncio.Future()


def _context(**overrides) -> MovementControllerContext:
    defaults = dict(
        combined_actions=CombinedActions(items=()),
        motion_id="test-motion",
        motion_group_state_stream_gen=_finite_states(),
    )
    defaults.update(overrides)
    return MovementControllerContext(**defaults)


async def _run(context: MovementControllerContext) -> list:
    controller_fn = move_forward(context)
    requests = []
    async with asyncio.timeout(5):
        async for request in controller_fn(_responses()):
            requests.append(request)
    return requests


async def test_start_reaches_the_wire_against_a_fast_finite_stream():
    """An empty-actions context (the preplanned path) runs to completion and
    the start command is sent even though the mocked state stream can be
    consumed to its end before the request loop's first turn."""
    requests = await _run(_context())

    assert any(isinstance(r, api.models.InitializeMovementRequest) for r in requests)
    assert any(isinstance(r, api.models.StartMovementRequest) for r in requests)


async def test_io_only_actions_with_a_preplanned_trajectory():
    """Attaching an IO-only overlay to a preplanned trajectory must keep working.

    Regression (PR #475 review): the cursor normalised ``actions=[]`` to "no
    action metadata" but not a non-empty list without motion actions, so this
    previously-supported ``execute()`` shape raised ``ValueError`` against the
    trajectory's real end location.
    """
    combined_actions = CombinedActions(items=(io_write(key="OUT#900", value=True),))
    joint_trajectory = api.models.JointTrajectory(
        joint_positions=[[0.0] * 6, [0.1] * 6], times=[0.0, 1.0], locations=[0.0, 1.0]
    )
    context = _context(combined_actions=combined_actions, joint_trajectory=joint_trajectory)

    requests = await _run(context)

    starts = [r for r in requests if isinstance(r, api.models.StartMovementRequest)]
    assert len(starts) == 1
    assert starts[0].set_outputs == combined_actions.to_set_io()
    assert starts[0].set_outputs, "the IO overlay must still travel on the start"


async def test_start_carries_the_io_overlay_and_io_gates():
    """set_outputs derived from the actions, and the context's start/pause IO
    conditions, must all travel on the emitted StartMovementRequest."""
    combined_actions = CombinedActions(
        items=(lin(Pose((100.0, 0, 0, 0, 0, 0))), io_write(key="OUT#900", value=True))
    )
    start_on_io = api.models.StartOnIO(
        io=api.models.IOBooleanValue(io="IN#1", value=True),
        comparator=api.models.Comparator.COMPARATOR_EQUALS,
        io_origin=api.models.IOOrigin.CONTROLLER,
    )
    pause_on_io = api.models.PauseOnIO(
        io=api.models.IOBooleanValue(io="IN#2", value=True),
        comparator=api.models.Comparator.COMPARATOR_EQUALS,
        io_origin=api.models.IOOrigin.CONTROLLER,
    )
    context = _context(
        combined_actions=combined_actions, start_on_io=start_on_io, pause_on_io=pause_on_io
    )

    requests = await _run(context)

    starts = [r for r in requests if isinstance(r, api.models.StartMovementRequest)]
    assert len(starts) == 1
    expected_outputs = combined_actions.to_set_io()
    assert expected_outputs, "fixture must produce a non-empty IO overlay"
    assert starts[0].set_outputs == expected_outputs
    assert starts[0].start_on_io == start_on_io
    assert starts[0].pause_on_io == pause_on_io


async def test_context_set_outputs_override_the_derived_overlay():
    """A context that resolved path triggers carries the finished ``set_outputs``;
    the adapter must forward it instead of re-deriving the overlay from the actions."""
    combined_actions = CombinedActions(
        items=(lin(Pose((100.0, 0, 0, 0, 0, 0))), io_write(key="OUT#900", value=True))
    )
    resolved = [
        api.models.SetIO(
            io=api.models.IOBooleanValue(io="OUT#900", value=True),
            location=0.5,
            io_origin=api.models.IOOrigin.CONTROLLER,
        )
    ]
    context = _context(combined_actions=combined_actions, set_outputs=resolved)

    requests = await _run(context)

    starts = [r for r in requests if isinstance(r, api.models.StartMovementRequest)]
    assert len(starts) == 1
    assert starts[0].set_outputs == resolved
    assert starts[0].set_outputs != combined_actions.to_set_io()


async def test_empty_context_set_outputs_is_not_treated_as_missing():
    """An explicitly empty resolved overlay must not fall back to ``to_set_io()``."""
    combined_actions = CombinedActions(
        items=(lin(Pose((100.0, 0, 0, 0, 0, 0))), io_write(key="OUT#900", value=True))
    )
    context = _context(combined_actions=combined_actions, set_outputs=[])

    requests = await _run(context)

    starts = [r for r in requests if isinstance(r, api.models.StartMovementRequest)]
    assert starts[0].set_outputs == []


# ---------------------------------------------------------------------------
# Controller-side IO pause: execute() must block through it and finish
# ---------------------------------------------------------------------------


def _paused_on_io(location: float) -> api.models.Execute:
    return _execute(location, api.models.TrajectoryPausedOnIO())


class _FedStates:
    """A state stream fed by the test; ends when ``close()`` is called."""

    def __init__(self):
        self._queue: asyncio.Queue = asyncio.Queue()

    def feed(self, *states):
        for state in states:
            self._queue.put_nowait(state)

    def close(self):
        self._queue.put_nowait(None)

    def gen(self):
        async def _gen():
            while (state := await self._queue.get()) is not None:
                yield state

        return _gen


def _pause_condition() -> api.models.PauseOnIO:
    return api.models.PauseOnIO(
        io=api.models.IOBooleanValue(io="hold", value=True),
        comparator=api.models.Comparator.COMPARATOR_EQUALS,
        io_origin=api.models.IOOrigin.BUS_IO,
    )


async def _collect(controller_fn, requests: list) -> None:
    async for request in controller_fn(_responses()):
        requests.append(request)


async def _wait_for(predicate, timeout: float = 5.0) -> None:
    async with asyncio.timeout(timeout):
        while not predicate():
            await asyncio.sleep(0)


async def test_io_pause_is_resumed_once_the_signal_clears():
    """The controller holds an IO pause until a new start arrives after the
    condition cleared (measured). move_forward waits for the release and starts
    again, so the one-shot execution completes at the target — with the pause
    condition and the IO overlay re-attached to the resume."""
    states = _FedStates()
    release_requested = asyncio.Event()
    release = asyncio.Event()

    async def wait_for_release():
        release_requested.set()
        await release.wait()

    context = _context(
        motion_group_state_stream_gen=states.gen(),
        pause_on_io=_pause_condition(),
        set_outputs=[
            api.models.SetIO(
                io=api.models.IOBooleanValue(io="OUT#1", value=True),
                location=1.5,
                io_origin=api.models.IOOrigin.CONTROLLER,
            )
        ],
        wait_for_pause_on_io_release=wait_for_release,
    )
    requests: list = []
    run = asyncio.create_task(_collect(move_forward(context), requests))

    states.feed(_state(True), _state(False, _execute(0.5)), _state(True, _paused_on_io(1.0)))
    await _wait_for(release_requested.is_set)
    assert not run.done(), "execute() must not end on a controller-side IO pause"
    starts = [r for r in requests if isinstance(r, api.models.StartMovementRequest)]
    assert len(starts) == 1

    release.set()
    await _wait_for(
        lambda: sum(isinstance(r, api.models.StartMovementRequest) for r in requests) == 2
    )
    # Stale pause frames right after the resume, then the motion, then the end.
    states.feed(
        _state(True, _paused_on_io(1.0)),
        _state(False, _execute(1.5)),
        _state(True, _execute(2.0, api.models.TrajectoryEnded())),
        _state(True),
    )
    async with asyncio.timeout(5):
        await run

    starts = [r for r in requests if isinstance(r, api.models.StartMovementRequest)]
    assert len(starts) == 2
    assert all(s.pause_on_io == _pause_condition() for s in starts)
    assert all(s.set_outputs == context.set_outputs for s in starts)


async def test_io_pause_without_a_release_waiter_ends_the_execution_early(caplog):
    """A hand-built context has no way to observe the signal; the adapter then
    keeps the previous contract (end early) rather than hanging forever."""
    states = _FedStates()
    context = _context(motion_group_state_stream_gen=states.gen(), pause_on_io=_pause_condition())
    requests: list = []
    run = asyncio.create_task(_collect(move_forward(context), requests))

    states.feed(_state(True), _state(False, _execute(0.5)), _state(True, _paused_on_io(1.0)))
    async with asyncio.timeout(5):
        await run

    starts = [r for r in requests if isinstance(r, api.models.StartMovementRequest)]
    assert len(starts) == 1
    assert "no release waiter" in caplog.text


# ---------------------------------------------------------------------------
# Loss of the signal source: the SDK pauses in the controller's place
# ---------------------------------------------------------------------------


def _paused_by_user(location: float) -> api.models.Execute:
    return _execute(location, api.models.TrajectoryPausedByUser())


async def test_signal_source_loss_pauses_from_the_sdk_and_resumes_after_release():
    """The controller stops evaluating a bus-IO condition when the bus is gone and
    keeps moving (measured). The adapter must then send a PauseMovementRequest itself
    and, once the release waiter returns (bus back, signal allows), start again."""
    states = _FedStates()
    loss = asyncio.Event()
    release_requested = asyncio.Event()
    release = asyncio.Event()

    async def wait_for_signal_loss():
        # Like the real waiter after the bus came back: one loss event, then it
        # waits for the next one rather than reporting the same loss again.
        await loss.wait()
        loss.clear()

    async def wait_for_release():
        release_requested.set()
        await release.wait()

    context = _context(
        motion_group_state_stream_gen=states.gen(),
        pause_on_io=_pause_condition(),
        wait_for_pause_on_io_release=wait_for_release,
        wait_for_pause_signal_loss=wait_for_signal_loss,
    )
    requests: list = []
    run = asyncio.create_task(_collect(move_forward(context), requests))

    states.feed(_state(True), _state(False, _execute(0.5)))
    await _wait_for(
        lambda: sum(isinstance(r, api.models.StartMovementRequest) for r in requests) == 1
    )
    loss.set()  # the bus-IO service went away while the robot is moving
    await _wait_for(lambda: any(isinstance(r, api.models.PauseMovementRequest) for r in requests))
    # The controller pauses on our request and holds the level-based pause state.
    states.feed(_state(False, _paused_by_user(0.8)), _state(True, _paused_by_user(0.8)))
    await _wait_for(release_requested.is_set)
    assert not run.done(), "execute() must not end while waiting for the signal to return"

    release.set()  # bus back and the signal allows motion again
    await _wait_for(
        lambda: sum(isinstance(r, api.models.StartMovementRequest) for r in requests) == 2
    )
    states.feed(
        _state(True, _paused_by_user(0.8)),  # stale re-publish
        _state(False, _execute(1.5)),
        _state(True, _execute(2.0, api.models.TrajectoryEnded())),
        _state(True),
    )
    async with asyncio.timeout(5):
        await run

    kinds = [type(r).__name__ for r in requests]
    assert kinds.count("StartMovementRequest") == 2
    assert kinds.count("PauseMovementRequest") == 1
    assert kinds.index("PauseMovementRequest") < len(kinds) - 1


async def test_a_failing_release_waiter_ends_the_execution_with_its_error():
    """No polling fallback: when the signal source cannot be observed the wait fails,
    and execute() must surface that instead of pausing forever."""
    states = _FedStates()

    async def wait_for_release():
        raise RuntimeError("bus IO conditions need a connected NATS client")

    context = _context(
        motion_group_state_stream_gen=states.gen(),
        pause_on_io=_pause_condition(),
        wait_for_pause_on_io_release=wait_for_release,
    )
    requests: list = []
    run = asyncio.create_task(_collect(move_forward(context), requests))
    states.feed(_state(True), _state(False, _execute(0.5)), _state(True, _paused_on_io(1.0)))

    with pytest.raises(RuntimeError, match="NATS"):
        async with asyncio.timeout(5):
            await run
