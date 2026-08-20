"""Tests for the synchronized session, driven over a faked executeTrajectory protocol.

The barrier semantics are the spec proven in production by the SHL backend: the
sync trigger is flipped true only once *every* group has armed its start-on-IO
condition (reported ``TrajectoryWaitForIO``), a repeated start re-runs the
barrier and must not be satisfied by a stale wait-for-IO state, and a location
spread beyond ``max_drift`` aborts the whole session.

Every test here needs a request/response loop: what one group's cursor is told
depends on what the others have already reported, which a recorded sequence of
responses cannot express. ``_FakeGateway`` closes that loop. It is the price of
covering the barrier in unit tests, and the reason it is confined to this file —
``tests/nova/cell/test_trajectory_executor_integration.py`` covers the same
paths against a live cell.
"""

import asyncio
from collections.abc import AsyncIterator
from unittest.mock import MagicMock

import pytest

from nova import api
from nova.cell.session_monitor import SyncDriftError
from nova.cell.trajectory_executor import GroupArgs, TrajectoryExecutor
from tests.cell.multi_group_doubles import (
    IOGateway,
    ended_state,
    motion_group,
    multi_trajectory,
    paused_state,
    running_state,
    sync_driver,
    wait_for_io_state,
)

pytestmark = pytest.mark.asyncio


class _FakeGateway(IOGateway):
    """The IO surface plus the bidirectional ``executeTrajectory`` protocol with
    FIFO acks."""

    def __init__(self):
        super().__init__()
        self.start_requests: dict[str, list[api.models.StartMovementRequest]] = {}
        self.initialize_requests: dict[str, api.models.InitializeMovementRequest] = {}
        self.execution_cells: dict[str, str] = {}
        # Milestone counters the tests await, so they never depend on how many
        # event-loop ticks a barrier happens to take.
        self._counts = dict.fromkeys(("init", "start", "pause", "write"), 0)
        self._changed = {milestone: asyncio.Event() for milestone in self._counts}

        self.trajectory_execution_api = MagicMock()
        self.trajectory_execution_api.execute_trajectory = self._execute_trajectory

    async def _record_write(self, cell, controller, io_value):
        await super()._record_write(cell, controller, io_value)
        self._reach("write")

    def _reach(self, milestone: str) -> None:
        self._counts[milestone] += 1
        self._changed[milestone].set()

    async def reached(self, milestone: str, count: int = 1) -> None:
        """Wait until ``count`` of a milestone have happened in total."""
        while self._counts[milestone] < count:
            self._changed[milestone].clear()
            await asyncio.wait_for(self._changed[milestone].wait(), timeout=1)

    async def _execute_trajectory(self, cell, controller, client_request_generator):
        responses: asyncio.Queue = asyncio.Queue()

        async def response_stream() -> AsyncIterator[api.models.ExecuteTrajectoryResponse]:
            while True:
                yield await responses.get()

        trajectory_id = ""
        async for request in client_request_generator(response_stream()):
            if isinstance(request, api.models.InitializeMovementRequest):
                trajectory_id = request.trajectory.id
                self.execution_cells[trajectory_id] = cell
                self.initialize_requests[trajectory_id] = request
                self._reach("init")
                responses.put_nowait(
                    api.models.ExecuteTrajectoryResponse(
                        root=api.models.InitializeMovementResponse()
                    )
                )
            elif isinstance(request, api.models.StartMovementRequest):
                self.start_requests.setdefault(trajectory_id, []).append(request)
                self._reach("start")
                responses.put_nowait(
                    api.models.ExecuteTrajectoryResponse(root=api.models.StartMovementResponse())
                )
            elif isinstance(request, api.models.PauseMovementRequest):
                self._reach("pause")
                responses.put_nowait(
                    api.models.ExecuteTrajectoryResponse(root=api.models.PauseMovementResponse())
                )


async def _settle() -> None:
    """Let everything that can run, run — for asserting that something did *not* happen."""
    for _ in range(30):
        await asyncio.sleep(0)


def _two_group_executor(
    gateway: _FakeGateway,
) -> tuple[TrajectoryExecutor, dict[str, asyncio.Queue]]:
    state_queues = {"a": asyncio.Queue(), "b": asyncio.Queue()}
    executor = (
        TrajectoryExecutor.builder(
            {
                "a": motion_group(gateway, state_queues["a"], trajectory_id="traj-a"),
                "b": motion_group(gateway, state_queues["b"], trajectory_id="traj-b"),
            }
        )
        .sync_on_io("sync-io")
        .build()
    )
    return executor, state_queues


class TestTrajectorySplitting:
    async def test_attach__loads_each_group_with_the_shared_parameterization(self):
        gateway = _FakeGateway()
        motion_groups = {
            name: motion_group(gateway, trajectory_id=f"traj-{name}") for name in ("a", "b")
        }
        executor = TrajectoryExecutor(motion_groups, sync=sync_driver(gateway))

        async with executor.attach(multi_trajectory("a", "b")):
            pass

        for group in motion_groups.values():
            loaded_trajectory, tcp = group._load_planned_motion.await_args.args
            assert loaded_trajectory.times == [0.0, 1.0, 2.0]
            assert [location.root for location in loaded_trajectory.locations] == [0.0, 1.0, 2.0]
            assert tcp is None


class TestExecutorAddressing:
    async def test_attach__executes_each_group_through_its_own_cell(self):
        gateway = _FakeGateway()
        motion_groups = {
            "a": motion_group(gateway, trajectory_id="traj-a", cell="cell-a"),
            "b": motion_group(gateway, trajectory_id="traj-b", cell="cell-b"),
        }
        executor = TrajectoryExecutor(motion_groups, sync=sync_driver(gateway))

        async with executor.attach(multi_trajectory("a", "b")):
            await gateway.reached("init", 2)

        assert gateway.execution_cells == {"traj-a": "cell-a", "traj-b": "cell-b"}


class TestSessionMonitors:
    async def test_monitors__called_empty__runs_without_the_default_drift_monitor(self):
        # Arrange: the same drift that aborts a default session (see
        # TestSynchronizedExecution) must be tolerated once monitors are dropped
        gateway = _FakeGateway()
        state_queues = {"a": asyncio.Queue(), "b": asyncio.Queue()}
        executor = (
            TrajectoryExecutor.builder(
                {
                    "a": motion_group(gateway, state_queues["a"], trajectory_id="traj-a"),
                    "b": motion_group(gateway, state_queues["b"], trajectory_id="traj-b"),
                }
            )
            .sync_on_io("sync-io")
            .monitors()
            .build()
        )

        # Act
        execute_task = asyncio.create_task(executor.execute(multi_trajectory("a", "b")))
        await gateway.reached("start", 2)
        state_queues["a"].put_nowait(wait_for_io_state(at_milliseconds=10))
        state_queues["b"].put_nowait(wait_for_io_state(at_milliseconds=20))
        await gateway.reached("write", 2)
        state_queues["a"].put_nowait(running_state(0.5, at_milliseconds=30))
        state_queues["b"].put_nowait(running_state(1.5, at_milliseconds=30))
        state_queues["a"].put_nowait(ended_state(2.0, at_milliseconds=40))
        state_queues["b"].put_nowait(ended_state(2.0, at_milliseconds=50))

        # Assert
        await asyncio.wait_for(execute_task, timeout=5)


class TestSynchronizedExecution:
    async def test_execute__flips_trigger_true_only_after_all_groups_wait_for_io(self):
        # Arrange
        gateway = _FakeGateway()
        executor, state_queues = _two_group_executor(gateway)

        # Act: both cursors get armed with the start condition
        execute_task = asyncio.create_task(executor.execute(multi_trajectory("a", "b")))
        await gateway.reached("start", 2)

        # Assert: armed with the trigger condition, barrier not yet released
        start_request = gateway.start_requests["traj-a"][0]
        assert start_request.start_on_io is not None
        assert start_request.start_on_io.io.root.io == "sync-io"
        assert gateway.trigger_writes == [False]

        # Act: running states do not release the barrier
        state_queues["a"].put_nowait(running_state(0.0, at_milliseconds=10))
        state_queues["b"].put_nowait(running_state(0.0, at_milliseconds=10))
        await _settle()

        # Assert
        assert gateway.trigger_writes == [False]

        # Act: one group waiting for IO is not enough
        state_queues["a"].put_nowait(wait_for_io_state(at_milliseconds=20))
        await _settle()

        # Assert
        assert gateway.trigger_writes == [False]

        # Act: all groups waiting for IO releases the barrier
        state_queues["b"].put_nowait(wait_for_io_state(at_milliseconds=30))
        await gateway.reached("write", 2)

        # Act: both trajectories run to their end
        for milliseconds, queue in ((40, state_queues["a"]), (50, state_queues["b"])):
            queue.put_nowait(running_state(1.0, at_milliseconds=milliseconds))
        state_queues["a"].put_nowait(ended_state(2.0, at_milliseconds=60))
        state_queues["b"].put_nowait(ended_state(2.0, at_milliseconds=70))
        await asyncio.wait_for(execute_task, timeout=5)

        # Assert: one release; the controller's own limit check is skipped by
        # default — it would rescale each trajectory individually and silently
        # break the shared parameterization
        assert gateway.trigger_writes == [False, True]
        assert gateway.initialize_requests["traj-a"].ignore_controller_limits is True
        assert gateway.initialize_requests["traj-b"].ignore_controller_limits is True

    async def test_attach__forward_after_pause__requires_fresh_wait_for_io(self):
        # Arrange
        gateway = _FakeGateway()
        executor, state_queues = _two_group_executor(gateway)

        async with executor.attach(multi_trajectory("a", "b")) as cursor:
            operation = cursor.forward()
            await gateway.reached("start", 2)
            state_queues["a"].put_nowait(wait_for_io_state(at_milliseconds=10))
            state_queues["b"].put_nowait(wait_for_io_state(at_milliseconds=20))
            await gateway.reached("write", 2)
            state_queues["a"].put_nowait(running_state(0.5, at_milliseconds=30))
            state_queues["b"].put_nowait(running_state(0.5, at_milliseconds=40))

            # Act: pause and resume
            cursor.pause()
            await gateway.reached("pause", 2)
            state_queues["a"].put_nowait(paused_state(0.5, at_milliseconds=50))
            state_queues["b"].put_nowait(paused_state(0.5, at_milliseconds=60))
            resume = cursor.forward()
            await gateway.reached("start", 4)

            # Assert: the barrier was re-armed and is not satisfied by the earlier
            # wait-for-IO states
            await _settle()
            assert gateway.trigger_writes == [False, True, False]

            # Act: fresh wait-for-IO states release the second barrier
            state_queues["a"].put_nowait(wait_for_io_state(at_milliseconds=70))
            state_queues["b"].put_nowait(wait_for_io_state(at_milliseconds=80))
            await gateway.reached("write", 4)

            state_queues["a"].put_nowait(ended_state(2.0, at_milliseconds=90))
            state_queues["b"].put_nowait(ended_state(2.0, at_milliseconds=100))
            await asyncio.wait_for(resume, timeout=5)
            with pytest.raises(asyncio.CancelledError):
                await operation

    async def test_execute__drift_beyond_max_drift__aborts_with_sync_drift_error(self):
        # Arrange
        gateway = _FakeGateway()
        executor, state_queues = _two_group_executor(gateway)

        # Act
        execute_task = asyncio.create_task(executor.execute(multi_trajectory("a", "b")))
        await gateway.reached("start", 2)
        state_queues["a"].put_nowait(wait_for_io_state(at_milliseconds=10))
        state_queues["b"].put_nowait(wait_for_io_state(at_milliseconds=20))
        await gateway.reached("write")
        state_queues["a"].put_nowait(running_state(0.5, at_milliseconds=30))
        state_queues["b"].put_nowait(running_state(1.5, at_milliseconds=30))

        # Assert
        with pytest.raises(SyncDriftError):
            await asyncio.wait_for(execute_task, timeout=5)

    async def test_execute__single_group__runs_the_barrier_and_applies_group_args(self):
        # Arrange
        gateway = _FakeGateway()
        states: asyncio.Queue = asyncio.Queue()
        executor = TrajectoryExecutor(
            motion_groups={"a": motion_group(gateway, states, trajectory_id="traj-a")},
            sync=sync_driver(gateway, groups=("a",)),
        )

        # Act
        execute_task = asyncio.create_task(
            executor.execute(
                multi_trajectory("a"), groups={"a": GroupArgs(ignore_controller_limits=False)}
            )
        )
        await gateway.reached("start")
        states.put_nowait(wait_for_io_state(at_milliseconds=10))
        await gateway.reached("write")
        states.put_nowait(ended_state(2.0, at_milliseconds=20))
        await asyncio.wait_for(execute_task, timeout=5)

        # Assert: the barrier ran for the single group and the per-group
        # ignore_controller_limits override reached the initialization
        assert gateway.start_requests["traj-a"][0].start_on_io is not None
        assert gateway.initialize_requests["traj-a"].ignore_controller_limits is False

    async def test_attach__pause_while_barrier_is_waiting__exits_cleanly(self):
        # Arrange
        gateway = _FakeGateway()
        executor, _ = _two_group_executor(gateway)

        # Act: pause before any group reported wait-for-IO, then leave the session
        async def run() -> asyncio.Future:
            async with executor.attach(multi_trajectory("a", "b")) as cursor:
                operation = cursor.forward()
                await gateway.reached("start", 2)
                cursor.pause()
                return operation

        operation = await asyncio.wait_for(run(), timeout=5)

        # Assert: the stranded barrier was cancelled with the operation, the
        # trigger was never released
        assert operation.cancelled()
        assert gateway.trigger_writes == [False]

    async def test_attach__forward_superseding_a_waiting_barrier__releases_the_trigger_once(self):
        # Arrange
        gateway = _FakeGateway()
        executor, state_queues = _two_group_executor(gateway)

        async with executor.attach(multi_trajectory("a", "b")) as cursor:
            cursor.forward()
            await gateway.reached("start", 2)
            cursor.pause()

            # Act: a new forward supersedes the stranded barrier
            resume = cursor.forward()
            await gateway.reached("start", 4)
            state_queues["a"].put_nowait(wait_for_io_state(at_milliseconds=10))
            state_queues["b"].put_nowait(wait_for_io_state(at_milliseconds=20))
            await gateway.reached("write")
            await _settle()

            # Assert: only the second barrier released the trigger
            assert gateway.trigger_writes == [False, False, True]

            state_queues["a"].put_nowait(ended_state(2.0, at_milliseconds=30))
            state_queues["b"].put_nowait(ended_state(2.0, at_milliseconds=40))
            await asyncio.wait_for(resume, timeout=5)

    async def test_attach__forward_after_the_trajectory_ended__fails_instead_of_hanging(self):
        # Arrange
        gateway = _FakeGateway()
        executor, state_queues = _two_group_executor(gateway)

        async def run() -> None:
            async with executor.attach(multi_trajectory("a", "b")) as cursor:
                operation = cursor.forward()
                await gateway.reached("start", 2)
                state_queues["a"].put_nowait(wait_for_io_state(at_milliseconds=10))
                state_queues["b"].put_nowait(wait_for_io_state(at_milliseconds=20))
                state_queues["a"].put_nowait(ended_state(2.0, at_milliseconds=30))
                state_queues["b"].put_nowait(ended_state(2.0, at_milliseconds=40))
                results = await operation
                assert set(results) == {"a", "b"}

                # Act: the session has run to its end and detached its cursors
                await cursor.forward()

        # Assert: the late forward fails loudly instead of deadlocking the exit
        with pytest.raises(RuntimeError):
            await asyncio.wait_for(run(), timeout=5)
