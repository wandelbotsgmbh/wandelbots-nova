"""Regression tests for move_forward as a TrajectoryCursor adapter.

Since the Phase C merge (docs/architecture/incoming/move-forward-to-cursor.md),
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
        joint_position=api.models.Joints(root=[0.0] * 6),
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
            location=api.models.Location(root=location),
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
    yield api.models.ExecuteTrajectoryResponse(root=api.models.InitializeMovementResponse())
    yield api.models.ExecuteTrajectoryResponse(root=api.models.StartMovementResponse())
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
