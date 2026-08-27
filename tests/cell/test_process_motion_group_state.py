"""Tests for move_forward state monitor logic.

These tests verify that trajectory completion is only detected when both
TrajectoryEnded (or TrajectoryPausedByUser) AND standstill are True.
"""

import asyncio
from datetime import datetime, timezone

import pytest

from nova import api
from nova.actions.container import CombinedActions, MovementControllerContext
from nova.cell.movement_controller.move_forward import move_forward
from nova.exceptions import ErrorDuringMovement, InitMovementFailed

# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------


def _make_motion_group_state(
    standstill: bool, execute: api.models.Execute | None = None
) -> api.models.MotionGroupState:
    """Create a MotionGroupState with the given standstill and execute fields."""
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


def _make_execute(
    trajectory_state: api.models.TrajectoryRunning
    | api.models.TrajectoryEnded
    | api.models.TrajectoryPausedByUser,
    location: float = 1.0,
) -> api.models.Execute:
    """Create an Execute with TrajectoryDetails."""
    return api.models.Execute(
        joint_position=[0.0] * 6,
        details=api.models.TrajectoryDetails(
            trajectory="traj-123", location=location, state=trajectory_state
        ),
    )


async def _async_iter(items):
    """Turn a list into an async iterator."""
    for item in items:
        yield item


def _make_response(inner) -> api.models.ExecuteTrajectoryResponse:
    """Wrap an inner response model in ExecuteTrajectoryResponse."""
    return inner


def _make_context(
    state_sequence: list[api.models.MotionGroupState], motion_id: str = "test-motion"
) -> MovementControllerContext:
    """Create a MovementControllerContext that yields a fixed sequence of states."""
    return MovementControllerContext(
        combined_actions=CombinedActions(items=()),
        motion_id=motion_id,
        start_on_io=None,
        motion_group_state_stream_gen=lambda: _async_iter(state_sequence),
    )


# ---------------------------------------------------------------------------
# move_forward state monitor tests
# ---------------------------------------------------------------------------


class TestMoveForwardStateMonitor:
    """Tests for the move_forward movement controller's state monitoring logic."""

    @pytest.mark.asyncio
    async def test_completes_on_trajectory_ended_with_standstill(self):
        """State monitor completes when TrajectoryEnded AND standstill are both true."""
        states = [
            # Running, no standstill
            _make_motion_group_state(
                standstill=False,
                execute=_make_execute(api.models.TrajectoryRunning(time_to_end=1000)),
            ),
            # Ended + standstill → should complete
            _make_motion_group_state(
                standstill=True, execute=_make_execute(api.models.TrajectoryEnded())
            ),
            # Extra state with standstill to confirm completion
            _make_motion_group_state(standstill=True),
        ]
        context = _make_context(states)
        controller_fn = move_forward(context)

        # Build a response stream that provides init and start responses,
        # then blocks forever (simulating waiting for errors that never come)
        responses = [
            _make_response(api.models.InitializeMovementResponse()),
            _make_response(api.models.StartMovementResponse()),
        ]

        async def response_stream():
            for r in responses:
                yield r
            # Block forever — the error consumer hangs here
            await asyncio.Future()

        # The controller should complete without raising
        async with asyncio.timeout(5):
            requests = []
            async for req in controller_fn(response_stream()):
                requests.append(req)

        assert any(isinstance(r, api.models.InitializeMovementRequest) for r in requests)
        assert any(isinstance(r, api.models.StartMovementRequest) for r in requests)

    @pytest.mark.asyncio
    async def test_trajectory_ended_then_bare_standstill_completes(self):
        """TrajectoryEnded WITHOUT standstill arms completion; the next standstill
        frame concludes it — even a bare one without an ``execute`` block.

        Current controllers drop the ``execute`` block the instant the robot
        settles (robotics/wbr MotionPointGenerator; changes with wbr!2262), so
        the bare standstill after an observed TrajectoryEnded can be the only
        completion signal that ever arrives. The monitor previously discarded
        those frames and waited forever for a TrajectoryEnded+standstill pair
        that current controllers never publish.

        TrajectoryEnded alone (frame 2, still decelerating) must NOT complete —
        completion requires the subsequent standstill (frame 3).
        """
        consumed_count = 0

        async def counting_states():
            nonlocal consumed_count
            states = [
                # 1: Running, not at standstill
                _make_motion_group_state(
                    standstill=False,
                    execute=_make_execute(api.models.TrajectoryRunning(time_to_end=1000)),
                ),
                # 2: TrajectoryEnded WITHOUT standstill — arms completion, must
                #    not complete on its own (robot still decelerating)
                _make_motion_group_state(
                    standstill=False, execute=_make_execute(api.models.TrajectoryEnded())
                ),
                # 3: Bare standstill, no execute — completes (the controller has
                #    dropped the execute block at settle; this is the only
                #    completion signal current controllers deliver)
                _make_motion_group_state(standstill=True),
                # 4–5: never consumed; present so a regression back to
                #      pair-required completion is caught by consumed_count
                _make_motion_group_state(
                    standstill=True, execute=_make_execute(api.models.TrajectoryEnded())
                ),
                _make_motion_group_state(standstill=True),
            ]
            for s in states:
                consumed_count += 1
                yield s

        context = MovementControllerContext(
            combined_actions=CombinedActions(items=()),
            motion_id="test-motion",
            start_on_io=None,
            motion_group_state_stream_gen=counting_states,
        )
        controller_fn = move_forward(context)

        responses = [
            _make_response(api.models.InitializeMovementResponse()),
            _make_response(api.models.StartMovementResponse()),
        ]

        async def response_stream():
            for r in responses:
                yield r
            await asyncio.Future()

        async with asyncio.timeout(5):
            async for _ in controller_fn(response_stream()):
                pass

        # Frame 2 (TrajectoryEnded, still decelerating) must not complete on its
        # own; frame 3 (the first standstill after it, bare) must. Returning at
        # exactly 3 also proves the monitor no longer needs the
        # TrajectoryEnded+standstill pair of frames 4–5.
        assert consumed_count == 3, (
            f"Monitor returned after {consumed_count} states — expected completion "
            "on the first standstill frame after TrajectoryEnded (frame 3)"
        )

    @pytest.mark.asyncio
    async def test_completes_on_trajectory_ended_then_standstill_in_next_state(self):
        """Two-phase completion: TrajectoryEnded+standstill sets the flag,
        then a subsequent standstill state confirms completion.
        """
        states = [
            # Running
            _make_motion_group_state(
                standstill=False,
                execute=_make_execute(api.models.TrajectoryRunning(time_to_end=1000)),
            ),
            # TrajectoryEnded WITH standstill → sets trajectory_ended = True
            _make_motion_group_state(
                standstill=True, execute=_make_execute(api.models.TrajectoryEnded())
            ),
            # Subsequent state with standstill → triggers return
            _make_motion_group_state(standstill=True),
        ]
        context = _make_context(states)
        controller_fn = move_forward(context)

        responses = [
            _make_response(api.models.InitializeMovementResponse()),
            _make_response(api.models.StartMovementResponse()),
        ]

        async def response_stream():
            for r in responses:
                yield r
            await asyncio.Future()

        async with asyncio.timeout(5):
            async for _ in controller_fn(response_stream()):
                pass

    @pytest.mark.asyncio
    async def test_init_failure_raises(self):
        """move_forward raises InitMovementFailed when initialization reports an error."""
        context = _make_context(state_sequence=[])
        controller_fn = move_forward(context)

        init_resp = api.models.InitializeMovementResponse(message="trajectory not found")
        responses = [_make_response(init_resp)]

        with pytest.raises(InitMovementFailed):
            async for _ in controller_fn(_async_iter(responses)):
                pass

    @pytest.mark.asyncio
    async def test_error_during_movement_raises(self):
        """move_forward raises ErrorDuringMovement when the error consumer receives an error."""

        # State stream that runs forever without completing
        async def infinite_states():
            while True:
                yield _make_motion_group_state(
                    standstill=False,
                    execute=_make_execute(api.models.TrajectoryRunning(time_to_end=5000)),
                )
                await asyncio.sleep(0)

        context = MovementControllerContext(
            combined_actions=CombinedActions(items=()),
            motion_id="test-motion",
            start_on_io=None,
            motion_group_state_stream_gen=lambda: infinite_states(),
        )
        controller_fn = move_forward(context)

        error_resp = api.models.MovementErrorResponse(message="collision detected")
        responses = [
            _make_response(api.models.InitializeMovementResponse()),
            _make_response(api.models.StartMovementResponse()),
            _make_response(error_resp),
        ]

        with pytest.raises(ErrorDuringMovement, match="collision detected"):
            async with asyncio.timeout(5):
                async for _ in controller_fn(_async_iter(responses)):
                    pass

    @pytest.mark.asyncio
    async def test_skips_states_without_execute(self):
        """States without execute field are skipped, monitor continues to next state."""
        states = [
            # No execute — should be skipped
            _make_motion_group_state(standstill=True),
            _make_motion_group_state(standstill=False),
            # Now a proper completion sequence
            _make_motion_group_state(
                standstill=True, execute=_make_execute(api.models.TrajectoryEnded())
            ),
            _make_motion_group_state(standstill=True),
        ]
        context = _make_context(states)
        controller_fn = move_forward(context)

        responses = [
            _make_response(api.models.InitializeMovementResponse()),
            _make_response(api.models.StartMovementResponse()),
        ]

        async def response_stream():
            for r in responses:
                yield r
            await asyncio.Future()

        async with asyncio.timeout(5):
            async for _ in controller_fn(response_stream()):
                pass
