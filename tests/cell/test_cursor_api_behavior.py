"""Integration experiments to probe the NOVA execute-trajectory stream protocol.

These tests discover the server's actual behavior for scenarios that the
TrajectoryCursor must handle correctly:

1. Same-direction re-command with new target while moving
2. Same-direction duplicate forward while moving
3. Direction reversal (forward → backward) WITHOUT pause while moving
4. Direction reversal WITH pause (the "safe" sequence)
5. Rapid-fire: two forward commands before first response arrives

Each test bypasses the TrajectoryCursor and drives the bidirectional stream
directly so we can observe raw protocol behavior without client-side
abstractions masking anything.

Run with:
    PYTHONPATH=. uv run pytest -rs -v -m integration tests/cell/test_cursor_api_behavior.py

Requires NOVA_API and NOVA_ACCESS_TOKEN environment variables.
"""

from __future__ import annotations

import asyncio
import logging
from math import pi

import pytest

from nova import api
from nova.actions import jnt, ptp
from nova.cell.controllers import virtual_controller
from nova.cell.movement_controller.trajectory_cursor import init_movement_gen
from nova.core.nova import Nova
from nova.exceptions import ErrorDuringMovement
from nova.types import MotionSettings
from nova.types.pose import Pose

logger = logging.getLogger(__name__)

# Slow speed so the robot takes a few seconds to traverse the trajectory,
# giving us time to issue mid-motion commands.
SLOW = MotionSettings(tcp_velocity_limit=20)

CONTROLLER_NAME = "ur-cursor-experiment"


# ---------------------------------------------------------------------------
# Fixtures
# ---------------------------------------------------------------------------


@pytest.fixture
async def experiment_ctx():
    """Yield (nova_api, cell_id, controller_id, motion_group) ready for cursor experiments.

    Creates a virtual UR10e, plans a 3-action trajectory at slow speed,
    and loads it into the trajectory cache.  The trajectory is long enough
    (~several seconds at 20 mm/s) to issue mid-motion commands.
    """
    async with Nova() as nova:
        cell = nova.cell()
        await cell.ensure_controller(
            virtual_controller(
                name=CONTROLLER_NAME,
                manufacturer=api.models.Manufacturer.UNIVERSALROBOTS,
                type="universalrobots-ur10e",
                position=[0.0, -pi / 2, -pi / 2, 0.0, 0.0, 0.0, 0.0],
            )
        )
        controller = await cell.controller(CONTROLLER_NAME)
        async with controller[0] as mg:
            # Move to a known home position first
            home_joints = (0, -pi / 2, -pi / 2, -pi / 2, pi / 2, -pi / 2)
            await mg.plan_and_execute(actions=[jnt(home_joints)], tcp="Flange")

            tcp_pose = await mg.tcp_pose("Flange")
            actions = [
                ptp(tcp_pose @ Pose((0, 0, 50, 0, 0, 0)), SLOW),
                ptp(tcp_pose @ Pose((0, 0, 100, 0, 0, 0)), SLOW),
                ptp(tcp_pose @ Pose((0, 0, 150, 0, 0, 0)), SLOW),
            ]
            joint_trajectory = await mg.plan(actions, "Flange")
            motion_id = await mg._load_planned_motion(joint_trajectory, "Flange")

            yield _ExperimentContext(
                exec_api=nova.api.trajectory_execution_api,
                cell_id=cell.id,
                controller_id=controller.id,
                motion_group=mg,
                motion_id=motion_id,
                joint_trajectory=joint_trajectory,
                actions=actions,
            )

            # Return to home after each test
            try:
                await mg.plan_and_execute(actions=[jnt(home_joints)], tcp="Flange")
            except ErrorDuringMovement:
                logger.warning("Failed to return robot to home position during experiment cleanup")


class _ExperimentContext:
    def __init__(
        self,
        *,
        exec_api,
        cell_id,
        controller_id,
        motion_group,
        motion_id,
        joint_trajectory,
        actions,
    ):
        self.exec_api = exec_api
        self.cell_id = cell_id
        self.controller_id = controller_id
        self.mg = motion_group
        self.motion_id = motion_id
        self.joint_trajectory = joint_trajectory
        self.actions = actions


# ---------------------------------------------------------------------------
# Low-level stream helper
# ---------------------------------------------------------------------------


async def _run_experiment(
    ctx: _ExperimentContext, command_sequence: CommandSequenceFunc, timeout: float = 30.0
) -> ExperimentResult:
    """Drive the execute_trajectory stream with a custom command sequence.

    Args:
        ctx: Experiment context with API handles and trajectory.
        command_sequence: An async callable that receives (response_stream, request_sink)
            and drives the protocol.  It should yield request messages and collect
            responses and state observations into the returned ExperimentResult.
        timeout: Maximum time for the whole experiment.
    """
    result = ExperimentResult()

    async def controller(response_stream):
        """The client_request_generator passed to execute_trajectory."""
        # Phase 1: init handshake
        async for req in init_movement_gen(ctx.motion_id, response_stream, 0.0):
            yield req
        result.init_ok = True
        logger.info("Init handshake complete")

        # Phase 2: hand off to the experiment's command sequence
        async for req in command_sequence(response_stream, result, ctx):
            yield req

    async with asyncio.timeout(timeout):
        await ctx.exec_api.execute_trajectory(
            cell=ctx.cell_id, controller=ctx.controller_id, client_request_generator=controller
        )

    return result


class ExperimentResult:
    """Collects observations from an experiment run."""

    def __init__(self):
        self.init_ok: bool = False
        self.responses: list[api.models.ExecuteTrajectoryResponse] = []
        self.response_types: list[str] = []
        self.errors: list[str] = []
        self.notes: list[str] = []

    def record_response(self, resp: api.models.ExecuteTrajectoryResponse):
        self.responses.append(resp)
        kind = type(resp.root).__name__
        self.response_types.append(kind)
        logger.info(f"  Response: {kind} — {resp.root}")
        if isinstance(resp.root, api.models.MovementErrorResponse):
            self.errors.append(resp.root.message)

    def __repr__(self):
        return (
            f"ExperimentResult(init_ok={self.init_ok}, "
            f"response_types={self.response_types}, "
            f"errors={self.errors}, "
            f"notes={self.notes})"
        )


# Type alias for command sequence callables
from typing import AsyncIterator, Callable

CommandSequenceFunc = Callable[
    [
        AsyncIterator[api.models.ExecuteTrajectoryResponse],  # response_stream
        ExperimentResult,  # result collector
        _ExperimentContext,  # ctx
    ],
    AsyncIterator,  # yields request messages
]


# ---------------------------------------------------------------------------
# Helper: wait until the robot is moving (non-standstill with execute set)
# ---------------------------------------------------------------------------


async def _wait_until_moving(ctx: _ExperimentContext, min_location: float = 0.1) -> float:
    """Poll motion group state until the robot is moving past min_location.

    Returns the location when motion is confirmed.
    """
    async for state in ctx.mg.stream_state():
        if (
            state.execute is not None
            and isinstance(state.execute.details, api.models.TrajectoryDetails)
            and not state.standstill
            and state.execute.details.location.root >= min_location
        ):
            loc = state.execute.details.location.root
            logger.info(f"  Robot moving at location {loc:.3f}")
            return loc
    raise RuntimeError("State stream ended without detecting motion")


async def _wait_until_standstill(ctx: _ExperimentContext) -> float:
    """Poll motion group state until the robot is at standstill with execute set.

    Returns the location at standstill.
    """
    async for state in ctx.mg.stream_state():
        if (
            state.execute is not None
            and isinstance(state.execute.details, api.models.TrajectoryDetails)
            and state.standstill
        ):
            loc = state.execute.details.location.root
            logger.info(f"  Robot at standstill, location {loc:.3f}")
            return loc
    raise RuntimeError("State stream ended without standstill")


# ---------------------------------------------------------------------------
# Experiment 1: Same direction, new target while moving
#   forward() → wait until moving → forward_to(target)
#   Question: does the server accept a second StartMovementRequest(FWD)?
#             Does it respond? Does the robot stop at the new target?
# ---------------------------------------------------------------------------


@pytest.mark.integration
async def test_experiment_1_same_direction_new_target(experiment_ctx):
    """Send Start(FWD) → while moving → Start(FWD, target=1.0).

    Observe:
    - How many StartMovementResponses come back?
    - Does the robot stop at location 1.0?
    - Any MovementErrorResponse?
    """
    ctx = experiment_ctx

    async def command_sequence(response_stream, result, ctx):
        # Send first forward (no target — go to end)
        yield api.models.StartMovementRequest(
            direction=api.models.Direction.DIRECTION_FORWARD,
            target_location=None,
            start_on_io=None,
            pause_on_io=None,
        )
        resp1 = await anext(response_stream)
        result.record_response(resp1)
        result.notes.append(f"Response to first Start(FWD): {type(resp1.root).__name__}")

        # Wait until robot is moving
        await _wait_until_moving(ctx, min_location=0.1)

        # Send second forward with target
        yield api.models.StartMovementRequest(
            direction=api.models.Direction.DIRECTION_FORWARD,
            target_location=api.models.Location(root=1.0),
            start_on_io=None,
            pause_on_io=None,
        )
        resp2 = await anext(response_stream)
        result.record_response(resp2)
        result.notes.append(f"Response to Start(FWD, target=1.0): {type(resp2.root).__name__}")

        # Wait for standstill
        final_loc = await _wait_until_standstill(ctx)
        result.notes.append(f"Final location: {final_loc:.3f}")

        # Drain any remaining responses (MovementError, etc.)
        # Don't yield anything more — let the stream end.

    res = await _run_experiment(ctx, command_sequence)

    logger.info(f"\n=== Experiment 1 Result ===\n{res}\nNotes: {res.notes}")
    # We don't assert specific behavior — this is an experiment.
    # But we log everything so the output reveals the protocol semantics.
    assert res.init_ok, "Init handshake must succeed"


# ---------------------------------------------------------------------------
# Experiment 2: Same direction, duplicate forward while moving
#   forward() → wait until moving → forward() (no target)
#   Question: does the server respond? Is it a no-op?
# ---------------------------------------------------------------------------


@pytest.mark.integration
async def test_experiment_2_same_direction_duplicate_forward(experiment_ctx):
    """Send Start(FWD) → while moving → Start(FWD) again with no target.

    Observe:
    - Does the server send a second StartMovementResponse?
    - Does the robot continue to the end?
    - Any errors?
    """
    ctx = experiment_ctx

    async def command_sequence(response_stream, result, ctx):
        # First forward
        yield api.models.StartMovementRequest(
            direction=api.models.Direction.DIRECTION_FORWARD,
            target_location=None,
            start_on_io=None,
            pause_on_io=None,
        )
        resp1 = await anext(response_stream)
        result.record_response(resp1)

        await _wait_until_moving(ctx)

        # Second forward (identical)
        yield api.models.StartMovementRequest(
            direction=api.models.Direction.DIRECTION_FORWARD,
            target_location=None,
            start_on_io=None,
            pause_on_io=None,
        )
        resp2 = await anext(response_stream)
        result.record_response(resp2)
        result.notes.append(f"Second Start(FWD) response: {type(resp2.root).__name__}")

        # Wait for completion
        final_loc = await _wait_until_standstill(ctx)
        result.notes.append(f"Final location: {final_loc:.3f}")

    res = await _run_experiment(ctx, command_sequence)
    logger.info(f"\n=== Experiment 2 Result ===\n{res}\nNotes: {res.notes}")
    assert res.init_ok


# ---------------------------------------------------------------------------
# Experiment 3: Direction reversal WITHOUT pause
#   forward() → wait until moving → backward()
#   Question: does the server accept it? Error? Implicit pause?
# ---------------------------------------------------------------------------


@pytest.mark.integration
async def test_experiment_3_direction_change_without_pause(experiment_ctx):
    """Send Start(FWD) → while moving → Start(BWD).

    Observe:
    - MovementErrorResponse?
    - Does the robot reverse?
    - Does the server implicitly pause first?
    """
    ctx = experiment_ctx

    async def command_sequence(response_stream, result, ctx):
        # Forward
        yield api.models.StartMovementRequest(
            direction=api.models.Direction.DIRECTION_FORWARD,
            target_location=None,
            start_on_io=None,
            pause_on_io=None,
        )
        resp1 = await anext(response_stream)
        result.record_response(resp1)

        await _wait_until_moving(ctx, min_location=0.3)

        # Reverse without pause
        yield api.models.StartMovementRequest(
            direction=api.models.Direction.DIRECTION_BACKWARD,
            target_location=None,
            start_on_io=None,
            pause_on_io=None,
        )

        # Await the response to the backward request
        resp2 = await asyncio.wait_for(anext(response_stream), timeout=5)
        result.record_response(resp2)
        if isinstance(resp2.root, api.models.MovementErrorResponse):
            result.notes.append(f"ERROR from server: {resp2.root.message}")
        else:
            result.notes.append(f"Backward accepted: {type(resp2.root).__name__}")

        # Now wait for the robot to reach standstill (backward motion completing)
        try:
            final_loc = await asyncio.wait_for(_wait_until_standstill(ctx), timeout=15)
            result.notes.append(f"Final location after reversal: {final_loc:.3f}")
            # If the server reversed, the robot should end up at location 0.0
            # (beginning of trajectory). If it continued forward, it ends at 3.0.
            if final_loc < 0.1:
                result.notes.append("Robot returned to start — full reversal confirmed")
            elif final_loc > 2.9:
                result.notes.append("Robot reached end — server ignored backward?")
            else:
                result.notes.append(f"Robot stopped mid-trajectory at {final_loc:.3f}")
        except TimeoutError:
            result.notes.append("Could not determine final location (timeout)")

    res = await _run_experiment(ctx, command_sequence)
    logger.info(f"\n=== Experiment 3 Result ===\n{res}\nNotes: {res.notes}")
    assert res.init_ok


# ---------------------------------------------------------------------------
# Experiment 4: Direction reversal WITH pause (safe sequence)
#   forward() → wait moving → pause() → wait standstill → backward()
#   This should be the safe/correct sequence.
# ---------------------------------------------------------------------------


@pytest.mark.integration
async def test_experiment_4_direction_change_with_pause(experiment_ctx):
    """Send Start(FWD) → Pause → wait standstill → Start(BWD).

    This is the expected safe sequence for direction reversal.
    Observe: does it complete cleanly? Where does the robot end up?
    """
    ctx = experiment_ctx

    async def command_sequence(response_stream, result, ctx):
        # Forward
        yield api.models.StartMovementRequest(
            direction=api.models.Direction.DIRECTION_FORWARD,
            target_location=None,
            start_on_io=None,
            pause_on_io=None,
        )
        resp1 = await anext(response_stream)
        result.record_response(resp1)

        await _wait_until_moving(ctx, min_location=0.3)

        # Pause
        yield api.models.PauseMovementRequest()
        resp2 = await anext(response_stream)
        result.record_response(resp2)
        result.notes.append(f"Pause response: {type(resp2.root).__name__}")

        # Wait for standstill
        pause_loc = await _wait_until_standstill(ctx)
        result.notes.append(f"Paused at location: {pause_loc:.3f}")

        # Backward
        yield api.models.StartMovementRequest(
            direction=api.models.Direction.DIRECTION_BACKWARD,
            target_location=None,
            start_on_io=None,
            pause_on_io=None,
        )
        resp3 = await anext(response_stream)
        result.record_response(resp3)
        result.notes.append(f"Backward response: {type(resp3.root).__name__}")

        # Wait for the robot to reach the start and stop
        final_loc = await _wait_until_standstill(ctx)
        result.notes.append(f"Final location: {final_loc:.3f}")

    res = await _run_experiment(ctx, command_sequence)
    logger.info(f"\n=== Experiment 4 Result ===\n{res}\nNotes: {res.notes}")
    assert res.init_ok
    assert not res.errors, f"Expected no errors, got: {res.errors}"


# ---------------------------------------------------------------------------
# Experiment 5: Rapid-fire — two forwards before first response arrives
#   forward() → forward() immediately (no await/sleep between)
#   Question: how many responses? In what order? Crash?
# ---------------------------------------------------------------------------


@pytest.mark.integration
async def test_experiment_5_rapid_fire_two_forwards(experiment_ctx):
    """Send Start(FWD) then immediately Start(FWD) without waiting for the
    first response.

    Observe:
    - Do we get 2 StartMovementResponses?
    - In what order?
    - Any errors?
    """
    ctx = experiment_ctx

    async def command_sequence(response_stream, result, ctx):
        # Yield both requests back-to-back
        yield api.models.StartMovementRequest(
            direction=api.models.Direction.DIRECTION_FORWARD,
            target_location=None,
            start_on_io=None,
            pause_on_io=None,
        )
        yield api.models.StartMovementRequest(
            direction=api.models.Direction.DIRECTION_FORWARD,
            target_location=api.models.Location(root=2.0),
            start_on_io=None,
            pause_on_io=None,
        )

        # Collect responses
        for i in range(2):
            try:
                resp = await asyncio.wait_for(anext(response_stream), timeout=5)
                result.record_response(resp)
                result.notes.append(f"Response {i + 1}: {type(resp.root).__name__}")
            except (StopAsyncIteration, TimeoutError) as e:
                result.notes.append(f"Response {i + 1}: {type(e).__name__} — no response")
                break

        # Wait for completion
        try:
            final_loc = await asyncio.wait_for(_wait_until_standstill(ctx), timeout=15)
            result.notes.append(f"Final location: {final_loc:.3f}")
        except TimeoutError:
            result.notes.append("Timeout waiting for standstill")

    res = await _run_experiment(ctx, command_sequence)
    logger.info(f"\n=== Experiment 5 Result ===\n{res}\nNotes: {res.notes}")
    assert res.init_ok
