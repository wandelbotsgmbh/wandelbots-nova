"""Live integration tests for TrajectoryCursor.

These tests require a running NOVA instance (set ``NOVA_API`` / ``NOVA_ACCESS_TOKEN``)
and are skipped unless the ``integration`` marker is selected. They use a *virtual*
controller, so no physical robot is moved.

Two areas are covered:

1. Source spans and circular actions: validates end-to-end against the real
   planner that a ``circular`` motion is exactly **one** action (one trajectory
   location unit), not two separate targets for its intermediate and target
   poses, and that every planned action carries an exact
   :class:`~nova.utils.SourceLocation` the cursor surfaces on its MotionEvent.
2. ``start_on_io`` / ``pause_on_io``: the cursor is plugged in as the
   ``movement_controller`` passed to ``MotionGroup.execute()`` — the same
   generic plug-in point ``move_forward`` itself uses — via a small adapter
   factory (``_make_cursor_controller``). This is **not** wired through
   ``TrajectoryTuner``/``nova/cell/tuner.py`` (that path doesn't forward these
   params — a deliberate, separate scope decision). The timing-sensitive tests
   are disabled (``_test_`` prefix, ``@pytest.mark.integration`` commented out)
   for the same reason ``test_pause_on_io.py``'s own mid-motion-trigger test is
   disabled: virtual-controller timing has been flaky in CI. They're checked in
   as a reference for manual verification against a live/virtual controller.
"""

import asyncio
from math import pi

import pytest

from nova import Nova, api
from nova.actions import cartesian_ptp, circular, jnt, joint_ptp
from nova.actions.container import MovementControllerContext
from nova.cell import virtual_controller
from nova.cell.movement_controller.trajectory_cursor import (
    MotionEvent,
    TrajectoryCursor,
    motion_started,
)
from nova.types import MovementControllerFunction, Pose
from nova.types.motion_settings import MotionSettings

# A non-singular UR10e configuration (joint 5 != 0 avoids the wrist singularity).
initial_joint_positions = [pi / 2, -pi / 2, pi / 2, 0, pi / 2, 0]

# KUKA home position, used for the start_on_io/pause_on_io tests below (matches
# test_pause_on_io.py's setup, which also relies on this controller's IO support).
kuka_initial_joint_positions = [0.0, -pi / 2, -pi / 2, 0.0, 0.0, 0.0, 0.0]


@pytest.fixture
async def ur_mg():
    """Virtual UR motion group ready for planning."""
    controller_name = "ur-cursor-integration"

    async with Nova() as nova:
        cell = nova.cell()
        await cell.ensure_controller(
            virtual_controller(
                name=controller_name,
                manufacturer=api.models.Manufacturer.UNIVERSALROBOTS,
                type="universalrobots-ur10e",
                position=[*initial_joint_positions, 0],
            )
        )
        ur = await cell.controller(controller_name)
        async with ur[0] as mg:
            yield mg


@pytest.fixture
async def kuka_mg():
    """Virtual KUKA motion group + controller handle, for IO read/write."""
    controller_name = "kuka-cursor-io-test"

    async with Nova() as nova:
        cell = nova.cell()
        await cell.ensure_controller(
            virtual_controller(
                name=controller_name,
                manufacturer=api.models.Manufacturer.KUKA,
                type="kuka-kr6_r700_sixx",
                position=kuka_initial_joint_positions,
            )
        )
        kuka = await cell.controller(controller_name)
        async with kuka[0] as mg:
            yield mg, kuka


async def _build_circular_actions(mg):
    """Build a small trajectory whose middle action is a single circular move."""
    tcp = (await mg.tcp_names())[0]
    home = tuple(initial_joint_positions)
    current = (await mg.forward_kinematics(joints=[list(home)], tcp=tcp))[0]

    intermediate = current @ Pose((30, 0, 30, 0, 0, 0))
    target = current @ Pose((60, 0, 0, 0, 0, 0))

    # fmt: off so the circular call keeps a genuine multi-line source span.
    # fmt: off
    actions = [
        joint_ptp(home),
        circular(
            target=target,
            intermediate=intermediate,
        ),
        cartesian_ptp(current),
    ]
    # fmt: on
    return actions, tcp


@pytest.mark.asyncio
@pytest.mark.integration
async def test_circular_is_a_single_action_in_planned_trajectory(ur_mg):
    """The planner treats a circular move as one action (one location unit)."""
    actions, tcp = await _build_circular_actions(ur_mg)

    trajectory = await ur_mg.plan(
        start_joint_position=tuple(initial_joint_positions), actions=actions, tcp=tcp
    )

    # End location equals the number of actions: circular counts as exactly one,
    # not two targets. This is the cursor's location-to-action contract.
    assert abs(trajectory.locations[-1].root - len(actions)) < 0.01

    circular_action = actions[1]
    loc = circular_action.source_location
    assert loc is not None
    # One selection spanning the whole circular(...) call, not two targets.
    assert loc.end_line is not None and loc.start_line is not None
    assert loc.end_line > loc.start_line


@pytest.mark.asyncio
@pytest.mark.integration
async def test_cursor_emits_source_span_for_current_action(ur_mg):
    """The initial MotionEvent carries the exact source span to highlight."""
    actions, tcp = await _build_circular_actions(ur_mg)
    trajectory = await ur_mg.plan(
        start_joint_position=tuple(initial_joint_positions), actions=actions, tcp=tcp
    )

    received: list[MotionEvent] = []

    @motion_started.connect
    async def _capture(_sender, event: MotionEvent):
        received.append(event)

    async def _state_stream():
        # The initial event is emitted at construction time without movement; an
        # empty stream is sufficient to capture it.
        if False:
            yield  # pragma: no cover

    try:
        cursor = TrajectoryCursor(
            motion_id="integration-test",
            motion_group_state_stream=_state_stream(),
            joint_trajectory=trajectory,
            actions=actions,
            initial_location=0.0,
        )
        await cursor._initialize_task
    finally:
        motion_started.disconnect(_capture)

    assert received, "no MotionEvent was emitted on cursor initialization"
    event = received[0]
    assert event.current_action is actions[0]
    assert event.current_action_source is not None
    assert event.current_action_source.start_line is not None


# ---------------------------------------------------------------------------
# start_on_io / pause_on_io
# ---------------------------------------------------------------------------


def _make_cursor_controller(joint_trajectory, actions, cursor_ready: asyncio.Event, holder: list):
    """Adapter plugging TrajectoryCursor in as an execute() movement_controller.

    Mirrors the shape of the built-in move_forward controller: a factory that
    receives a MovementControllerContext and returns the async-generator
    "controller" function passed as client_request_generator. Unlike
    move_forward, the cursor never auto-starts movement — the caller must
    explicitly call cursor.forward()/backward() once the cursor is published
    via `holder`.
    """

    def factory(context: MovementControllerContext) -> MovementControllerFunction:
        cursor = TrajectoryCursor(
            motion_id=context.motion_id,
            motion_group_state_stream=context.motion_group_state_stream_gen(),
            joint_trajectory=joint_trajectory,
            actions=actions,
            initial_location=0.0,
        )
        holder.append(cursor)
        cursor_ready.set()
        return cursor.cntrl

    return factory


async def _plan_move(mg, delta: float):
    """Plan a joint move of ``delta`` radians from the current position on joint 0;
    returns (trajectory, actions). Callers pick delta to fit the test: small and
    slow when movement should barely progress (start_on_io gating), large and
    slow when there must be enough travel left to interrupt mid-flight
    (pause_on_io) — see test_pause_on_io.py's own precedent for the same choice.
    """
    current_joints = await mg.joints()
    target_joints = list(current_joints)
    target_joints[0] += delta
    actions = [jnt(target_joints, settings=MotionSettings(tcp_velocity_limit=30))]
    trajectory = await mg.plan(actions, tcp="Flange", start_joint_position=current_joints)
    return trajectory, actions


# TODO: enable once verified against a live/virtual controller; not run in CI.
# @pytest.mark.asyncio
# @pytest.mark.integration
async def _test_cursor_forward_start_on_io_delays_movement_start(kuka_mg):
    """start_on_io must gate the actual start of movement: the robot should not
    move until the IO condition becomes true, even though forward() was already
    called."""
    mg, kuka = kuka_mg
    await kuka.write("OUT#900", False)

    trajectory, actions = await _plan_move(mg, delta=0.1)
    start_joints = await mg.joints()

    cursor_ready = asyncio.Event()
    holder: list[TrajectoryCursor] = []
    execute_task = asyncio.create_task(
        mg.execute(
            trajectory,
            "Flange",
            actions,
            movement_controller=_make_cursor_controller(trajectory, actions, cursor_ready, holder),
        )
    )
    await asyncio.wait_for(cursor_ready.wait(), timeout=10.0)
    cursor = holder[0]

    start_io = api.models.StartOnIO(
        io=api.models.IOBooleanValue(io="OUT#900", value=True),
        comparator=api.models.Comparator.COMPARATOR_EQUALS,
        io_origin=api.models.IOOrigin.CONTROLLER,
    )
    forward_future = cursor.forward(start_on_io=start_io)

    try:
        # IO condition not met yet: the robot must not have moved.
        await asyncio.sleep(1.0)
        joints_while_waiting = await mg.joints()
        assert abs(joints_while_waiting[0] - start_joints[0]) < 0.01, (
            "robot moved before start_on_io condition was met"
        )
        assert not forward_future.done()

        # Flip the IO: movement should now proceed to completion.
        await kuka.write("OUT#900", True)
        await asyncio.wait_for(forward_future, timeout=30.0)

        final_joints = await mg.joints()
        assert abs(final_joints[0] - start_joints[0]) > 0.01, "robot never moved"
    finally:
        await kuka.write("OUT#900", False)
        cursor.detach()
        execute_task.cancel()


# TODO: enable once verified against a live/virtual controller; not run in CI.
# Disabled to match test_pause_on_io.py's own (flaky-in-CI) mid-motion-trigger test.
# @pytest.mark.asyncio
# @pytest.mark.integration
async def _test_cursor_forward_pause_on_io_stops_early_mid_trajectory(kuka_mg):
    """pause_on_io must stop the cursor's forward() early, mid-trajectory, with
    no exception raised — TrajectoryPausedOnIO resolves the operation cleanly."""
    mg, kuka = kuka_mg
    await kuka.write("OUT#900", False)

    trajectory, actions = await _plan_move(mg, delta=1.5)
    start_joints = await mg.joints()

    cursor_ready = asyncio.Event()
    holder: list[TrajectoryCursor] = []
    execute_task = asyncio.create_task(
        mg.execute(
            trajectory,
            "Flange",
            actions,
            movement_controller=_make_cursor_controller(trajectory, actions, cursor_ready, holder),
        )
    )
    await asyncio.wait_for(cursor_ready.wait(), timeout=10.0)
    cursor = holder[0]

    pause_io = api.models.PauseOnIO(
        io=api.models.IOBooleanValue(io="OUT#900", value=True),
        comparator=api.models.Comparator.COMPARATOR_EQUALS,
        io_origin=api.models.IOOrigin.CONTROLLER,
    )
    forward_future = cursor.forward(pause_on_io=pause_io)

    try:

        async def trigger_io_once_moving():
            start_time = asyncio.get_event_loop().time()
            while True:
                current = await mg.joints()
                if abs(current[0] - start_joints[0]) > 0.01:
                    await kuka.write("OUT#900", True)
                    return
                if asyncio.get_event_loop().time() - start_time > 5.0:
                    raise TimeoutError("Motion never started")
                await asyncio.sleep(0.1)

        await asyncio.wait_for(trigger_io_once_moving(), timeout=10.0)

        # No exception should propagate: TrajectoryPausedOnIO resolves the
        # operation as ended, not as an error.
        result = await asyncio.wait_for(forward_future, timeout=15.0)

        final_joints = await mg.joints()
        target_joints = actions[0].target
        movement_amount = abs(final_joints[0] - start_joints[0])
        distance_to_target = abs(final_joints[0] - target_joints[0])
        assert movement_amount > 0.01, "robot didn't move"
        assert distance_to_target > 0.1, "motion wasn't interrupted early"
        assert result.final_location is not None
    finally:
        await kuka.write("OUT#900", False)
        cursor.detach()
        execute_task.cancel()


# TODO: enable once verified against a live/virtual controller; not run in CI.
# @pytest.mark.asyncio
# @pytest.mark.integration
async def _test_cursor_detach_on_standstill_tears_down_on_pause_on_io(kuka_mg):
    """detach_on_standstill=True must fully tear down the control loop when
    pause_on_io fires mid-motion, exactly as it does on true completion — this
    pins the TrajectoryPausedOnIO -> is_ended (not is_paused) contract for a
    cursor caller who opted into auto-detach."""
    mg, kuka = kuka_mg
    await kuka.write("OUT#900", False)

    trajectory, actions = await _plan_move(mg, delta=1.5)
    start_joints = await mg.joints()

    cursor_ready = asyncio.Event()
    holder: list[TrajectoryCursor] = []

    def factory(context: MovementControllerContext) -> MovementControllerFunction:
        cursor = TrajectoryCursor(
            motion_id=context.motion_id,
            motion_group_state_stream=context.motion_group_state_stream_gen(),
            joint_trajectory=trajectory,
            actions=actions,
            initial_location=0.0,
            detach_on_standstill=True,
        )
        holder.append(cursor)
        cursor_ready.set()
        return cursor.cntrl

    execute_task = asyncio.create_task(
        mg.execute(trajectory, "Flange", actions, movement_controller=factory)
    )
    await asyncio.wait_for(cursor_ready.wait(), timeout=10.0)
    cursor = holder[0]

    pause_io = api.models.PauseOnIO(
        io=api.models.IOBooleanValue(io="OUT#900", value=True),
        comparator=api.models.Comparator.COMPARATOR_EQUALS,
        io_origin=api.models.IOOrigin.CONTROLLER,
    )
    forward_future = cursor.forward(pause_on_io=pause_io)

    try:

        async def trigger_io_once_moving():
            start_time = asyncio.get_event_loop().time()
            while True:
                current = await mg.joints()
                if abs(current[0] - start_joints[0]) > 0.01:
                    await kuka.write("OUT#900", True)
                    return
                if asyncio.get_event_loop().time() - start_time > 5.0:
                    raise TimeoutError("Motion never started")
                await asyncio.sleep(0.1)

        await asyncio.wait_for(trigger_io_once_moving(), timeout=10.0)

        # The operation resolves cleanly...
        await asyncio.wait_for(forward_future, timeout=15.0)

        # ...but the cursor's control loop must have torn itself down, since
        # detach_on_standstill=True treats the IO-triggered pause like real
        # completion (is_ended), not like a resumable pause (is_paused).
        assert cursor._stop_event.is_set()
        with pytest.raises(StopAsyncIteration):
            await asyncio.wait_for(cursor.__anext__(), timeout=2.0)

        # A subsequent forward() must fail fast (not hang) rather than queue
        # a command nothing will ever send.
        stale_future = cursor.forward()
        with pytest.raises(RuntimeError):
            await asyncio.wait_for(stale_future, timeout=2.0)
    finally:
        await kuka.write("OUT#900", False)
        cursor.detach()
        execute_task.cancel()
