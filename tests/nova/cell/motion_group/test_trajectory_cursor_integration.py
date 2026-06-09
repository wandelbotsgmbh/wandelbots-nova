"""Live integration tests for trajectory-cursor source spans and circular actions.

These tests require a running NOVA instance (set ``NOVA_API`` / ``NOVA_ACCESS_TOKEN``)
and are skipped unless the ``integration`` marker is selected. They use a *virtual*
controller, so no physical robot is moved.

They validate, end-to-end against the real planner, the two guarantees that the
source-span work depends on:

1. A ``circular`` motion is exactly **one** action (one trajectory location unit),
   not two separate targets for its intermediate and target poses.
2. Every planned action carries an exact :class:`~nova.utils.SourceLocation`, and
   the cursor surfaces it on the emitted :class:`MotionEvent`.
"""

from math import pi

import pytest

from nova import Nova, api
from nova.actions import cartesian_ptp, circular, joint_ptp
from nova.cell import virtual_controller
from nova.cell.movement_controller.trajectory_cursor import (
    MotionEvent,
    TrajectoryCursor,
    motion_started,
)
from nova.types import Pose

# A non-singular UR10e configuration (joint 5 != 0 avoids the wrist singularity).
initial_joint_positions = [pi / 2, -pi / 2, pi / 2, 0, pi / 2, 0]


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
