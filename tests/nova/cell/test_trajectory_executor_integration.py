"""Live integration tests for TrajectoryExecutor.

These require a running NOVA instance (``NOVA_API`` / ``NOVA_ACCESS_TOKEN``) and
are skipped unless the ``integration`` marker is selected. They use a *virtual*
controller, so no physical robot is moved.

The controller and the recorded trajectory are the example's own fixtures — the
only committed configuration with two motion groups on one controller, which is
what the IO barrier needs.
"""

import asyncio
import json
from pathlib import Path

import pytest
import pytest_asyncio

from nova import Nova, api
from nova.actions import jnt
from nova.cell import GroupArgs, MotionGroup, TrajectoryExecutor

CONTROLLER = "kuka-executor-integration"
ROBOT, POSITIONER = f"0@{CONTROLLER}", f"1@{CONTROLLER}"
TCPS = {ROBOT: "1", POSITIONER: "0"}
SYNC_IO = "OUT#1"

# The barrier's promise is that no client round-trip sits between the starts, so
# the controllers acknowledge execution within the same state-publication tick.
# A start serialized over the network would take tens of milliseconds per group.
MAX_START_SKEW_SECONDS = 0.05

_EXAMPLES = Path(__file__).parents[3] / "examples"


def _recorded_trajectory() -> api.models.MultiJointTrajectory:
    """The example's trajectory, keyed for this test's controller."""
    stored = api.models.MultiJointTrajectory.model_validate_json(
        (_EXAMPLES / "multi_motion_group_trajectory.json").read_text()
    )
    samples = stored.joint_positions_by_motion_group_key
    return api.models.MultiJointTrajectory(
        joint_positions_by_motion_group_key={
            ROBOT: samples["0@kuka"],
            POSITIONER: samples["1@kuka"],
        },
        times=stored.times,
        locations=stored.locations,
    )


def _starts(trajectory: api.models.MultiJointTrajectory) -> dict[str, tuple[float, ...]]:
    samples = trajectory.joint_positions_by_motion_group_key
    return {name: tuple(samples[name][0]) for name in TCPS}


@pytest_asyncio.fixture(scope="module", loop_scope="module")
async def synchronized_groups():
    """Both motion groups of a virtual KUKA, standing at the recorded start.

    Getting there costs more than the tests themselves, so it is shared: the
    tests that do not need the recorded trajectory build theirs from wherever
    the groups stand. The robot is spawned at the start; ``initial_joint_position``
    places the first motion group only, so the positioner has to drive there.
    """
    starts = _starts(_recorded_trajectory())

    async with Nova() as nova:
        cell = nova.cell()
        await cell.ensure_controller(
            api.models.RobotController(
                name=CONTROLLER,
                configuration=api.models.VirtualController(
                    manufacturer=api.models.Manufacturer.KUKA,
                    type="kuka-kr210_r2700_2",
                    json=(_EXAMPLES / "multi_motion_group_controller.json").read_text(),
                    initial_joint_position=json.dumps([*starts[ROBOT], 0.0]),
                ),
            )
        )
        controller = await cell.controller(CONTROLLER)
        groups = {name: controller.motion_group(name) for name in TCPS}
        for name, group in groups.items():
            await group.plan_and_execute([jnt(list(starts[name]))], tcp=TCPS[name])
        yield controller, groups


async def _first_execution_acknowledgement(
    group, acknowledged: dict[str, object], name: str
) -> None:
    """Record when the controller reported this group's trajectory running."""
    async for state in group.stream_state():
        details = state.execute.details if state.execute else None
        if details is not None and isinstance(details.state, api.models.TrajectoryRunning):
            acknowledged[name] = state.timestamp
            return


async def _ramp_from_here(groups: dict[str, MotionGroup]) -> api.models.MultiJointTrajectory:
    """A two-second ramp of each group's last joint, starting where it stands.

    Taking the start from the live position keeps these tests independent of
    each other and of the recorded trajectory, which may only be executed from
    its own first sample.
    """
    samples, span, delta = 21, 2.0, 0.1
    parameter = [span * i / (samples - 1) for i in range(samples)]
    joints = {name: tuple(await group.joints()) for name, group in groups.items()}
    return api.models.MultiJointTrajectory(
        joint_positions_by_motion_group_key={
            name: [[*start[:-1], start[-1] + delta * p / span] for p in parameter]
            for name, start in joints.items()
        },
        times=parameter,
        locations=parameter,
    )


def _executor(groups: dict[str, MotionGroup]) -> TrajectoryExecutor:
    return TrajectoryExecutor.builder(groups).sync_on_io(SYNC_IO, controller=CONTROLLER).build()


_GROUP_ARGS = {name: GroupArgs(tcp=tcp) for name, tcp in TCPS.items()}


@pytest.mark.asyncio(loop_scope="module")
@pytest.mark.integration
async def test_execute_recorded_trajectory_starts_both_groups_in_one_tick(synchronized_groups):
    """The barrier starts both groups together and runs the trajectory to its end."""
    controller, groups = synchronized_groups
    trajectory = _recorded_trajectory()
    samples = trajectory.joint_positions_by_motion_group_key

    acknowledged: dict[str, object] = {}
    watchers = [
        asyncio.create_task(_first_execution_acknowledgement(group, acknowledged, name))
        for name, group in groups.items()
    ]
    try:
        await asyncio.wait_for(
            _executor(groups).execute(trajectory, groups=_GROUP_ARGS), timeout=120
        )
    finally:
        for watcher in watchers:
            watcher.cancel()

    for name, group in groups.items():
        expected = tuple(samples[name][-1])
        reached = tuple(await group.joints())
        assert max(abs(a - b) for a, b in zip(reached, expected)) < 1e-3

    assert set(acknowledged) == set(groups)
    skew = abs((acknowledged[ROBOT] - acknowledged[POSITIONER]).total_seconds())
    assert skew <= MAX_START_SKEW_SECONDS
    assert await controller.read(SYNC_IO) is True


@pytest.mark.asyncio(loop_scope="module")
@pytest.mark.integration
@pytest.mark.xfail(
    reason="RAE bug on SM 26.7: a forward() after forward_to() keeps the stale "
    "target from the forward_to and does not advance to the end. Reproduced live "
    "on the single TrajectoryCursor too, so it is controller-side, not the SDK.",
    strict=False,
)
async def test_forward_to_intermediate_target_keeps_the_session_open(synchronized_groups):
    """Stopping at an intermediate target must not end the session.

    The controller reports ``TrajectoryEnded`` at every commanded stop, so a
    session that trusted that state would tear down here instead of letting the
    caller step on.
    """
    _, groups = synchronized_groups
    trajectory = await _ramp_from_here(groups)

    async with _executor(groups).attach(trajectory, _GROUP_ARGS) as cursor:
        await asyncio.wait_for(cursor.forward_to(1.0), timeout=60)
        assert 1.0 - 0.01 <= cursor.current_location < 2.0

        # The session is still usable: stepping on reaches the end.
        await asyncio.wait_for(cursor.forward(), timeout=60)
        assert cursor.current_location >= 2.0 - 0.01


@pytest.mark.asyncio(loop_scope="module")
@pytest.mark.integration
@pytest.mark.xfail(
    reason="RAE bug on SM 26.7: pausing a slow ramp is not honored — the "
    "trajectory keeps running and never emits TrajectoryPausedByUser, so the "
    "pause never completes. Controller-side; flaky, hence strict=False.",
    strict=False,
)
async def test_pause_then_forward_rearms_the_barrier(synchronized_groups):
    """A paused session resumes through the barrier and reaches the end."""
    _, groups = synchronized_groups
    trajectory = await _ramp_from_here(groups)
    ends = {
        name: tuple(joints[-1])
        for name, joints in trajectory.joint_positions_by_motion_group_key.items()
    }

    async with _executor(groups).attach(trajectory, _GROUP_ARGS) as cursor:
        forward = cursor.forward()
        while cursor.current_location <= 0.0:
            await asyncio.sleep(0.05)
        paused = cursor.pause()
        assert paused is not None
        await asyncio.wait_for(paused, timeout=30)
        with pytest.raises(asyncio.CancelledError):
            await forward

        paused_at = cursor.current_location
        assert paused_at < 2.0

        await asyncio.wait_for(cursor.forward(), timeout=60)
        assert cursor.current_location > paused_at

    for name, group in groups.items():
        reached = tuple(await group.joints())
        assert max(abs(a - b) for a, b in zip(reached, ends[name])) < 1e-3
