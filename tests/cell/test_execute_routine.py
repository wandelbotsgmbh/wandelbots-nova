"""Executing a CommandRoutine through ``MotionGroup.plan_and_execute_routine``.

Behaviour is asserted at the public entry point only: what the planner was asked
to plan, which states came out, and the errors. The gateway is a double; the
``executeTrajectory`` protocol fake is shared with ``test_execute_shared_stream``.
"""

import asyncio
import math
from datetime import datetime, timezone
from unittest.mock import AsyncMock, MagicMock

import pytest

from nova import api
from nova.cell.motion_group import MotionGroup
from nova.cell.state_stream import MotionGroupStateStreamRegistry
from nova.command_routines import (
    command_routine,
    marker,
    move_cartesian_ptp,
    move_circular,
    move_joint_ptp,
    move_linear,
    set_io,
    at_path_fraction,
    at_time,
)
from nova.core.gateway import ApiGateway
from nova.datasets import Dataset, DatasetFrame, DatasetPose
from nova.exceptions import PoseResolutionError, UnsupportedRoutineCommand
from nova.types import Pose
from tests.cell.test_execute_shared_stream import _ExecuteProtocolFake
from tests.cell.test_state_stream import FakeUpstream

pytestmark = pytest.mark.asyncio


def _pose(x: float) -> api.models.InlinePoseReference:
    return api.models.InlinePoseReference(
        pose=api.models.Pose(position=[x, 0.0, 0.0], orientation=[0.0, 0.0, 0.0])
    )


def _setup(tcp: str = "Flange") -> api.models.MotionGroupSetup:
    return api.models.MotionGroupSetup(
        motion_group_model="UniversalRobots_UR10e",
        cycle_time=8,
        mounting=api.models.Pose(position=[0.0] * 3, orientation=[0.0] * 3),
        tcp_offset=api.models.Pose(position=[0.0] * 3, orientation=[0.0] * 3),
        global_limits=api.models.LimitSet(),
    )


async def _plan(cell, plan_trajectory_request) -> api.models.PlanTrajectoryResponse:
    """Plan every motion command as one unit of path location."""
    count = len(plan_trajectory_request.motion_commands)
    return api.models.PlanTrajectoryResponse(
        response=api.models.JointTrajectory(
            joint_positions=[[0.0] * 6] * (count + 1),
            times=[float(i) for i in range(count + 1)],
            locations=[float(i) for i in range(count + 1)],
        )
    )


def _motion_group(upstream: FakeUpstream) -> MotionGroup:
    gateway = MagicMock(spec=ApiGateway)
    registry = MotionGroupStateStreamRegistry(
        open_stream=lambda cell, controller_id, motion_group_id, rate: upstream.open(rate)
    )
    gateway.motion_group_state_stream = registry.stream
    gateway.trajectory_execution_api = MagicMock()
    gateway.trajectory_execution_api.execute_trajectory = _ExecuteProtocolFake(upstream)
    gateway.trajectory_planning_api = MagicMock()
    gateway.trajectory_planning_api.plan_trajectory = AsyncMock(side_effect=_plan)
    motion_group = MotionGroup(
        api_client=gateway, cell="cell", controller_id="ctrl", motion_group_id="0@ctrl"
    )
    motion_group._load_planned_motion = AsyncMock(return_value="traj-1")
    return motion_group


def _planned_commands(motion_group: MotionGroup) -> list[api.models.MotionCommand]:
    plan = motion_group._api_client.trajectory_planning_api.plan_trajectory
    plan.assert_awaited_once()
    return plan.await_args.kwargs["plan_trajectory_request"].motion_commands


def _routine(*commands, **context) -> api.models.CommandRoutine:
    context = {
        "tcp": "Flange",
        "motion_group_setup": _setup(),
        "start_joint_position": [0.0] * 6,
    } | context
    return command_routine("r", commands=list(commands), **context)


_BRANCH = api.models.KinematicConfiguration(
    kinematic_branch=api.models.KinematicBranch(
        shoulder_branch="FRONT", elbow_branch="UP", wrist_branch="NO_FLIP"
    )
)


def _dataset(frames: dict[str, DatasetFrame], poses: dict[str, DatasetPose]) -> Dataset:
    now = datetime(2026, 1, 1, tzinfo=timezone.utc)
    dataset = Dataset(
        dataset="d", revision=1, created_at=now, updated_at=now, frames=frames, poses=poses
    )
    dataset.attach_frame_tree()
    return dataset


def _fixture_dataset() -> Dataset:
    """A frame shifted 1000 mm in x, with one pose taught on it."""
    return _dataset(
        frames={"fixture": DatasetFrame(frame="fixture", dataset="d", pose=Pose((1000, 0, 0)))},
        poses={
            "slot": DatasetPose(
                dataset_pose="slot",
                dataset="d",
                frame="fixture",
                pose=Pose((100, 0, 0)),
                kinematic_configuration=_BRANCH,
            )
        },
    )


class TestExecution:
    """A routine is planned and executed with its own context."""

    async def test_runs_a_motion_routine_to_completion(self):
        # GIVEN
        motion_group = _motion_group(FakeUpstream())
        routine = _routine(
            move_linear(_pose(100.0)), move_linear(_pose(200.0)), motion_group="0@ctrl"
        )

        # WHEN
        async with asyncio.timeout(5):
            await motion_group.plan_and_execute_routine(routine)

        # THEN
        motion_group._load_planned_motion.assert_awaited_once()
        paths = [command.path for command in _planned_commands(motion_group)]
        assert [tuple(path.target_pose.position) for path in paths] == [
            (100.0, 0.0, 0.0),
            (200.0, 0.0, 0.0),
        ]

    async def test_uses_the_routine_context(self):
        # GIVEN
        motion_group = _motion_group(FakeUpstream())
        setup = _setup()
        routine = _routine(
            move_linear(_pose(100.0)), motion_group_setup=setup, start_joint_position=[0.1] * 6
        )

        # WHEN
        async with asyncio.timeout(5):
            await motion_group.plan_and_execute_routine(routine)

        # THEN
        request = (
            motion_group._api_client.trajectory_planning_api.plan_trajectory.await_args.kwargs[
                "plan_trajectory_request"
            ]
        )
        assert request.start_joint_position == [0.1] * 6
        assert request.motion_group_setup.motion_group_model == setup.motion_group_model
        assert request.motion_group_setup.tcp_offset == setup.tcp_offset

    async def test_command_settings_take_precedence_over_routine_defaults(self):
        # GIVEN
        motion_group = _motion_group(FakeUpstream())
        default = api.models.MotionSettings(
            limits_override=api.models.LimitsOverride(tcp_velocity_limit=10.0)
        )
        own = api.models.MotionSettings(
            limits_override=api.models.LimitsOverride(tcp_velocity_limit=99.0)
        )
        routine = _routine(
            move_linear(_pose(1.0)),
            move_linear(_pose(2.0), settings=own),
            default_motion_settings=default,
        )

        # WHEN
        async with asyncio.timeout(5):
            await motion_group.plan_and_execute_routine(routine)

        # THEN
        limits = [
            command.limits_override.tcp_velocity_limit
            for command in _planned_commands(motion_group)
        ]
        assert limits == [10.0, 99.0]

    async def test_executes_every_explicit_path_type(self):
        # GIVEN
        motion_group = _motion_group(FakeUpstream())
        routine = _routine(
            move_linear(_pose(1.0)),
            move_cartesian_ptp(_pose(2.0)),
            move_joint_ptp([0.0] * 6),
            move_circular(_pose(3.0), via_pose=_pose(2.5)),
        )

        # WHEN
        async with asyncio.timeout(5):
            await motion_group.plan_and_execute_routine(routine)

        # THEN
        assert [
            command.path.path_definition_name for command in _planned_commands(motion_group)
        ] == ["PathLine", "PathCartesianPTP", "PathJointPTP", "PathCircle"]

    async def test_a_planned_routine_can_be_executed_later(self):
        # GIVEN
        motion_group = _motion_group(FakeUpstream())
        routine = _routine(move_linear(_pose(1.0)), move_linear(_pose(2.0)))
        trajectory = await motion_group.plan_routine(routine)
        motion_group._load_planned_motion.assert_not_awaited()  # planning does not move the robot

        # WHEN
        async with asyncio.timeout(5):
            await motion_group.execute_routine(routine, trajectory)

        # THEN
        motion_group._load_planned_motion.assert_awaited_once()


class _RecordingProtocolFake(_ExecuteProtocolFake):
    """Records the start requests the execution sends to the controller."""

    def __init__(self, upstream: FakeUpstream):
        super().__init__(upstream)
        self.start_requests: list[api.models.StartMovementRequest] = []

    async def __call__(self, cell, controller, client_request_generator):
        async def recording(responses):
            async for request in client_request_generator(responses):
                if isinstance(request, api.models.StartMovementRequest):
                    self.start_requests.append(request)
                yield request

        await super().__call__(cell, controller, recording)


class TestSetIO:
    """set_io commands become the outputs requested at start of execution."""

    async def _run(self, *commands) -> list[api.models.SetIO]:
        upstream = FakeUpstream()
        motion_group = _motion_group(upstream)
        recorder = _RecordingProtocolFake(upstream)
        motion_group._api_client.trajectory_execution_api.execute_trajectory = recorder

        async with asyncio.timeout(5):
            await motion_group.plan_and_execute_routine(_routine(*commands))

        return recorder.start_requests[0].set_outputs

    async def test_set_io_without_trigger_switches_at_the_following_motion_boundary(self):
        # WHEN
        outputs = await self._run(
            move_linear(_pose(1.0)), set_io("valve", True), move_linear(_pose(2.0))
        )

        # THEN
        assert [(o.io.io, o.io.value, o.location) for o in outputs] == [("valve", True, 1.0)]

    async def test_integer_and_analog_values_and_bus_origin_are_honoured(self):
        # WHEN
        outputs = await self._run(
            set_io("count", 7),
            set_io("speed", 0.5, io_origin=api.models.IOOrigin.BUS_IO),
            move_linear(_pose(1.0)),
        )

        # THEN
        assert [(o.io.io, o.io.value, o.io_origin) for o in outputs] == [
            ("count", "7", api.models.IOOrigin.CONTROLLER),
            ("speed", 0.5, api.models.IOOrigin.BUS_IO),
        ]

    async def test_path_fraction_trigger_switches_inside_the_following_motion(self):
        # WHEN
        outputs = await self._run(
            move_linear(_pose(1.0)),
            set_io("valve", True, at=at_path_fraction(0.5)),
            move_linear(_pose(2.0)),
        )

        # THEN
        assert [o.location for o in outputs] == [1.5]

    async def test_time_trigger_switches_at_the_triggered_location(self):
        # WHEN (planned times equal locations)
        outputs = await self._run(
            set_io("valve", True, at=at_time(0.25, api.models.AtReference.PREVIOUS)),
            move_linear(_pose(1.0)),
        )

        # THEN
        assert [o.location for o in outputs] == [0.25]


class TestRejection:
    """Routines that cannot run are rejected before any motion."""

    async def test_rejects_an_unsupported_command_before_any_motion(self):
        # GIVEN
        motion_group = _motion_group(FakeUpstream())
        routine = _routine(move_linear(_pose(1.0)), marker("grasp"))

        # WHEN
        with pytest.raises(UnsupportedRoutineCommand, match=r"Command 1 \(marker\)"):
            await motion_group.plan_and_execute_routine(routine)

        # THEN
        motion_group._api_client.trajectory_planning_api.plan_trajectory.assert_not_awaited()
        motion_group._load_planned_motion.assert_not_awaited()

    async def test_rejects_a_routine_for_another_motion_group(self):
        # GIVEN
        motion_group = _motion_group(FakeUpstream())
        routine = _routine(move_linear(_pose(1.0)), motion_group="1@other")

        # WHEN
        # THEN
        with pytest.raises(ValueError, match="1@other"):
            await motion_group.plan_and_execute_routine(routine)


class TestStop:
    """Cancelling the awaiting task stops the run."""

    async def test_cancelling_the_run_stops_it(self):
        # GIVEN
        upstream = FakeUpstream()
        motion_group = _motion_group(upstream)
        task = asyncio.create_task(
            motion_group.plan_and_execute_routine(_routine(move_linear(_pose(1.0))))
        )
        async with asyncio.timeout(5):
            while not motion_group._load_planned_motion.await_count:
                await asyncio.sleep(0)

            # WHEN
            task.cancel()
            with pytest.raises((asyncio.CancelledError, BaseExceptionGroup)):
                await task

        # THEN
        await asyncio.wait_for(upstream.closed.wait(), 1.0)


class TestPoseResolution:
    """Dataset poses are resolved to world poses before planning."""

    async def test_local_pose_is_resolved_to_world_with_its_kinematic_configuration(self):
        # GIVEN
        motion_group = _motion_group(FakeUpstream())
        routine = _routine(move_cartesian_ptp("slot"))

        # WHEN
        async with asyncio.timeout(5):
            await motion_group.plan_and_execute_routine(routine, dataset=_fixture_dataset())

        # THEN
        (command,) = _planned_commands(motion_group)
        assert tuple(command.path.target_pose.position) == (1100.0, 0.0, 0.0)
        assert command.path.kinematic_configuration == _BRANCH

    async def test_inline_pose_in_a_dataset_frame_is_resolved_to_world(self):
        # GIVEN
        motion_group = _motion_group(FakeUpstream())
        in_frame = api.models.InlinePoseReference(
            pose=api.models.Pose(position=[5.0, 0.0, 0.0], orientation=[0.0] * 3), frame="fixture"
        )

        # WHEN
        async with asyncio.timeout(5):
            await motion_group.plan_and_execute_routine(
                _routine(move_linear(in_frame)), dataset=_fixture_dataset()
            )

        # THEN
        (command,) = _planned_commands(motion_group)
        assert tuple(command.path.target_pose.position) == (1005.0, 0.0, 0.0)

    async def test_unresolved_local_pose_fails_before_any_motion_naming_the_pose(self):
        # GIVEN
        motion_group = _motion_group(FakeUpstream())
        routine = _routine(move_linear(_pose(1.0)), move_linear("missing-pose"))

        # WHEN
        with pytest.raises(PoseResolutionError, match="missing-pose"):
            await motion_group.plan_and_execute_routine(routine, dataset=_fixture_dataset())

        # THEN
        motion_group._api_client.trajectory_planning_api.plan_trajectory.assert_not_awaited()
        motion_group._load_planned_motion.assert_not_awaited()

    async def test_local_pose_without_a_dataset_fails_naming_the_pose(self):
        # GIVEN
        motion_group = _motion_group(FakeUpstream())

        # WHEN
        with pytest.raises(PoseResolutionError, match="slot"):
            await motion_group.plan_and_execute_routine(_routine(move_linear("slot")))

        # THEN
        motion_group._api_client.trajectory_planning_api.plan_trajectory.assert_not_awaited()

    async def test_pose_on_a_rotated_nested_frame_is_resolved_to_world(self):
        # GIVEN
        # `base` is rotated 90 deg about z at x=1000, `child` is 10 mm along base's y,
        # and a pose is taught at x=100 on `child`
        dataset = _dataset(
            frames={
                "base": DatasetFrame(
                    frame="base", dataset="d", pose=Pose((1000, 0, 0, 0, 0, math.pi / 2))
                ),
                "child": DatasetFrame(
                    frame="child", dataset="d", reference_frame="base", pose=Pose((0, 10, 0))
                ),
            },
            poses={
                "p": DatasetPose(
                    dataset_pose="p", dataset="d", frame="child", pose=Pose((100, 0, 0))
                )
            },
        )
        motion_group = _motion_group(FakeUpstream())

        # WHEN
        async with asyncio.timeout(5):
            await motion_group.plan_and_execute_routine(_routine(move_linear("p")), dataset=dataset)

        # THEN
        # Worked out by hand:
        # child origin = (1000, 0, 0) + Rz(90) @ (0, 10, 0) = (990, 0, 0);
        # the pose adds Rz(90) @ (100, 0, 0) = (0, 100, 0)
        (command,) = _planned_commands(motion_group)
        target = command.path.target_pose
        assert tuple(target.position) == pytest.approx((990.0, 100.0, 0.0))
        assert tuple(target.orientation) == pytest.approx((0.0, 0.0, math.pi / 2))
