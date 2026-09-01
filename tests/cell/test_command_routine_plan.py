"""Tests for CommandRoutine support in AbstractRobot.plan/execute/plan_and_execute.

`_resolve_actions_like` is exercised directly against a minimal `AbstractRobot` stub for the
tcp/start_joint_position/motion_group_setup defaulting and the motion-group mismatch check.
`MotionGroup.plan` is exercised end-to-end (with a mocked API client) to confirm a routine
converts into a request the real planning pipeline accepts.
"""

from typing import AsyncGenerator
from unittest.mock import AsyncMock, MagicMock

import pytest

from nova import api
from nova.actions import Action, MovementController
from nova.actions.motions import JointPTP
from nova.cell.motion_group import MotionGroup
from nova.cell.robot_cell import AbstractRobot
from nova.command_routines import (
    UnsupportedCommandRoutineFeature,
    command_routine,
    move_joint_ptp,
    move_linear,
)
from nova.command_routines.commands import motion, path_line
from nova.core.gateway import ApiGateway
from nova.types import MotionState, Pose, RobotState


class _FakeRobot(AbstractRobot):
    def __init__(self, id: str = "0@controller"):
        super().__init__(id=id)
        self.plan_calls: list[dict] = []

    async def _plan(
        self,
        actions: list[Action],
        tcp: str | None = None,
        start_joint_position: tuple[float, ...] | None = None,
        motion_group_setup: api.models.MotionGroupSetup | None = None,
        payload_override=None,
        singularity_handling=None,
    ) -> api.models.JointTrajectory:
        self.plan_calls.append(
            {
                "actions": actions,
                "tcp": tcp,
                "start_joint_position": start_joint_position,
                "motion_group_setup": motion_group_setup,
            }
        )
        return api.models.JointTrajectory(joint_positions=[[0.0] * 6], times=[0.0], locations=[0.0])

    def _execute(
        self,
        joint_trajectory: api.models.JointTrajectory,
        tcp: str | None,
        actions: list[Action],
        movement_controller: MovementController | None,
        start_on_io=None,
        pause_on_io=None,
    ) -> AsyncGenerator[MotionState, None]:
        raise NotImplementedError

    async def get_state(self, tcp: str | None = None) -> RobotState:
        raise NotImplementedError

    async def joints(self) -> tuple:
        raise NotImplementedError

    async def tcp_pose(self, tcp: str | None = None) -> Pose:
        raise NotImplementedError

    async def tcps(self) -> dict:
        raise NotImplementedError

    async def tcp_names(self) -> list[str]:
        raise NotImplementedError

    async def active_tcp(self):
        raise NotImplementedError

    async def active_tcp_name(self) -> str | None:
        raise NotImplementedError

    async def stop(self):
        raise NotImplementedError


class TestResolveActionsLikeDefaults:
    @pytest.mark.asyncio
    async def test_plan_uses_routine_tcp_and_start_joint_position(self):
        robot = _FakeRobot()
        routine = command_routine(
            "routine",
            tcp="flange",
            start_joint_position=[0.0] * 6,
            commands=[move_joint_ptp([0.1] * 6)],
        )

        await robot.plan(routine)

        [call] = robot.plan_calls
        assert call["tcp"] == "flange"
        assert call["start_joint_position"] == (0.0,) * 6
        assert isinstance(call["actions"][0], JointPTP)

    @pytest.mark.asyncio
    async def test_explicit_tcp_overrides_routine_tcp(self):
        robot = _FakeRobot()
        routine = command_routine("routine", tcp="flange", commands=[move_joint_ptp([0.1] * 6)])

        await robot.plan(routine, tcp="other")

        assert robot.plan_calls[0]["tcp"] == "other"

    @pytest.mark.asyncio
    async def test_explicit_start_joint_position_overrides_routine(self):
        robot = _FakeRobot()
        routine = command_routine(
            "routine", start_joint_position=[0.0] * 6, commands=[move_joint_ptp([0.1] * 6)]
        )

        await robot.plan(routine, start_joint_position=(1.0,) * 6)

        assert robot.plan_calls[0]["start_joint_position"] == (1.0,) * 6

    @pytest.mark.asyncio
    async def test_routine_motion_group_setup_used_when_not_given(self):
        robot = _FakeRobot()
        setup = api.models.MotionGroupSetup(motion_group_model="test-model", cycle_time=8)
        routine = command_routine(
            "routine", motion_group_setup=setup, commands=[move_joint_ptp([0.1] * 6)]
        )

        await robot.plan(routine)

        assert robot.plan_calls[0]["motion_group_setup"] == setup


class TestMotionGroupReferenceValidation:
    @pytest.mark.asyncio
    async def test_mismatched_motion_group_raises(self):
        robot = _FakeRobot(id="0@controller")
        routine = command_routine(
            "routine", motion_group="1@controller", commands=[move_joint_ptp([0.1] * 6)]
        )

        with pytest.raises(ValueError, match="targets motion group"):
            await robot.plan(routine)

    @pytest.mark.asyncio
    async def test_matching_motion_group_is_accepted(self):
        robot = _FakeRobot(id="0@controller")
        routine = command_routine(
            "routine", motion_group="0@controller", commands=[move_joint_ptp([0.1] * 6)]
        )

        await robot.plan(routine)

        assert len(robot.plan_calls) == 1


class TestPoseResolverPassthrough:
    @pytest.mark.asyncio
    async def test_plan_forwards_pose_resolver_to_resolve_local_pose_references(self):
        robot = _FakeRobot()
        routine = command_routine("routine", commands=[move_linear("approach")])

        await robot.plan(
            routine, tcp="flange", pose_resolver={"approach": Pose((1, 2, 3, 4, 5, 6))}
        )

        [call] = robot.plan_calls
        assert call["actions"][0].target == Pose((1, 2, 3, 4, 5, 6))

    @pytest.mark.asyncio
    async def test_plan_without_pose_resolver_raises_for_local_pose_reference(self):
        robot = _FakeRobot()
        routine = command_routine("routine", commands=[move_linear("approach")])

        with pytest.raises(UnsupportedCommandRoutineFeature, match="LocalPoseReference"):
            await robot.plan(routine, tcp="flange")


@pytest.fixture
def mock_motion_group():
    """A MotionGroup with mocked API client internals, mirroring test_optional_tcp.py."""
    mock_api_client = MagicMock(spec=ApiGateway)

    mock_state = MagicMock()
    mock_state.joint_position = [0.0, -1.57, -1.57, 0.0, 0.0, 0.0]
    mock_state.tcp_pose = api.models.Pose(position=[0.0, 0.0, 0.0], orientation=[0.0, 0.0, 0.0])
    mock_state.tcp = None
    mock_api_client.motion_group_api = MagicMock()
    mock_api_client.motion_group_api.get_current_motion_group_state = AsyncMock(
        return_value=mock_state
    )

    mock_description = MagicMock()
    mock_description.motion_group_model = "test-model"
    mock_description.cycle_time = 8
    mock_description.mounting = None
    mock_description.tcps = {
        "flange": MagicMock(
            pose=api.models.Pose(position=[0, 0, 0], orientation=[0, 0, 0]), name="flange"
        )
    }
    mock_description.operation_limits = MagicMock()
    mock_description.operation_limits.auto_limits = api.models.LimitSet(joints=[])
    mock_description.safety_tool_colliders = None
    mock_description.safety_link_colliders = None
    mock_description.safety_zones = None
    mock_api_client.motion_group_api.get_motion_group_description = AsyncMock(
        return_value=mock_description
    )

    mock_plan_response = MagicMock()
    mock_plan_response.response = api.models.JointTrajectory(
        joint_positions=[[0.0, -1.57, -1.57, 0.0, 0.0, 0.0], [0.1, -1.47, -1.47, 0.1, 0.1, 0.1]],
        times=[0.0, 1.0],
        locations=[0.0, 1.0],
    )
    mock_api_client.trajectory_planning_api = MagicMock()
    mock_api_client.trajectory_planning_api.plan_trajectory = AsyncMock(
        return_value=mock_plan_response
    )

    return MotionGroup(
        api_client=mock_api_client,
        cell="test_cell",
        controller_id="test-controller",
        motion_group_id="0@test-controller",
    )


class TestMotionGroupPlanWithCommandRoutine:
    @pytest.mark.asyncio
    async def test_plan_joint_routine_without_tcp(self, mock_motion_group):
        routine = command_routine(
            "routine",
            start_joint_position=[0.0, -1.57, -1.57, 0.0, 0.0, 0.0],
            commands=[move_joint_ptp([0.1, -1.47, -1.47, 0.1, 0.1, 0.1])],
        )

        trajectory = await mock_motion_group.plan(routine)

        assert trajectory is not None
        assert len(trajectory.joint_positions) > 0

    @pytest.mark.asyncio
    async def test_plan_cartesian_routine_uses_routine_tcp(self, mock_motion_group):
        """A cartesian motion needs a TCP; omitting it on `plan()` must not raise because
        the routine's own `tcp` is used."""
        target = api.models.InlinePoseReference(
            pose=api.models.Pose(position=(1, 2, 3), orientation=(0, 0, 0))
        )
        routine = command_routine(
            "routine",
            tcp="flange",
            start_joint_position=[0.0, -1.57, -1.57, 0.0, 0.0, 0.0],
            commands=[motion(target, path_line())],
        )

        # Succeeding at all (a cartesian action with tcp=None would raise, as the next test
        # shows) proves the routine's own `tcp` was picked up as the default.
        trajectory = await mock_motion_group.plan(routine)

        assert trajectory is not None

    @pytest.mark.asyncio
    async def test_plan_cartesian_routine_without_tcp_raises(self, mock_motion_group):
        """Without a routine-level `tcp` (and none passed explicitly), the existing
        TCP-required validation for cartesian actions still applies."""
        target = api.models.InlinePoseReference(
            pose=api.models.Pose(position=(1, 2, 3), orientation=(0, 0, 0))
        )
        routine = command_routine(
            "routine",
            start_joint_position=[0.0, -1.57, -1.57, 0.0, 0.0, 0.0],
            commands=[motion(target, path_line())],
        )

        with pytest.raises(ValueError, match="TCP is required"):
            await mock_motion_group.plan(routine)
