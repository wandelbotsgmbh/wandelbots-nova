"""Tests for payload resolution and per-call override in MotionGroup planning."""

from math import pi
from unittest.mock import AsyncMock, MagicMock

import pytest

from nova import Nova, api
from nova.actions import jnt
from nova.api import models
from nova.cell import virtual_controller
from nova.cell.motion_group import MotionGroup
from nova.core.gateway import ApiGateway


def _payload(name: str, mass: float) -> models.Payload:
    return models.Payload(name=name, payload=mass)


def _build_mock_motion_group(
    *, payloads: dict[str, models.Payload] | None, active_payload: str | None
) -> tuple[MotionGroup, MagicMock]:
    """Construct a MotionGroup with a mocked ApiGateway suitable for resolution tests.

    Returns the motion group and the mock ApiGateway so individual tests can inspect
    captured request payloads.
    """
    mock_api_client = MagicMock(spec=ApiGateway)

    # State (used by joints(), _fetch_state, active_payload_name)
    mock_state = MagicMock()
    mock_state.joint_position = [0.0, -1.57, -1.57, 0.0, 0.0, 0.0]
    mock_state.tcp_pose = models.Pose(
        position=models.Vector3d([0.0, 0.0, 0.0]),
        orientation=models.RotationVector([0.0, 0.0, 0.0]),
    )
    mock_state.tcp = None
    mock_state.payload = active_payload
    mock_api_client.motion_group_api = MagicMock()
    mock_api_client.motion_group_api.get_current_motion_group_state = AsyncMock(
        return_value=mock_state
    )

    # Description (used by get_setup, payloads(), tcps())
    mock_description = MagicMock()
    mock_description.motion_group_model = models.MotionGroupModel("test-model")
    mock_description.cycle_time = 8
    mock_description.mounting = None
    mock_description.tcps = None
    mock_description.payloads = payloads
    mock_description.operation_limits = MagicMock()
    mock_description.operation_limits.auto_limits = models.LimitSet(joints=[])
    mock_description.safety_tool_colliders = None
    mock_description.safety_link_colliders = None
    mock_description.safety_zones = None
    mock_api_client.motion_group_api.get_motion_group_description = AsyncMock(
        return_value=mock_description
    )

    # Trajectory planning (returns a trivial trajectory; we'll capture the request)
    mock_plan_response = MagicMock()
    mock_plan_response.response = models.JointTrajectory(
        joint_positions=[
            models.Joints([0.0, -1.57, -1.57, 0.0, 0.0, 0.0]),
            models.Joints([0.1, -1.47, -1.47, 0.1, 0.1, 0.1]),
        ],
        times=[0.0, 1.0],
        locations=[models.Location(0.0), models.Location(1.0)],
    )
    mock_api_client.trajectory_planning_api = MagicMock()
    mock_api_client.trajectory_planning_api.plan_trajectory = AsyncMock(
        return_value=mock_plan_response
    )

    motion_group = MotionGroup(
        api_client=mock_api_client,
        cell="test_cell",
        controller_id="test-controller",
        motion_group_id="0@test-controller",
    )
    return motion_group, mock_api_client


# ---------------------------------------------------------------------------
# get_setup payload resolution
# ---------------------------------------------------------------------------


class TestGetSetupPayloadResolution:
    """Verify the 5-step precedence documented in MotionGroup.get_setup."""

    @pytest.mark.asyncio
    async def test_no_payloads_registered_yields_none(self):
        mg, _ = _build_mock_motion_group(payloads=None, active_payload=None)
        setup = await mg.get_setup()
        assert setup.payload is None

    @pytest.mark.asyncio
    async def test_rule1_explicit_payload_object_wins(self):
        # Rule 1 wins over everything else.
        mg, _ = _build_mock_motion_group(
            payloads={"a": _payload("a", 1.0), "b": _payload("b", 2.0)}, active_payload="a"
        )
        custom = _payload("custom", 9.9)
        setup = await mg.get_setup(payload=custom)
        assert setup.payload is custom

    @pytest.mark.asyncio
    async def test_rule1_explicit_payload_name_resolves(self):
        mg, _ = _build_mock_motion_group(
            payloads={"a": _payload("a", 1.0), "b": _payload("b", 2.0)}, active_payload="a"
        )
        setup = await mg.get_setup(payload="b")
        assert setup.payload is not None
        assert setup.payload.name == "b"
        assert setup.payload.payload == 2.0

    @pytest.mark.asyncio
    async def test_rule1_explicit_unknown_name_raises_keyerror(self):
        mg, _ = _build_mock_motion_group(payloads={"a": _payload("a", 1.0)}, active_payload=None)
        with pytest.raises(KeyError):
            await mg.get_setup(payload="missing")

    @pytest.mark.asyncio
    async def test_rule2_tcp_name_match_beats_active_id(self):
        # Description has both "grip" (matching tcp_name) and "other" (matching active_payload).
        # Rule 2 must win over Rule 3.
        mg, _ = _build_mock_motion_group(
            payloads={"grip": _payload("grip", 1.5), "other": _payload("other", 2.5)},
            active_payload="other",
        )
        setup = await mg.get_setup(tcp_name="grip")
        assert setup.payload is not None
        assert setup.payload.name == "grip"

    @pytest.mark.asyncio
    async def test_rule3_active_id_used_when_no_tcp_match(self):
        mg, _ = _build_mock_motion_group(
            payloads={"a": _payload("a", 1.0), "b": _payload("b", 2.0)}, active_payload="b"
        )
        # tcp_name has no matching payload → rule 2 misses, rule 3 hits.
        setup = await mg.get_setup(tcp_name="some_tcp_without_payload")
        assert setup.payload is not None
        assert setup.payload.name == "b"

    @pytest.mark.asyncio
    async def test_rule4_single_payload_fallback(self):
        # Only one payload, no caller arg, no tcp match, no active id.
        only = _payload("only", 3.3)
        mg, _ = _build_mock_motion_group(payloads={"only": only}, active_payload=None)
        setup = await mg.get_setup()
        assert setup.payload is only

    @pytest.mark.asyncio
    async def test_rule5_multiple_payloads_no_signals_yields_none(self):
        mg, _ = _build_mock_motion_group(
            payloads={"a": _payload("a", 1.0), "b": _payload("b", 2.0)}, active_payload=None
        )
        setup = await mg.get_setup()
        assert setup.payload is None


# ---------------------------------------------------------------------------
# Helpers (payloads / payload_names / active_payload / active_payload_name)
# ---------------------------------------------------------------------------


class TestPayloadHelpers:
    @pytest.mark.asyncio
    async def test_payloads_returns_registered(self):
        a, b = _payload("a", 1.0), _payload("b", 2.0)
        mg, _ = _build_mock_motion_group(payloads={"a": a, "b": b}, active_payload=None)
        assert await mg.payloads() == {"a": a, "b": b}

    @pytest.mark.asyncio
    async def test_payloads_empty_when_none(self):
        mg, _ = _build_mock_motion_group(payloads=None, active_payload=None)
        assert await mg.payloads() == {}

    @pytest.mark.asyncio
    async def test_payload_names(self):
        mg, _ = _build_mock_motion_group(
            payloads={"a": _payload("a", 1.0), "b": _payload("b", 2.0)}, active_payload=None
        )
        assert sorted(await mg.payload_names()) == ["a", "b"]

    @pytest.mark.asyncio
    async def test_active_payload_name(self):
        mg, _ = _build_mock_motion_group(payloads=None, active_payload="heavy")
        assert await mg.active_payload_name() == "heavy"

    @pytest.mark.asyncio
    async def test_active_payload_resolves(self):
        heavy = _payload("heavy", 5.0)
        mg, _ = _build_mock_motion_group(
            payloads={"heavy": heavy, "light": _payload("light", 0.5)}, active_payload="heavy"
        )
        assert await mg.active_payload() is heavy

    @pytest.mark.asyncio
    async def test_active_payload_none_when_no_active(self):
        mg, _ = _build_mock_motion_group(payloads={"a": _payload("a", 1.0)}, active_payload=None)
        assert await mg.active_payload() is None


# ---------------------------------------------------------------------------
# plan() forwards payload to the planner
# ---------------------------------------------------------------------------


class TestPlanForwardsPayload:
    @pytest.mark.asyncio
    async def test_plan_uses_explicit_payload(self):
        mg, mock_client = _build_mock_motion_group(
            payloads={"a": _payload("a", 1.0)}, active_payload=None
        )
        custom = _payload("custom", 7.7)

        await mg.plan([jnt((0.0, 0.0, 0.0, 0.0, 0.0, 0.0))], payload=custom)

        plan_call = mock_client.trajectory_planning_api.plan_trajectory.await_args
        request = plan_call.kwargs["plan_trajectory_request"]
        assert request.motion_group_setup.payload == custom

    @pytest.mark.asyncio
    async def test_plan_resolves_by_name(self):
        a = _payload("a", 1.0)
        b = _payload("b", 2.0)
        mg, mock_client = _build_mock_motion_group(payloads={"a": a, "b": b}, active_payload=None)

        await mg.plan([jnt((0.0, 0.0, 0.0, 0.0, 0.0, 0.0))], payload="b")

        request = mock_client.trajectory_planning_api.plan_trajectory.await_args.kwargs[
            "plan_trajectory_request"
        ]
        assert request.motion_group_setup.payload is not None
        assert request.motion_group_setup.payload.name == "b"

    @pytest.mark.asyncio
    async def test_plan_overrides_supplied_motion_group_setup(self):
        # If the caller passes both a motion_group_setup and a payload, payload wins
        # without mutating the caller's setup.
        mg, mock_client = _build_mock_motion_group(
            payloads={"a": _payload("a", 1.0)}, active_payload=None
        )
        original_payload = _payload("from_setup", 0.1)
        supplied_setup = models.MotionGroupSetup(
            motion_group_model=models.MotionGroupModel("test-model"),
            cycle_time=8,
            payload=original_payload,
        )
        override = _payload("override", 4.4)

        await mg.plan(
            [jnt((0.0, 0.0, 0.0, 0.0, 0.0, 0.0))],
            motion_group_setup=supplied_setup,
            payload=override,
        )

        request = mock_client.trajectory_planning_api.plan_trajectory.await_args.kwargs[
            "plan_trajectory_request"
        ]
        assert request.motion_group_setup.payload == override
        # Caller's setup must not be mutated.
        assert supplied_setup.payload is original_payload


# ---------------------------------------------------------------------------
# Integration: planning the same path with two different payloads must yield
# different trajectories on a live virtual controller.
# ---------------------------------------------------------------------------


@pytest.fixture
async def ur_mg():
    """Virtual UR10e motion group at a known start position."""
    controller_name = "ur-payload-test"
    initial = [pi / 2, -pi / 2, pi / 2, 0, 0, 0]

    async with Nova() as nova:
        cell = nova.cell()
        await cell.ensure_controller(
            virtual_controller(
                name=controller_name,
                manufacturer=models.Manufacturer.UNIVERSALROBOTS,
                type="universalrobots-ur10e",
                position=[*initial, 0],
            )
        )
        ur = await cell.controller(controller_name)
        async with ur[0] as mg:
            yield mg


def _trajectory_signature(trajectory: models.JointTrajectory) -> tuple[float, tuple[float, ...]]:
    """Reduce a trajectory to a comparable signature: total duration and final joints."""
    duration = trajectory.times[-1] if trajectory.times else 0.0
    final = tuple(trajectory.joint_positions[-1].root) if trajectory.joint_positions else ()
    return duration, final


@pytest.mark.asyncio
@pytest.mark.integration
async def test_payload_changes_planned_trajectory(ur_mg):
    """Plan the same motion twice with light vs heavy payloads.

    The planner is dynamics-aware: a heavier payload (with offset CoM) should
    produce a measurably different trajectory (typically longer in time).
    Skips gracefully if the controller rejects arbitrary ad-hoc payloads.
    """
    initial = [pi / 2, -pi / 2, pi / 2, 0, 0, 0]
    target_joints = (pi / 4, -pi / 3, pi / 3, pi / 6, -pi / 6, pi / 4)

    light = api.models.Payload(name="light", payload=0.1)
    heavy = api.models.Payload(
        name="heavy", payload=10.0, center_of_mass=api.models.Vector3d([0.0, 0.0, 100.0])
    )

    try:
        light_traj = await ur_mg.plan(
            start_joint_position=tuple(initial),
            actions=[jnt(target_joints)],
            tcp="Flange",
            payload=light,
        )
        heavy_traj = await ur_mg.plan(
            start_joint_position=tuple(initial),
            actions=[jnt(target_joints)],
            tcp="Flange",
            payload=heavy,
        )
    except Exception as e:  # pragma: no cover - depends on backend behavior
        pytest.skip(f"Controller does not accept ad-hoc payloads: {e}")

    light_sig = _trajectory_signature(light_traj)
    heavy_sig = _trajectory_signature(heavy_traj)

    assert light_sig != heavy_sig, (
        f"Expected different trajectories for light vs heavy payload, "
        f"got identical signature {light_sig}"
    )
