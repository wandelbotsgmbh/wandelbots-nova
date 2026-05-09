"""Tests for jog_joints() and jog_tcp() joggers."""

from __future__ import annotations

from unittest.mock import AsyncMock, MagicMock

import pytest

from policy.jogger import JointJogger, TcpJogger, jog_joints, jog_tcp
from policy.types import EmergencyStopError, GuardStopError, MotionError, PidConfig

# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------


def _mock_mg(mg_id: str = "0@ur10e", num_joints: int = 6) -> MagicMock:
    """Create a mock MotionGroup."""
    mg = MagicMock()
    mg.id = mg_id
    # For get_controller_id
    mg._controller_id = mg_id.split("@")[1] if "@" in mg_id else mg_id
    mg._cell = "cell"
    mg._api_client = MagicMock()
    return mg


def _mock_session(mg_id: str = "0@ur10e", num_joints: int = 6) -> MagicMock:
    """Create a mock PidJoggingSession."""
    session = MagicMock()
    session.motion_group_id = mg_id
    session._num_joints = num_joints
    session.has_failed = False
    session.failure_reason = ""
    session.current_state = MagicMock()
    session.current_state.joints = tuple([0.0] * num_joints)
    session.start = AsyncMock()
    session.stop = AsyncMock()
    session.update_chunk = MagicMock()
    return session


# ---------------------------------------------------------------------------
# Factory function tests
# ---------------------------------------------------------------------------


class TestJogJointsFactory:
    def test_single_mg(self):
        mg = _mock_mg()
        jogger = jog_joints(mg)
        assert isinstance(jogger, JointJogger)
        assert not jogger._multi

    def test_multi_mg(self):
        mg1, mg2 = _mock_mg("0@ur10e"), _mock_mg("0@ur10e-2")
        jogger = jog_joints([mg1, mg2])
        assert isinstance(jogger, JointJogger)
        assert jogger._multi

    def test_custom_config(self):
        mg = _mock_mg()
        cfg = PidConfig(p_gain=5.0)
        jogger = jog_joints(mg, config=cfg)
        assert jogger._sessions[mg]._config.p_gain == 5.0


class TestJogTcpFactory:
    def test_single_mg_with_tcp(self):
        mg = _mock_mg()
        jogger = jog_tcp(mg, tcp="Flange")
        assert isinstance(jogger, TcpJogger)
        assert not jogger._multi

    def test_multi_mg_dict(self):
        mg1, mg2 = _mock_mg("0@ur10e"), _mock_mg("0@ur10e-2")
        jogger = jog_tcp({mg1: "Flange", mg2: "Tool"})
        assert isinstance(jogger, TcpJogger)
        assert jogger._multi

    def test_tcp_default_uses_cartesian_limits(self):
        mg = _mock_mg()
        jogger = jog_tcp(mg, tcp="Flange")
        session = jogger._sessions[mg]
        # PID should have per-axis limits from tcp_velocity_limit / tcp_orientation_velocity_limit
        assert isinstance(session._pid.velocity_limit, list)
        assert len(session._pid.velocity_limit) == 6
        assert session._pid.velocity_limit[0] == 250.0  # translation
        assert session._pid.velocity_limit[3] == 1.5    # rotation


# ---------------------------------------------------------------------------
# Target setter — JointJogger
# ---------------------------------------------------------------------------


class TestJointJoggerTarget:
    def _make(self, *mg_ids: str, num_joints: int = 6) -> tuple[JointJogger, list[MagicMock]]:
        mgs = [_mock_mg(mid) for mid in mg_ids]
        jogger = JointJogger.__new__(JointJogger)
        jogger._mg_list = mgs
        jogger._multi = len(mgs) > 1
        jogger._sessions = {mg: _mock_session(mg.id, num_joints) for mg in mgs}
        jogger._estop = None
        jogger._target = None
        return jogger, mgs

    def test_single_list(self):
        jogger, (mg,) = self._make("0@ur10e")
        jogger.target = [1.0, 2.0, 3.0, 4.0, 5.0, 6.0]
        assert jogger.target == [1.0, 2.0, 3.0, 4.0, 5.0, 6.0]
        jogger._sessions[mg].update_chunk.assert_called_once()

    def test_multi_dict(self):
        jogger, (mg1, mg2) = self._make("0@ur10e", "0@ur10e-2")
        t1 = [1.0] * 6
        t2 = [2.0] * 6
        jogger.target = {mg1: t1, mg2: t2}
        assert jogger.target == {mg1: t1, mg2: t2}

    def test_single_rejects_dict_error(self):
        jogger, _ = self._make("0@ur10e")
        with pytest.raises(TypeError, match="Expected list"):
            jogger.target = "not a list"

    def test_multi_rejects_list(self):
        jogger, _ = self._make("0@ur10e", "0@ur10e-2")
        with pytest.raises(TypeError, match="multiple motion groups"):
            jogger.target = [1.0] * 6

    def test_wrong_dimensions(self):
        jogger, _ = self._make("0@ur10e")
        with pytest.raises(ValueError, match="expects 6"):
            jogger.target = [1.0, 2.0, 3.0]

    def test_none_clears_target(self):
        jogger, _ = self._make("0@ur10e")
        jogger.target = [1.0] * 6
        jogger.target = None
        assert jogger.target is None


# ---------------------------------------------------------------------------
# Target setter — TcpJogger
# ---------------------------------------------------------------------------


class TestTcpJoggerTarget:
    def _make_pose(self, *values: float) -> MagicMock:
        """Create a mock Pose."""
        pose = MagicMock()
        pose.position = list(values[:3]) if len(values) >= 3 else [0, 0, 0]
        pose.orientation = list(values[3:6]) if len(values) >= 6 else [0, 0, 0]
        return pose

    def _make(self, *mg_ids: str) -> tuple[TcpJogger, list[MagicMock]]:
        mgs = [_mock_mg(mid) for mid in mg_ids]
        jogger = TcpJogger.__new__(TcpJogger)
        jogger._mg_list = mgs
        jogger._multi = len(mgs) > 1
        jogger._sessions = {mg: _mock_session(mg.id) for mg in mgs}
        jogger._estop = None
        jogger._target = None
        return jogger, mgs

    def test_single_pose_via_validate(self):
        jogger, (mg,) = self._make("0@ur10e")
        pose = self._make_pose(100, 200, 300, 0.1, 0.2, 0.3)
        jogger._validate_and_push(mg, list(pose.position) + list(pose.orientation))
        jogger._sessions[mg].update_chunk.assert_called_once()

    def test_rejects_wrong_type(self):
        jogger, _ = self._make("0@ur10e")
        with pytest.raises(TypeError, match="Expected Pose"):
            jogger.target = "not a pose"

    def test_none_clears(self):
        jogger, _ = self._make("0@ur10e")
        jogger.target = None
        assert jogger.target is None


# ---------------------------------------------------------------------------
# State reading
# ---------------------------------------------------------------------------


class TestState:
    def test_single_returns_robot_state(self):
        jogger, (mg,) = TestJointJoggerTarget()._make("0@ur10e")
        state = jogger.state()
        assert state is not None
        assert state == jogger._sessions[mg].current_state

    def test_multi_returns_dict(self):
        jogger, (mg1, mg2) = TestJointJoggerTarget()._make("0@ur10e", "0@ur10e-2")
        states = jogger.state()
        assert isinstance(states, dict)
        assert mg1 in states
        assert mg2 in states

    def test_none_when_no_state(self):
        jogger, (mg,) = TestJointJoggerTarget()._make("0@ur10e")
        jogger._sessions[mg].current_state = None
        assert jogger.state() is None


# ---------------------------------------------------------------------------
# Error propagation
# ---------------------------------------------------------------------------


class TestErrorPropagation:
    @pytest.mark.asyncio
    async def test_session_failure_raises_motion_error(self):
        jogger, (mg,) = TestJointJoggerTarget()._make("0@ur10e")
        jogger._sessions[mg].has_failed = True
        jogger._sessions[mg].failure_reason = "Motion error on '0@ur10e': joint_limit"

        with pytest.raises(MotionError):
            async for _ in jogger:
                break

    @pytest.mark.asyncio
    async def test_session_failure_raises_guard_stop(self):
        jogger, (mg,) = TestJointJoggerTarget()._make("0@ur10e")
        jogger._sessions[mg].has_failed = True
        jogger._sessions[mg].failure_reason = "Safety guard 'workspace_guard' triggered"

        with pytest.raises(GuardStopError):
            async for _ in jogger:
                break

    @pytest.mark.asyncio
    async def test_session_failure_raises_runtime_error(self):
        jogger, (mg,) = TestJointJoggerTarget()._make("0@ur10e")
        jogger._sessions[mg].has_failed = True
        jogger._sessions[mg].failure_reason = "connection reset"

        with pytest.raises(RuntimeError, match="connection reset"):
            async for _ in jogger:
                break

    @pytest.mark.asyncio
    async def test_estop_raises(self):
        jogger, _ = TestJointJoggerTarget()._make("0@ur10e")
        jogger._estop = MagicMock()
        jogger._estop.error = EmergencyStopError("ur10e", "SAFETY_STATE_EMERGENCY_STOP")

        with pytest.raises(EmergencyStopError):
            async for _ in jogger:
                break


# ---------------------------------------------------------------------------
# Safety guards
# ---------------------------------------------------------------------------


class TestSafetyGuards:
    def test_guards_passed_to_sessions(self):
        mg = _mock_mg()

        def my_guard(ctx):
            return True

        jogger = jog_joints(mg, safety_guards=[my_guard])
        session = jogger._sessions[mg]
        assert my_guard in session._safety_guards

    def test_tcp_guards_passed_to_sessions(self):
        mg = _mock_mg()

        def my_guard(ctx):
            return True

        jogger = jog_tcp(mg, tcp="Flange", safety_guards=[my_guard])
        session = jogger._sessions[mg]
        assert my_guard in session._safety_guards


# ---------------------------------------------------------------------------
# Velocity controller per-axis limits
# ---------------------------------------------------------------------------


class TestPerAxisVelocityLimit:
    def test_float_limit(self):
        from policy.velocity_controller import VelocityController

        vc = VelocityController(velocity_limit=1.0, p_gain=10.0, tolerance=0.001)
        vel = vc.compute([0.0, 0.0], [1.0, 1.0])
        assert all(abs(v) <= 1.0 for v in vel)

    def test_list_limit(self):
        from policy.velocity_controller import VelocityController

        vc = VelocityController(velocity_limit=[1.0, 100.0], p_gain=10.0, tolerance=0.001)
        vel = vc.compute([0.0, 0.0], [1.0, 1.0])
        # First axis clamped to 1.0, second can go up to 100.0
        assert abs(vel[0]) <= 1.0
        assert abs(vel[1]) <= 100.0

    def test_tcp_velocity_limits_on_jogger(self):
        """TCP velocity limits are set on the jogger, not on PidConfig."""
        mg = _mock_mg()
        jogger = jog_tcp(
            mg, tcp="Flange",
            tcp_velocity_limit=500.0,
            tcp_orientation_velocity_limit=2.0,
        )
        session = jogger._sessions[mg]
        # The session's PID should have per-axis limits
        limits = session._pid.velocity_limit
        assert isinstance(limits, list)
        assert limits == [500.0, 500.0, 500.0, 2.0, 2.0, 2.0]
