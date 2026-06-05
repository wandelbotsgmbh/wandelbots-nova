"""Tests for jog_joints() and jog_tcp() joggers."""

from __future__ import annotations

from unittest.mock import AsyncMock, MagicMock

import pytest

from policy.jogging import JointJogger
from policy.types import EmergencyStopError, MotionError


def _mock_mg(mg_id: str = "0@ur10e", num_joints: int = 6) -> MagicMock:
    mg = MagicMock()
    mg.id = mg_id
    mg._controller_id = mg_id.split("@")[1] if "@" in mg_id else mg_id
    mg._cell = "cell"
    mg._api_client = MagicMock()
    return mg


def _mock_session(mg_id: str = "0@ur10e", num_joints: int = 6) -> MagicMock:
    session = MagicMock()
    session.motion_group_id = mg_id
    session.num_joints = num_joints
    session.mode = "joint"
    session.has_failed = False
    session.failure_reason = ""
    session.failure_exception = None
    session.stop_condition_triggered = None
    session.current_state = MagicMock()
    session.current_state.joints = tuple([0.0] * num_joints)
    session.session_elapsed_ms = 0
    session.start = AsyncMock()
    session.stop = AsyncMock()
    session.update_chunk = MagicMock()
    return session


# ---------------------------------------------------------------------------
# Target validation and dispatch
# ---------------------------------------------------------------------------


class TestJointJoggerTarget:
    def _make(self, *mg_ids: str, num_joints: int = 6) -> tuple[JointJogger, list[MagicMock]]:
        mgs = [_mock_mg(mid) for mid in mg_ids]
        jogger = JointJogger.__new__(JointJogger)
        jogger._mg_list = mgs
        jogger._multi = len(mgs) > 1
        jogger._sessions = {mg: _mock_session(mg.id, num_joints) for mg in mgs}
        jogger._estop = None
        jogger._rerun = None
        jogger._target = None
        return jogger, mgs

    def test_single_target_dispatches(self):
        jogger, (mg,) = self._make("0@ur10e")
        jogger.set_target([1.0, 2.0, 3.0, 4.0, 5.0, 6.0])
        assert jogger.target == [1.0, 2.0, 3.0, 4.0, 5.0, 6.0]
        jogger._sessions[mg].update_chunk.assert_called_once()

    def test_multi_target_dispatches(self):
        jogger, (mg1, mg2) = self._make("0@ur10e", "0@ur10e-2")
        t1, t2 = [1.0] * 6, [2.0] * 6
        jogger.set_target({mg1: t1, mg2: t2})
        assert jogger.target == {mg1: t1, mg2: t2}
        jogger._sessions[mg1].update_chunk.assert_called_once()
        jogger._sessions[mg2].update_chunk.assert_called_once()

    def test_wrong_dimensions_rejected(self):
        jogger, _ = self._make("0@ur10e")
        with pytest.raises(ValueError, match="expects 6"):
            jogger.set_target([1.0, 2.0, 3.0])

    def test_chunk_uses_last_step_as_target(self):
        jogger, (mg,) = self._make("0@ur10e")
        chunk = [[float(i)] * 6 for i in range(4)]
        jogger.set_target(chunk, dt_ms=33.0)
        jogger._sessions[mg].update_chunk.assert_called_with(
            steps=chunk, dt_ms=33.0, start_time_ms=0
        )
        assert jogger.target == chunk[-1]

    def test_multi_chunk(self):
        jogger, (mg1, mg2) = self._make("0@ur10e", "0@ur10e-2")
        c1 = [[float(i)] * 6 for i in range(4)]
        c2 = [[float(i + 10)] * 6 for i in range(4)]
        jogger.set_target({mg1: c1, mg2: c2}, dt_ms=33.0)
        jogger._sessions[mg1].update_chunk.assert_called_with(steps=c1, dt_ms=33.0, start_time_ms=0)
        jogger._sessions[mg2].update_chunk.assert_called_with(steps=c2, dt_ms=33.0, start_time_ms=0)


# ---------------------------------------------------------------------------
# Error propagation — different failure reasons map to different exceptions
# ---------------------------------------------------------------------------


class TestErrorPropagation:
    @pytest.mark.asyncio
    async def test_motion_error(self):
        jogger, (mg,) = TestJointJoggerTarget()._make("0@ur10e")
        jogger._sessions[mg].has_failed = True
        jogger._sessions[mg].has_failed = True
        jogger._sessions[mg].failure_exception = MotionError("0@ur10e", "joint_limit")
        with pytest.raises(MotionError):
            async for _ in jogger:
                break

    @pytest.mark.asyncio
    async def test_stop_condition(self):
        """A fired stop condition ends the loop normally and is reported (no raise)."""
        jogger, (mg,) = TestJointJoggerTarget()._make("0@ur10e")
        jogger._sessions[mg].stop_condition_triggered = "workspace_stop"
        iterations = 0
        async for _ in jogger:
            iterations += 1
            break
        assert iterations == 0  # loop ended before yielding
        assert jogger.stop_condition_triggered == "workspace_stop"

    @pytest.mark.asyncio
    async def test_connection_error(self):
        jogger, (mg,) = TestJointJoggerTarget()._make("0@ur10e")
        jogger._sessions[mg].has_failed = True
        jogger._sessions[mg].has_failed = True
        jogger._sessions[mg].failure_exception = RuntimeError("connection reset")
        with pytest.raises(RuntimeError, match="connection reset"):
            async for _ in jogger:
                break

    @pytest.mark.asyncio
    async def test_estop(self):
        jogger, _ = TestJointJoggerTarget()._make("0@ur10e")
        jogger._estop = MagicMock()
        jogger._estop.error = EmergencyStopError("ur10e", "SAFETY_STATE_EMERGENCY_STOP")
        with pytest.raises(EmergencyStopError):
            async for _ in jogger:
                break
