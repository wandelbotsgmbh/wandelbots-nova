"""Tests for e-stop detection.

These mock the controller-state stream at the SDK boundary and assert the
observable behaviour: ``EstopMonitor`` raises (via its ``error``) on a
non-operational safety state and stays clear on operational ones, and the
``check_*`` helpers re-raise faults. The live counterpart lives in
``integration/test_estop_live.py``.
"""

from __future__ import annotations

import asyncio
import types
from unittest.mock import MagicMock, patch

import pytest

from nova import api
from novapolicy.estop import EstopMonitor, check_estop, check_sessions
from novapolicy.types import EmergencyStopError, MotionError

_SafetyState = api.models.SafetyStateType


# ---------------------------------------------------------------------------
# check_estop / check_sessions — faults re-raise, healthy state is a no-op
# ---------------------------------------------------------------------------


def test_check_estop_raises_when_error_set():
    monitor = MagicMock()
    monitor.error = EmergencyStopError("ur10e", "SAFETY_STATE_PROTECTIVE_STOP")
    with pytest.raises(EmergencyStopError):
        check_estop(monitor)


def test_check_estop_noop_when_clear():
    monitor = MagicMock()
    monitor.error = None
    check_estop(monitor)  # no raise
    check_estop(None)  # no raise


def test_check_sessions_reraises_failure_exception():
    session = MagicMock()
    session.has_failed = True
    session.failure_exception = MotionError("ur10e", "joint_limit")
    with pytest.raises(MotionError):
        check_sessions({"0@ur10e": session})


def test_check_sessions_ignores_healthy_sessions():
    session = MagicMock()
    session.has_failed = False
    check_sessions({"0@ur10e": session})  # no raise


# ---------------------------------------------------------------------------
# EstopMonitor against a mocked controller-state stream
# ---------------------------------------------------------------------------


class _FakeStream:
    """Async iterator over canned states; blocks once exhausted (like a live stream)."""

    def __init__(self, states: list[object]) -> None:
        self._states = list(states)

    def __aiter__(self) -> _FakeStream:
        return self

    async def __anext__(self) -> object:
        if self._states:
            return self._states.pop(0)
        await asyncio.sleep(3600)  # stay open until cancelled
        raise StopAsyncIteration

    async def aclose(self) -> None:
        pass


def _mg(controller_id: str = "ur10e") -> MagicMock:
    mg = MagicMock()
    mg.id = f"0@{controller_id}"
    return mg


def _patched_monitor(states: list[object]) -> EstopMonitor:
    """EstopMonitor whose stream yields the given states."""
    gateway = MagicMock()
    gateway.controller_api.stream_robot_controller_state = lambda **_kw: _FakeStream(states)
    monitor = EstopMonitor([_mg()])
    patcher = patch.multiple(
        "novapolicy.estop",
        get_api_gateway=lambda _mg: gateway,
        get_cell=lambda _mg: "cell",
        get_controller_id=lambda mg: mg.id.split("@")[1],
    )
    patcher.start()
    monitor._patcher = patcher
    return monitor


async def _run_monitor_until(monitor: EstopMonitor, *, timeout_s: float = 1.0) -> None:
    await monitor.start()
    deadline = asyncio.get_event_loop().time() + timeout_s
    while monitor.error is None and asyncio.get_event_loop().time() < deadline:
        await asyncio.sleep(0.01)
    await monitor.stop()
    monitor._patcher.stop()  # type: ignore[attr-defined]


@pytest.mark.asyncio
async def test_monitor_fires_on_non_operational_state():
    """A protective stop in the state stream sets an EmergencyStopError."""
    state = types.SimpleNamespace(safety_state=_SafetyState.SAFETY_STATE_PROTECTIVE_STOP)
    monitor = _patched_monitor([state])
    await _run_monitor_until(monitor)
    assert isinstance(monitor.error, EmergencyStopError)
    assert monitor.error.safety_state == "SAFETY_STATE_PROTECTIVE_STOP"


@pytest.mark.asyncio
async def test_monitor_stays_clear_on_operational_states():
    """NORMAL and REDUCED are operational — the monitor never fires."""
    states = [
        types.SimpleNamespace(safety_state=_SafetyState.SAFETY_STATE_NORMAL),
        types.SimpleNamespace(safety_state=_SafetyState.SAFETY_STATE_REDUCED),
    ]
    monitor = _patched_monitor(states)
    await _run_monitor_until(monitor, timeout_s=0.2)
    assert monitor.error is None
