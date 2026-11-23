"""Tests for e-stop detection.

These mock the controller-state stream at the SDK boundary and assert the
observable behaviour: ``EstopMonitor`` raises (via its ``error``) on a
non-operational safety state and stays clear on operational ones, and the
session lifecycle's ``check`` re-raises faults. The live counterpart lives in
``integration/test_estop_live.py``.
"""

from __future__ import annotations

import asyncio
import logging
import types
from unittest.mock import AsyncMock, MagicMock, patch

from pydantic import ValidationError
import pytest

from nova import api
from novapolicy.estop import EstopMonitor
from novapolicy.jogging.lifecycle import SessionLifecycle
from novapolicy.tests.doubles import mg as _mg
from novapolicy.types import EmergencyStopError, MotionError, WaypointConfig

_SafetyState = api.models.SafetyStateType


# ---------------------------------------------------------------------------
# SessionLifecycle.check — faults re-raise, healthy state is a no-op
# ---------------------------------------------------------------------------


def _lifecycle(session: MagicMock | None = None) -> SessionLifecycle:
    lifecycle = SessionLifecycle(config=WaypointConfig())
    if session is not None:
        lifecycle.sessions["0@ur10e"] = session
    return lifecycle


async def _monitored(error: EmergencyStopError | None) -> SessionLifecycle:
    """A lifecycle whose e-stop monitor reports ``error``."""
    lifecycle = _lifecycle()
    monitor = MagicMock(start=AsyncMock(), error=error)
    with (
        patch("novapolicy.jogging.lifecycle.EstopMonitor", return_value=monitor),
        patch("novapolicy.rerun._is_rerun_active", return_value=False),
    ):
        await lifecycle.start_monitoring()
    return lifecycle


@pytest.mark.asyncio
async def test_check_raises_when_the_estop_monitor_has_an_error():
    lifecycle = await _monitored(EmergencyStopError("ur10e", "SAFETY_STATE_PROTECTIVE_STOP"))
    with pytest.raises(EmergencyStopError):
        lifecycle.check()


@pytest.mark.asyncio
async def test_check_is_a_noop_while_the_estop_monitor_is_clear():
    assert (await _monitored(None)).check() is None
    assert _lifecycle().check() is None  # no monitor started yet


def test_check_reraises_a_session_failure():
    session = MagicMock()
    session.has_failed = True
    session.failure_exception = MotionError("ur10e", "joint_limit")
    with pytest.raises(MotionError):
        _lifecycle(session).check()


def test_check_ignores_healthy_sessions():
    session = MagicMock()
    session.has_failed = False
    session.stop_condition_triggered = None
    assert _lifecycle(session).check() is None


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


class _UnparseableThenStates(_FakeStream):
    """First stream raises a ValidationError (as the SDK does on an unknown enum value)."""

    calls = 0

    async def __anext__(self) -> object:
        _UnparseableThenStates.calls += 1
        if _UnparseableThenStates.calls == 1:
            raise _validation_error()
        return await super().__anext__()


def _validation_error() -> Exception:
    try:
        api.models.RobotControllerState.model_validate({"mode": "MODE_UNDEFINED"})
    except ValidationError as e:
        return e
    raise AssertionError("expected a ValidationError")


@pytest.mark.asyncio
async def test_monitor_keeps_watching_after_an_unparseable_state():
    """Regression: an unknown mode (MODE_UNDEFINED) once switched the monitor off for the run."""
    _UnparseableThenStates.calls = 0
    estop = types.SimpleNamespace(safety_state=_SafetyState.SAFETY_STATE_ROBOT_EMERGENCY_STOP)
    gateway = MagicMock()
    gateway.controller_api.stream_robot_controller_state = lambda **_kw: _UnparseableThenStates([
        estop
    ])
    monitor = _patched_monitor([])
    with patch("novapolicy.estop.get_api_gateway", lambda _mg: gateway):
        await _run_monitor_until(monitor)
    assert isinstance(monitor.error, EmergencyStopError)
    assert monitor.error.safety_state == "SAFETY_STATE_ROBOT_EMERGENCY_STOP"


class _UnparseableTimesThenStates(_FakeStream):
    unparseable = 3

    async def __anext__(self) -> object:
        if _UnparseableTimesThenStates.unparseable > 0:
            _UnparseableTimesThenStates.unparseable -= 1
            raise _validation_error()
        return await super().__anext__()


@pytest.mark.asyncio
async def test_a_streak_of_unparseable_states_is_logged_once_and_its_end_too(caplog):
    _UnparseableTimesThenStates.unparseable = 3
    estop = types.SimpleNamespace(safety_state=_SafetyState.SAFETY_STATE_ROBOT_EMERGENCY_STOP)
    gateway = MagicMock()
    gateway.controller_api.stream_robot_controller_state = lambda **_kw: (
        _UnparseableTimesThenStates([estop])
    )
    monitor = _patched_monitor([])
    with (
        caplog.at_level(logging.INFO),
        patch("novapolicy.estop.get_api_gateway", lambda _mg: gateway),
    ):
        await _run_monitor_until(monitor)
    assert isinstance(monitor.error, EmergencyStopError)
    assert sum("unparseable controller state" in r.message for r in caplog.records) == 1
    assert sum("reads controller states again" in r.message for r in caplog.records) == 1


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


class _DroppingThenStates(_FakeStream):
    """The first stream drops with a connection error; later ones deliver states."""

    drops = 1

    async def __anext__(self) -> object:
        if _DroppingThenStates.drops > 0:
            _DroppingThenStates.drops -= 1
            raise OSError("connection reset")
        return await super().__anext__()


@pytest.mark.asyncio
async def test_a_dropped_connection_is_reopened_and_an_estop_still_detected():
    _DroppingThenStates.drops = 1
    estop = types.SimpleNamespace(safety_state=_SafetyState.SAFETY_STATE_ROBOT_EMERGENCY_STOP)
    gateway = MagicMock()
    gateway.controller_api.stream_robot_controller_state = lambda **_kw: _DroppingThenStates([
        estop
    ])
    monitor = _patched_monitor([])
    with patch("novapolicy.estop.get_api_gateway", lambda _mg: gateway):
        await _run_monitor_until(monitor)
    assert isinstance(monitor.error, EmergencyStopError)
    assert monitor.error.safety_state == "SAFETY_STATE_ROBOT_EMERGENCY_STOP"


class _AlwaysDropping(_FakeStream):
    async def __anext__(self) -> object:
        raise OSError("no route to host")


@pytest.mark.asyncio
async def test_a_monitor_that_reads_no_state_stops_the_run():
    """Regression: a connection error used to end the monitor and leave the run unmonitored."""
    gateway = MagicMock()
    gateway.controller_api.stream_robot_controller_state = lambda **_kw: _AlwaysDropping([])
    monitor = _patched_monitor([])
    with (
        patch("novapolicy.estop.get_api_gateway", lambda _mg: gateway),
        patch("novapolicy.estop._BLIND_LIMIT_S", 0.2),
    ):
        await _run_monitor_until(monitor, timeout_s=2.0)
    assert isinstance(monitor.error, EmergencyStopError)
    assert monitor.error.safety_state == "E-STOP MONITOR BLIND"
