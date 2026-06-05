"""Tests for e-stop detection.

Unit tests mock the controller-state stream; the integration test drives a real
virtual controller via ``set_estop`` and asserts ``EstopMonitor`` fires.
"""

from __future__ import annotations

import asyncio
import types
from unittest.mock import MagicMock, patch

import pytest

from nova import api
from policy.estop import (
    _OPERATIONAL_SAFETY_STATES,
    EstopMonitor,
    check_estop,
    check_sessions,
)
from policy.types import EmergencyStopError, MotionError

_SafetyState = api.models.SafetyStateType


# ---------------------------------------------------------------------------
# Safety-state definition (single source of truth = SafetyStateType enum)
# ---------------------------------------------------------------------------


def test_operational_states_are_the_enum_values():
    assert (
        frozenset({_SafetyState.SAFETY_STATE_NORMAL, _SafetyState.SAFETY_STATE_REDUCED})
        == _OPERATIONAL_SAFETY_STATES
    )


# ---------------------------------------------------------------------------
# check_estop / check_sessions
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
        "policy.estop",
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
    state = types.SimpleNamespace(safety_state=_SafetyState.SAFETY_STATE_PROTECTIVE_STOP)
    monitor = _patched_monitor([state])
    await _run_monitor_until(monitor)
    assert isinstance(monitor.error, EmergencyStopError)
    assert monitor.error.safety_state == "SAFETY_STATE_PROTECTIVE_STOP"


@pytest.mark.asyncio
async def test_monitor_stays_clear_on_operational_states():
    states = [
        types.SimpleNamespace(safety_state=_SafetyState.SAFETY_STATE_NORMAL),
        types.SimpleNamespace(safety_state=_SafetyState.SAFETY_STATE_REDUCED),
    ]
    monitor = _patched_monitor(states)
    await _run_monitor_until(monitor, timeout_s=0.2)
    assert monitor.error is None


# ---------------------------------------------------------------------------
# Integration: drive a real virtual controller into e-stop
# ---------------------------------------------------------------------------


@pytest.mark.integration
@pytest.mark.asyncio
async def test_estop_monitor_detects_real_set_estop():
    from nova import Nova
    from nova.cell import virtual_controller

    async with Nova() as nova_instance:
        cell = nova_instance.cell()
        controller = await cell.ensure_controller(
            virtual_controller(
                name="ur10e-estop-mon",
                manufacturer=api.models.Manufacturer.UNIVERSALROBOTS,
                type="universalrobots-ur10e",
            )
        )
        mg = controller[0]
        monitor = EstopMonitor([mg])
        await monitor.start()
        try:
            await controller.set_estop(active=True)
            for _ in range(50):
                if monitor.error is not None:
                    break
                await asyncio.sleep(0.1)
            assert isinstance(monitor.error, EmergencyStopError)
        finally:
            await monitor.stop()
            await controller.set_estop(active=False)
