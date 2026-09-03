"""Surfacing of the motion-group state stream rate.

Resolution order for one execution: explicit ``state_stream_rate_msecs``
parameter, then ``NovaConfig.motion_group_state_rate_msecs`` (env:
``NOVA_MOTION_GROUP_STATE_RATE_MSECS``), then the server default (``None``,
200 ms). The TrajectoryExecutor surfaces the same knob per group through
``GroupArgs``.
"""

import asyncio
from collections.abc import AsyncIterator

import pytest

from nova import api
from nova.cell.trajectory_executor import GroupArgs, TrajectoryExecutor
from nova.config import NovaConfig
from tests.cell.multi_group_doubles import motion_group as motion_group_double
from tests.cell.multi_group_doubles import multi_trajectory, sync_driver
from tests.cell.test_execute_shared_stream import _joint_trajectory, _motion_group
from tests.cell.test_state_stream import FakeUpstream

pytestmark = pytest.mark.asyncio


async def _run_execution(motion_group) -> None:
    async with asyncio.timeout(5):
        async for _ in motion_group.stream_execute(
            _joint_trajectory(), tcp=None, actions=[], state_stream_rate_msecs=None
        ):
            pass


async def test_explicit_rate_opens_the_socket_at_that_rate():
    upstream = FakeUpstream()
    motion_group = _motion_group(upstream)

    async with asyncio.timeout(5):
        async for _ in motion_group.stream_execute(
            _joint_trajectory(), tcp=None, actions=[], state_stream_rate_msecs=100
        ):
            pass

    assert upstream.open_rates == [100]


async def test_nova_config_rate_is_the_fallback():
    upstream = FakeUpstream()
    motion_group = _motion_group(upstream)
    motion_group._api_client.config = NovaConfig(
        host="http://localhost", motion_group_state_rate_msecs=333
    )

    await _run_execution(motion_group)

    assert upstream.open_rates == [333]


async def test_explicit_rate_wins_over_nova_config():
    upstream = FakeUpstream()
    motion_group = _motion_group(upstream)
    motion_group._api_client.config = NovaConfig(
        host="http://localhost", motion_group_state_rate_msecs=333
    )

    async with asyncio.timeout(5):
        async for _ in motion_group.stream_execute(
            _joint_trajectory(), tcp=None, actions=[], state_stream_rate_msecs=50
        ):
            pass

    assert upstream.open_rates == [50]


async def test_without_rate_the_server_default_is_requested():
    upstream = FakeUpstream()
    motion_group = _motion_group(upstream)

    await _run_execution(motion_group)

    assert upstream.open_rates == [None]


async def test_group_args_rate_reaches_each_groups_stream():
    from tests.cell.test_trajectory_executor_session import _FakeGateway

    gateway = _FakeGateway()
    requested_rates: dict[str, list] = {"a": [], "b": []}

    def make_group(name: str):
        group = motion_group_double(gateway, trajectory_id=f"traj-{name}")
        zero_arg_stream = group.stream_state

        def stream_state(rate_msecs=None) -> AsyncIterator[api.models.MotionGroupState]:
            requested_rates[name].append(rate_msecs)
            return zero_arg_stream()

        group.stream_state = stream_state
        return group

    executor = TrajectoryExecutor(
        {"a": make_group("a"), "b": make_group("b")}, sync=sync_driver(gateway)
    )

    async with asyncio.timeout(5):
        async with executor.attach(
            multi_trajectory("a", "b"),
            {"a": GroupArgs(state_stream_rate_msecs=250)},  # "b" keeps the default
        ):
            pass

    assert requested_rates["a"] == [250]
    assert requested_rates["b"] == [None]


async def test_tuning_mode_honors_the_resolved_rate(monkeypatch):
    """ENABLE_TRAJECTORY_TUNING must not silently drop an explicit rate: the
    tuner's zero-arg stream factory carries it, bound the same way as for the
    TrajectoryExecutor."""
    from unittest.mock import AsyncMock, MagicMock

    from tests.cell.multi_group_doubles import state
    from tests.cell.test_execute_shared_stream import _with_tcp

    upstream = FakeUpstream()
    motion_group = _motion_group(upstream)
    motion_group._api_client.motion_group_api = MagicMock()
    motion_group._api_client.motion_group_api.get_current_motion_group_state = AsyncMock(
        return_value=_with_tcp(state(True))
    )
    monkeypatch.setattr("nova.cell.motion_group.ENABLE_TRAJECTORY_TUNING", True)

    captured: dict = {}

    class FakeTuner:
        def __init__(self, plan_fn, execute_fn):
            pass

        async def tune(self, actions, motion_group_state_stream_fn):
            captured["factory"] = motion_group_state_stream_fn
            return
            yield  # unreachable; makes this an async generator

    monkeypatch.setattr("nova.cell.motion_group.TrajectoryTuner", FakeTuner)

    async with asyncio.timeout(5):
        async for _ in motion_group.stream_execute(
            _joint_trajectory(), tcp=None, actions=[], state_stream_rate_msecs=77
        ):
            pass

    stream = captured["factory"]()  # the tuner calls the factory zero-arg
    upstream.feed(_with_tcp(state(True)))
    await asyncio.wait_for(stream.__anext__(), 1.0)
    assert upstream.open_rates == [77], "the tuning stream must carry the resolved rate"
    await stream.aclose()
