"""Unit tests for the NATS connection-state helpers."""

import asyncio
from types import SimpleNamespace

import pytest

from nova.core.nats_state import ReconnectSignal, nats_usable

pytestmark = pytest.mark.asyncio


@pytest.mark.parametrize(
    ("client", "usable"),
    [
        (None, False),
        (SimpleNamespace(is_connected=True, is_reconnecting=False, is_closed=False), True),
        (SimpleNamespace(is_connected=False, is_reconnecting=True, is_closed=False), True),
        (SimpleNamespace(is_connected=False, is_reconnecting=False, is_closed=False), False),
        (SimpleNamespace(is_connected=False, is_reconnecting=False, is_closed=True), False),
        # nats-py reports is_connected while draining; a closed client never counts
        (SimpleNamespace(is_connected=True, is_reconnecting=False, is_closed=True), False),
        # a double that only knows is_connected
        (SimpleNamespace(is_connected=True), True),
        (SimpleNamespace(is_connected=False), False),
    ],
)
def test_nats_usable_accepts_connected_and_reconnecting_clients(client, usable):
    assert nats_usable(client) is usable


async def test_reconnect_signal_wakes_every_waiter_once_per_reconnect():
    signal = ReconnectSignal()
    seen = {"a": 0, "b": 0}

    async def waiter(name: str) -> None:
        while True:
            await signal.wait()
            seen[name] += 1

    tasks = [asyncio.create_task(waiter("a")), asyncio.create_task(waiter("b"))]
    await asyncio.sleep(0)

    await signal.fire()
    await asyncio.sleep(0)
    assert seen == {"a": 1, "b": 1}

    await signal.fire()
    await asyncio.sleep(0)
    assert seen == {"a": 2, "b": 2}
    assert signal.generation == 2

    for task in tasks:
        task.cancel()


async def test_reconnect_signal_does_not_report_reconnects_before_the_wait():
    signal = ReconnectSignal()
    await signal.fire()

    wait = asyncio.create_task(signal.wait())
    await asyncio.sleep(0)
    assert not wait.done(), "a reconnect that happened earlier is not a new one"

    await signal.fire()
    async with asyncio.timeout(1):
        await wait


async def test_reconnect_signal_wait_since_catches_a_reconnect_before_the_wait():
    """A consumer reads the generation, sets up its subscriptions, then waits: a reconnect
    that lands in between must still be reported."""
    signal = ReconnectSignal()
    since = signal.generation
    await signal.fire()

    async with asyncio.timeout(1):
        await signal.wait(since)
