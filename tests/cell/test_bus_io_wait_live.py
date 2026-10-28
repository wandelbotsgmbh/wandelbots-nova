"""Live-broker tests for bus-IO waits across an expired NATS login (marked ``nats``).

A program paused on a bus IO waits for pushed values. When the server expires the login
the client reconnects, but values pushed in between are gone. These tests drive that
window against a real ``nats-server`` through ``Nova``'s client and check that the wait
neither fails nor misses the release.
"""

import asyncio
import json
from types import SimpleNamespace
from unittest.mock import AsyncMock

import nats
import pytest

from nova import api
from nova.cell.io_condition import IOConditionWatcher, motion_enable_signal
from nova.config import NovaConfig
from nova.core.nova import Nova
from tests.nats_live import expire_login, nats_server_url, wait_connected

pytestmark = pytest.mark.nats

VALUES = "nova.v2.cells.cell.bus-ios.ios"


@pytest.fixture(scope="module")
def nats_url():
    with nats_server_url() as url:
        yield url


@pytest.fixture
async def nova(nats_url):
    config = NovaConfig(
        host="http://127.0.0.1:9",
        nats_client_config={
            "servers": nats_url,
            "reconnect_time_wait": 0.1,
            "max_reconnect_attempts": 20,
        },
    )
    client = Nova(config)
    await client.open()
    yield client
    await client.close()


@pytest.fixture
async def plc(nats_url):
    """A second client standing in for the bus-IO service publishing values."""
    client = await nats.connect(nats_url)
    yield client
    await client.drain()


def _gateway(*reads: bool):
    """A bus-IO API double whose successive reads of ``enable`` return ``reads``."""
    return SimpleNamespace(
        bus_ios_api=SimpleNamespace(
            get_bus_io_values=AsyncMock(
                side_effect=[[api.models.IOBooleanValue(io="enable", value=v)] for v in reads]
            )
        )
    )


def _pushed(value: bool) -> bytes:
    return json.dumps([{"io": "enable", "value": value, "value_type": "boolean"}]).encode()


async def test_release_during_the_expired_login_is_seen(nova, plc):
    gateway = _gateway(False, True)  # paused at first; released by the time we re-read
    watcher = IOConditionWatcher(gateway, "cell", "ctrl", nats_client=nova.nats)

    wait = asyncio.create_task(watcher.wait_until_released(motion_enable_signal("enable")))
    await asyncio.sleep(0.2)
    assert not wait.done()

    await expire_login(nova.nats)
    # The PLC releases while our subscription is not on the server.
    await plc.publish(VALUES, _pushed(True))
    await plc.flush()

    async with asyncio.timeout(5):
        await wait
    assert gateway.bus_ios_api.get_bus_io_values.await_count == 2


async def test_wait_started_during_the_expired_login_does_not_fail(nova, plc):
    gateway = _gateway(False)
    watcher = IOConditionWatcher(gateway, "cell", "ctrl", nats_client=nova.nats)

    await expire_login(nova.nats)
    wait = asyncio.create_task(watcher.wait_until_released(motion_enable_signal("enable")))
    await asyncio.sleep(0.05)
    assert not wait.done(), "no IOConditionUnavailable while reconnecting"

    await wait_connected(nova.nats)
    await asyncio.sleep(0.1)  # let the queued subscribe reach the server
    await plc.publish(VALUES, _pushed(True))
    await plc.flush()

    async with asyncio.timeout(5):
        await wait
