import asyncio

import pytest

from nova.core.nova import Nova
from nova.events.nats import Message


@pytest.mark.asyncio
async def test_nats_pub_sub():
    nova = Nova()
    await nova.connect()

    collected_message = None

    async def cb(msg):
        print("received message")
        nonlocal collected_message
        collected_message = msg

    await nova.api_gateway.subscribe("nova.test.subject", cb=cb)
    nova.api_gateway.publish_message(Message(subject="nova.test.subject", data=b"test message"))

    # todo, wait for message, this makes the test flaky
    # but since this is integration test, it is okay for now
    await asyncio.sleep(2)

    assert collected_message is not None, "No message received"
    assert collected_message.data == b"test message", (
        "Received message does not match published message"
    )
    assert collected_message.subject == "nova.test.subject", (
        "Received message subject does not match"
    )
