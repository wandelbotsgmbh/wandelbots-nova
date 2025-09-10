import asyncio

import pytest

from nova.core.nova import Nova
from nova.nats import Message


@pytest.mark.integration
@pytest.mark.xdist_group("program-runs")
@pytest.mark.asyncio
async def test_nats_pub_sub():
    nova = Nova()
    await nova.connect()

    collected_message = None

    async def cb(msg):
        print("received message")
        nonlocal collected_message
        collected_message = msg

    await nova.nats.subscribe("nova.test.subject", on_message=cb)
    await nova.nats.publish_message(Message(subject="nova.test.subject", data=b"test message"))

    await asyncio.sleep(2)

    assert collected_message is not None, "No message received"
    assert collected_message.data == b"test message", (
        "Received message does not match published message"
    )
    assert collected_message.subject == "nova.test.subject", (
        "Received message subject does not match"
    )


@pytest.mark.integration
@pytest.mark.xdist_group("program-runs")
@pytest.mark.asyncio
async def test_nats_message_order():
    nova = Nova()
    await nova.connect()

    messages = []

    async def collect_message(msg):
        messages.append(msg.data.decode())

    await nova.nats.subscribe("nova.test.order", on_message=collect_message)

    for i in range(1, 11):
        await nova.nats.publish_message(Message(subject="nova.test.order", data=str(i).encode()))

    await asyncio.sleep(3)

    assert len(messages) == 10
    assert messages == ["1", "2", "3", "4", "5", "6", "7", "8", "9", "10"]


@pytest.mark.integration
@pytest.mark.xdist_group("program-runs")
@pytest.mark.asyncio
async def test_nats_message_order_two_instances():
    nova_listener = Nova()
    nova_publisher = Nova()

    async with nova_listener, nova_publisher:
        messages = []

        async def collect_message(msg):
            messages.append(msg.data.decode())

        # Subscribe with the listener instance
        await nova_listener.nats.subscribe(
            "nova.test.order.two_instances", on_message=collect_message
        )

        # Publish with the publisher instance
        for i in range(1, 11):
            await nova_publisher.nats.publish_message(
                Message(subject="nova.test.order.two_instances", data=str(i).encode())
            )

        await asyncio.sleep(5)

        assert len(messages) == 10
        assert messages == ["1", "2", "3", "4", "5", "6", "7", "8", "9", "10"]
