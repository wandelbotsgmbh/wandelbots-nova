import asyncio

import pytest

from nova.core.nova import Nova


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

    await nova.nats.subscribe("nova.test.subject", cb=cb)
    await nova.nats.publish(subject="nova.test.subject", payload=b"test message")

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

    await nova.nats.subscribe("nova.test.order", cb=collect_message)

    for i in range(1, 11):
        await nova.nats.publish(subject="nova.test.order", payload=str(i).encode())

    await asyncio.sleep(3)

    assert len(messages) == 10
    assert messages == ["1", "2", "3", "4", "5", "6", "7", "8", "9", "10"]
