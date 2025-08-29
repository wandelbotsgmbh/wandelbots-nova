import asyncio

from nova.core.nova import Nova
from nova.nats import Message


# @pytest.mark.xdist_group("program-runs")
# @pytest.mark.asyncio
async def _test_nats_pub_sub():
    nova = Nova()
    await nova.connect()

    collected_message = None

    async def cb(msg):
        print("received message")
        nonlocal collected_message
        collected_message = msg

    await nova.api_gateway.subscribe("nova.test.subject", on_message=cb)
    nova.api_gateway.publish_message(Message(subject="nova.test.subject", data=b"test message"))

    await asyncio.sleep(2)

    assert collected_message is not None, "No message received"
    assert collected_message.data == b"test message", (
        "Received message does not match published message"
    )
    assert collected_message.subject == "nova.test.subject", (
        "Received message subject does not match"
    )
