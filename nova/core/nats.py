import asyncio

import nats

from nova.logging import logger


class Message:
    def __init__(self, subject: str, data: bytes):
        self.subject = subject
        self.data = data


class _Publisher:
    def __init__(self, nats_client: nats.NATS):
        """
        Publishes messages to NATS with a background task.
        Uses a queue to manage publishing tasks.
        """
        self._nats_client = nats_client
        self._publish_queue: asyncio.Queue[Message] = asyncio.Queue()
        self._publish_queue_consumer = asyncio.create_task(self._publish_queue_task())
        self._logger = logger.getChild("NatsPublisher")

    async def connect(self):
        """Connect method for consistency with context manager pattern."""
        # Publisher is already initialized with the NATS client, so nothing to do here
        self._logger.debug("NATS publisher is ready")
        return self

    async def close(self):
        """Close the NATS publisher and clean up resources."""
        await self._stop_nats_message_consumer()
        logger.debug("NATS publisher closed")

    async def __aenter__(self):
        """Async context manager entry."""
        await self.connect()
        return self

    async def __aexit__(self, exc_type, exc_val, exc_tb):
        """Async context manager exit."""
        await self.close()

    def publish(self, message: Message):
        self._publish_queue.put_nowait(message)

    async def _stop_nats_message_consumer(self):
        """Stop the NATS message consumer task and clear remaining queue items."""
        if self._publish_queue_consumer and not self._publish_queue_consumer.done():
            self._logger.info("Stopping NATS message consumer")
            self._publish_queue_consumer.cancel()

            try:
                await self._publish_queue_consumer
            except asyncio.CancelledError:
                pass

            self._publish_queue_consumer = None
        else:
            self._logger.debug("NATS message consumer not running")

    async def _publish_queue_task(self):
        """Consume all program state data from the queue and publish it to NATS."""
        # TODO: double check this logic
        try:
            while True:
                nats_message = await self._publish_queue.get()
                self._logger.info("publishing")

                try:
                    self._logger.info(
                        f"publishing message to {nats_message.subject}, message size: {len(nats_message.data)}"
                    )
                    await self._nats_client.publish(
                        subject=nats_message.subject, payload=nats_message.data
                    )
                except asyncio.CancelledError:
                    # allow cancellation
                    raise
                except Exception as e:
                    self._logger.error(f"Failed to publish program state change to NATS: {e}")

                    # don't exhaust the event loop
                    await asyncio.sleep(1)
        except asyncio.CancelledError:
            self._logger.info("Clearing remaining items in nats publishing queue")
            while not self._publish_queue.empty():
                message = self._publish_queue.get_nowait()
                try:
                    await self._nats_client.publish(message.subject, message.data)
                except Exception as e:
                    self._logger.error(f"Failed to publish remaining messages to NATS: {e}")

            self._logger.info("NATS message consumer cancelled")
