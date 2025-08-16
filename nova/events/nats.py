import asyncio
import logging
import sys
from typing import Any, Awaitable, Callable

from decouple import config
from nats import NATS

# Can't import from nova.core because of cyclic imports
# need to refactor probably
# nova.core.gateway needs this
LOG_LEVEL: str = config("LOG_LEVEL", default="INFO").upper()
LOG_FORMAT: str = config("LOG_FORMAT", default="%(asctime)s [%(levelname)s] %(name)s: %(message)s")
LOG_DATETIME_FORMAT: str = config("LOG_DATETIME_FORMAT", default="%Y-%m-%d %H:%M:%S")
LOGGER_NAME: str = config("LOGGER_NAME", default="wandelbots-nova")

# Setting up the underlying logger
formatter: logging.Formatter = logging.Formatter(LOG_FORMAT, datefmt=LOG_DATETIME_FORMAT)
handler: logging.StreamHandler = logging.StreamHandler(sys.stdout)
handler.setLevel(LOG_LEVEL)
handler.setFormatter(formatter)

logger: logging.Logger = logging.getLogger(LOGGER_NAME)
logger.setLevel(LOG_LEVEL)
logger.addHandler(handler)


class Message:
    # TODO: we can add support for other NATS publishing params
    # e.g. message headers, reply
    # see nats.publish
    def __init__(self, subject: str, data: bytes):
        self.subject = subject
        self.data = data


class Client:
    """
    A wrapper around nats package, don't use nats package directly in the project.
    Instead use this client, so we can change it later.
    """

    def __init__(self, nats_servers: str):
        self._nats_servers = nats_servers
        self._nats_client: NATS = None

    async def connect(self):
        """Connect method for consistency with context manager pattern."""
        # Client is already initialized with the NATS client, so nothing to do here
        self._nats_client = NATS()
        await self._nats_client.connect(self._nats_servers)
        logger.debug("NATS client is ready")
        return self

    async def close(self):
        """Close the NATS client and clean up resources."""
        await self._nats_client.drain()
        logger.debug("NATS client closed")

    async def __aenter__(self):
        """Async context manager entry."""
        await self.connect()
        return self

    async def __aexit__(self, exc_type, exc_val, exc_tb):
        """Async context manager exit."""
        await self.close()

    async def publish(self, message: Message):
        await self._nats_client.publish(message.subject, message.data)

    async def subscribe(self, subject: str, cb: Callable[[Any], Awaitable[None]]):
        """Subscribe to a NATS subject with a callback function."""
        return await self._nats_client.subscribe(subject, cb=cb)


# TODO: this class requires some work
class Publisher:
    def __init__(self, nats_client: Client):
        """
        Publishes messages to NATS with a background task.
        Uses a queue to manage publishing tasks.
        """
        self._nats_client = nats_client
        self._publish_queue = asyncio.Queue()
        self._publish_queue_consumer = asyncio.create_task(self._publish_queue_task())

    async def connect(self):
        """Connect method for consistency with context manager pattern."""
        # Publisher is already initialized with the NATS client, so nothing to do here
        logger.debug("NATS publisher is ready")
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
            logger.info("Stopping NATS message consumer")
            self._publish_queue_consumer.cancel()

            try:
                await self._publish_queue_consumer
            except asyncio.CancelledError:
                pass

            self._publish_queue_consumer = None
        else:
            logger.debug("NATS message consumer not running")

    async def _publish_queue_task(self):
        """Consume all program state data from the queue and publish it to NATS."""
        # TODO: double check this logic
        try:
            while True:
                nats_message = await self._publish_queue.get()
                logger.info("publishing")

                try:
                    await self._nats_client.publish(nats_message)
                except asyncio.CancelledError:
                    # allow cancellation
                    raise
                except Exception as e:
                    logger.error(f"Failed to publish program state change to NATS: {e}")

                    # don't exhaust the event loop
                    await asyncio.sleep(1)
        except asyncio.CancelledError:
            logger.info("Clearing remaining items in nats publishing queue")
            while not self._publish_queue.empty():
                message = self._publish_queue.get_nowait()
                try:
                    await self._nats_client.publish(message.subject, message.data)
                except Exception as e:
                    logger.error(f"Failed to publish remaining messages to NATS: {e}")

            logger.info("NATS message consumer cancelled")
