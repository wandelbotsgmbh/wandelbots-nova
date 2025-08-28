"""
NATS client for Nova integration.
"""

from __future__ import annotations

import asyncio
from typing import Awaitable, Callable

import nats
from decouple import config
from nats.aio.msg import Msg as NatsLibMessage

from nova.logging import logger
from nova.nats.message import Message


class _Publisher:
    """Private NATS publisher that handles message publishing with a background task queue."""

    def __init__(self, nats_client: nats.NATS):
        """
        Initialize the NATS publisher.

        Uses a queue to manage publishing tasks.

        Args:
            nats_client: The NATS client instance
        """
        self._nats_client = nats_client
        self._publish_queue: asyncio.Queue[Message] = asyncio.Queue()
        self._publish_queue_consumer = None
        self._logger = logger.getChild("NatsPublisher")
        self._closing = False
        self._shutdown_lock = asyncio.Lock()

        # Start the consumer task
        self._start_consumer()

    def _start_consumer(self):
        """Start the consumer task if it's not already running."""
        if self._publish_queue_consumer is None or self._publish_queue_consumer.done():
            self._logger.debug("Starting NATS message consumer")
            self._publish_queue_consumer = asyncio.create_task(self._publish_queue_task())

    async def connect(self):
        """Connect method for consistency with context manager pattern."""
        # Publisher is already initialized with the NATS client, so nothing to do here
        self._logger.debug("NATS publisher is ready")
        return self

    async def close(self):
        """Close the NATS publisher and clean up resources."""
        async with self._shutdown_lock:
            # Set the closing flag to prevent new messages from being queued
            self._closing = True

            # First, try to gracefully shutdown by sending a sentinel value
            # This allows the consumer to process remaining messages and exit cleanly
            try:
                # Use a special sentinel message to signal shutdown
                sentinel = Message(subject="_shutdown", data=b"")
                self._publish_queue.put_nowait(sentinel)
            except Exception as e:
                self._logger.warning(f"Failed to send shutdown sentinel: {e}")

            await self._stop_nats_message_consumer()

            # Final flush to ensure no messages were added during shutdown
            await self._flush_remaining_messages()

            logger.debug("NATS publisher closed")

    async def __aenter__(self):
        """Async context manager entry."""
        await self.connect()
        return self

    async def __aexit__(self, exc_type, exc_val, exc_tb):
        """Async context manager exit."""
        await self.close()

    def publish(self, message: Message):
        """
        Queue a message for publishing.

        Args:
            message: The message to publish
        """
        if self._closing:
            self._logger.warning(
                f"Publisher is closing, message for {message.subject} will be ignored"
            )
            return

        # Check if we can queue the message
        try:
            self._publish_queue.put_nowait(message)
            self._logger.debug(f"Queued message for subject: {message.subject}")
        except Exception as e:
            self._logger.error(f"Failed to queue message for publishing: {e}")
            return

        # Ensure consumer is running after queuing the message
        if self._publish_queue_consumer is None or self._publish_queue_consumer.done():
            if not self._closing:  # Only restart if we're not in the process of closing
                self._logger.warning("Consumer not running after queuing message, restarting it")
                self._start_consumer()
            else:
                self._logger.info(
                    f"Consumer stopped and publisher is closing, but message for {message.subject} is already queued for final flush"
                )

    async def _stop_nats_message_consumer(self):
        """Stop the NATS message consumer task."""
        if self._publish_queue_consumer and not self._publish_queue_consumer.done():
            self._logger.info("Stopping NATS message consumer")

            # Wait for the consumer task to finish gracefully (it should exit when it sees the sentinel)
            try:
                # Give the consumer some time to process remaining messages and exit gracefully
                await asyncio.wait_for(self._publish_queue_consumer, timeout=5.0)
                self._logger.info("NATS message consumer finished gracefully")
            except asyncio.TimeoutError:
                self._logger.warning("NATS message consumer did not finish gracefully, cancelling")
                self._publish_queue_consumer.cancel()
                try:
                    await self._publish_queue_consumer
                except asyncio.CancelledError:
                    pass
            except Exception as e:
                self._logger.error(f"Error while waiting for consumer to finish: {e}")
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

                # Check for shutdown sentinel
                if nats_message.subject == "_shutdown":
                    self._logger.info("Received shutdown sentinel, exiting consumer gracefully")
                    break

                self._logger.info("publishing")

                try:
                    self._logger.info(
                        f"publishing message to {nats_message.subject}, message size: {len(nats_message.data)}"
                    )
                    await self._nats_client.publish(
                        subject=nats_message.subject, payload=nats_message.data
                    )
                except asyncio.CancelledError:
                    # Put the message back in the queue so it can be processed during shutdown
                    try:
                        self._publish_queue.put_nowait(nats_message)
                    except Exception:
                        pass
                    raise
                except Exception as e:
                    self._logger.error(f"Failed to publish program state change to NATS: {e}")

                    # don't exhaust the event loop
                    await asyncio.sleep(1)
        except asyncio.CancelledError:
            self._logger.info("NATS message consumer cancelled")
            # Process remaining messages during cancellation
            await self._flush_remaining_messages()
        finally:
            self._logger.info("NATS message consumer task finished")

    async def _flush_remaining_messages(self):
        """Flush any remaining messages in the queue."""
        queue_size = self._publish_queue.qsize()
        self._logger.debug(f"Queue size before flushing: {queue_size}")

        if queue_size == 0:
            self._logger.debug("No remaining messages to flush")
            return

        self._logger.info(f"Flushing {queue_size} remaining messages in NATS publishing queue")
        message_count = 0

        while not self._publish_queue.empty():
            try:
                message = self._publish_queue.get_nowait()

                # Skip the shutdown sentinel if it's still in the queue
                if message.subject == "_shutdown":
                    self._logger.debug("Skipping shutdown sentinel message")
                    continue

                await self._nats_client.publish(message.subject, message.data)
                message_count += 1
                self._logger.info(f"Published remaining message to {message.subject}")
            except asyncio.QueueEmpty:
                # Queue became empty during processing, which is fine
                self._logger.debug("Queue became empty during flush")
                break
            except Exception as e:
                self._logger.error(f"Failed to publish remaining message to NATS: {e}")

        if message_count > 0:
            self._logger.info(f"Successfully published {message_count} remaining messages")
        else:
            self._logger.debug("No remaining messages to publish after filtering")


class NatsClient:
    """NATS client for Nova with connection management and publishing capabilities."""

    def __init__(
        self,
        host: str | None = None,
        access_token: str | None = None,
        nats_client_config: dict | None = None,
    ):
        """
        Initialize the NATS client.

        Args:
            host (str | None): The Nova API host.
            access_token (str | None): An access token for authentication.
            nats_client_config (dict | None): Configuration dictionary for NATS client.
        """
        self._host = host
        self._access_token = access_token
        self._nats_config = nats_client_config or {}
        self._nats_client = None
        self._nats_publisher = None
        self._nats_connection_string = None
        self._init_nats_client()

    def _init_nats_client(self) -> None:
        """
        Initialize the NATS WebSocket connection string.

        Order of precedence:
        1) Use self._host if present (derive ws/wss + default port).
        2) Otherwise, read from NATS_BROKER env var.
        """
        host = self._host
        if host:
            host = host.strip()
            is_https = host.startswith("https://")
            # Remove protocol and trailing slashes
            clean_host = host.replace("https://", "").replace("http://", "").rstrip("/")

            scheme, port = ("wss", 443) if is_https else ("ws", 80)
            token = self._access_token
            auth = f"{token}@" if token else ""

            self._nats_connection_string = f"{scheme}://{auth}{clean_host}:{port}/api/nats"
            return

        logger.debug("Host not set; reading NATS connection from env var")
        self._nats_connection_string = config("NATS_BROKER", default=None)

        if not self._nats_connection_string:
            raise ValueError("NATS connection string is not set.")

    async def connect(self):
        """Connect to NATS server."""
        if self._nats_client is not None:
            return

        self._nats_client = await nats.connect(self._nats_connection_string)
        self._nats_publisher = _Publisher(self._nats_client)
        logger.debug("NATS client connected successfully")

    async def close(self):
        """Close NATS connection."""
        if self._nats_publisher:
            await self._nats_publisher.close()
        if self._nats_client:
            await self._nats_client.close()
        logger.debug("NATS client closed")

    async def __aenter__(self):
        await self.connect()
        return self

    async def __aexit__(self, exc_type, exc_val, exc_tb):
        await self.close()

    @property
    def nats_client(self) -> nats.NATS:
        """Access to the underlying NATS client for advanced operations."""
        if self._nats_client is None:
            raise RuntimeError("NATS client is not connected. Call connect() first.")
        return self._nats_client

    def publish_message(self, message: Message):
        """
        Publish a message to a NATS subject.
        Args:
            message (Message): The message to publish.
        """
        logger.warning(
            "Wandelbots Nova NATS integration is in BETA, You might experience issues and the API can change."
        )

        if self._nats_publisher is None:
            logger.warning("your message is ignored")
            logger.warning(
                "You can't publish message when you didn't create a connection. First call connect() or use contextmanager."
            )
            return

        self._nats_publisher.publish(message)

    async def subscribe(self, subject: str, on_message: Callable[[Message], Awaitable[None]]):
        """
        Subscribe to a NATS subject and inform the callback when a message is received.
        Args:
            subject (str): The NATS subject to subscribe to.
            on_message (Callable[[Message], Awaitable[None]]): The callback to call when a message is received.
        """
        logger.warning(
            "Wandelbots Nova NATS integration is in BETA, You might experience issues and the API can change."
        )

        if self._nats_client is None:
            raise RuntimeError("NATS client is not connected. Call connect() first.")

        async def data_mapper(msg: NatsLibMessage):
            message = Message(subject=msg.subject, data=msg.data)
            await on_message(message)

        await self._nats_client.subscribe(subject, cb=data_mapper)
        # ensure subscription is sent to server
        # TODO: nats library has weird meanings for "flushing", double check for correctness
        await self._nats_client.flush()
