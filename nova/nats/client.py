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
    """Minimal async publisher with a single background worker and graceful shutdown."""

    def __init__(self, nats_client: nats.NATS):
        self._nats_client = nats_client
        self._logger = logger.getChild("NatsPublisher")
        self._queue: asyncio.Queue[Message | None] = asyncio.Queue()
        self._worker_task: asyncio.Task | None = asyncio.create_task(self._worker())
        self._closed = False

    def publish(self, message: Message) -> None:
        """Enqueue a message for publishing."""
        if self._closed:
            self._logger.warning(
                f"Publisher is closed; ignoring message for {message.subject}"
            )
            return
        try:
            self._queue.put_nowait(message)
        except Exception as e:
            self._logger.error(f"Failed to enqueue message for {message.subject}: {e}")

    async def close(self) -> None:
        """Signal worker to stop, flush remaining messages, and wait for completion."""
        if self._closed:
            return
        self._closed = True

        # Wake up the worker with a sentinel
        try:
            self._queue.put_nowait(None)
        except Exception:
            pass

        if self._worker_task is not None:
            try:
                await self._worker_task
            except asyncio.CancelledError:
                pass
            finally:
                self._worker_task = None
        self._logger.debug("NATS publisher closed")

    async def _worker(self) -> None:
        """Consume messages from the queue and publish them to NATS.

        On shutdown sentinel (None), flush any remaining messages and exit.
        """
        try:
            while True:
                item = await self._queue.get()
                if item is None:  # shutdown signal
                    break
                try:
                    await self._nats_client.publish(subject=item.subject, payload=item.data)
                except Exception as e:
                    self._logger.error(
                        f"Failed to publish message to {item.subject}: {e}"
                    )
                    # Avoid tight loop in case of repeated failure
                    await asyncio.sleep(1)

            # Flush any remaining messages before exit
            flushed = 0
            while not self._queue.empty():
                m = self._queue.get_nowait()
                if m is None:
                    continue
                try:
                    await self._nats_client.publish(subject=m.subject, payload=m.data)
                    flushed += 1
                except Exception as e:
                    self._logger.error(
                        f"Failed to publish remaining message to {m.subject}: {e}"
                    )
            if flushed:
                self._logger.info(f"Flushed {flushed} remaining messages before shutdown")
        except asyncio.CancelledError:
            # If cancelled unexpectedly, just exit
            self._logger.debug("NATS publisher worker cancelled")
        finally:
            self._logger.info("NATS publisher worker stopped")


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
        self._connect_lock = asyncio.Lock()
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
        """Connect to NATS server.

        Creates the publisher on first successful connection and reuses it thereafter.
        Subsequent calls are no-ops while connected.
        """
        async with self._connect_lock:
            if self._nats_client is not None:
                return

            self._nats_client = await nats.connect(self._nats_connection_string)
            self._nats_publisher = _Publisher(self._nats_client)
            logger.debug("NATS client connected successfully")

    async def close(self):
        """Close NATS connection. Safe to call multiple times."""
        async with self._connect_lock:
            if self._nats_publisher:
                try:
                    await self._nats_publisher.close()
                finally:
                    self._nats_publisher = None
            if self._nats_client:
                try:
                    await self._nats_client.close()
                finally:
                    self._nats_client = None
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
