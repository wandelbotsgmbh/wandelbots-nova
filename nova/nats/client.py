"""
NATS client for Nova integration.
"""

from __future__ import annotations

import asyncio
from typing import Awaitable, Callable

import nats
from decouple import config
from nats.aio.client import Client as NatsClient
from nats.aio.msg import Msg as NatsLibMessage

from nova.logging import logger
from nova.nats.message import Message

# Internal publisher removed; we publish directly to NATS


class NatsClient:
    """NATS client for Nova with connection management and publishing capabilities."""

    def __init__(
        self,
        host: str | None = None,
        access_token: str | None = None,
        # TODO: check with Dirk, this allows all the config to pass from nova class to nats library
        # is this really what we want
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
        self._nats_client: NatsClient | None = None
        self._nats_connection_string: str = ""
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

        Subsequent calls are no-ops while connected.
        """
        async with self._connect_lock:
            if self._nats_client is not None:
                return

            self._nats_client = await nats.connect(
                self._nats_connection_string, **self._nats_config
            )
            logger.debug("NATS client connected successfully")

    async def close(self):
        """Close NATS connection. Safe to call multiple times."""
        async with self._connect_lock:
            if self._nats_client:
                try:
                    await self._nats_client.drain()
                finally:
                    self._nats_client = None
            logger.debug("NATS client closed")

    async def __aenter__(self):
        await self.connect()
        return self

    async def __aexit__(self, exc_type, exc_val, exc_tb):
        await self.close()

    def is_connected(self) -> bool:
        """Check if the NATS client is connected.

        Returns:
            bool: True if the NATS client is connected, False otherwise.
        """
        return self._nats_client is not None and self._nats_client.is_connected

    async def publish_message(self, message: Message) -> None:
        """
        Publish a message to a NATS subject.
        Args:
            message (Message): The message to publish.
        """
        logger.warning(
            "Wandelbots Nova NATS integration is in BETA, You might experience issues and the API can change."
        )

        if self._nats_client is None:
            logger.warning("your message is ignored")
            logger.warning(
                "You can't publish message when you didn't create a connection. First call connect() or use contextmanager."
            )
            return
        try:
            await self._nats_client.publish(subject=message.subject, payload=message.data)
        except Exception as e:
            logger.error(f"Failed to publish message to {message.subject}: {e}")

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
