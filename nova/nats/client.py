"""
NATS client for Nova integration.
"""

import asyncio
from typing import Awaitable, Callable

import nats
from decouple import config
from nats.aio.msg import Msg as NatsLibMessage

from nova.logging import logger
from nova.nats.message import Message


class NatsClient:
    """NATS client for Nova with connection management and publishing capabilities."""

    def __init__(
        self,
        host: str | None = None,
        access_token: str | None = None,
        # TODO: accept "broker" parameter so the user can override the connection string
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
        self._nats_client: nats.NATS | None = None
        self._nats_connection_string: str = ""
        self._connect_lock = asyncio.Lock()
        self._init_nats_client()

    # TODO: nats connection string is not built correctly when being accessed like below
    # nova = Nova(host="...")
    # nova.nats -> here we are still dependent on NATS_BROKER env variable
    def _init_nats_client(self) -> None:
        host = self._host
        token = self._access_token

        if host and token:
            host = host.strip()
            is_http = host.startswith("http://")
            # Remove protocol and trailing slashes
            clean_host = host.replace("https://", "").replace("http://", "").rstrip("/")

            scheme, port = ("ws", 80) if is_http else ("wss", 443)
            auth = f"{token}@"

            self._nats_connection_string = f"{scheme}://{auth}{clean_host}:{port}/api/nats"
            return

        logger.debug("Host and token not both set; reading NATS connection from env var")
        self._nats_connection_string = config("NATS_BROKER", default=None)

        if not self._nats_connection_string:
            logger.warning("NATS connection string is not set. NATS client will be disabled.")

    async def connect(self):
        """Connect to NATS server.

        Subsequent calls are no-ops while connected.
        """
        if not self._nats_connection_string:
            logger.info("NATS client is not configured. Skipping connection.")
            return

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
        if not self._nats_connection_string:
            return False
        return self._nats_client is not None and self._nats_client.is_connected

    async def publish_message(self, message: Message) -> None:
        """
        Publish a message to a NATS subject.
        Args:
            message (Message): The message to publish.
        """
        logger.debug(
            "Wandelbots Nova NATS integration is in BETA, You might experience issues and the API can change."
        )

        if not self.is_connected():
            logger.debug("NATS client is not connected. Skipping message publishing.")
            return

        if self._nats_client is None:
            logger.debug("NATS client is None. Skipping message publishing.")
            return

        try:
            await self._nats_client.publish(subject=message.subject, payload=message.data)
            # Ensure the server has processed the publish before returning
            await self._nats_client.flush()
        except Exception as e:
            logger.error(f"Failed to publish message to {message.subject}: {e}")

    async def subscribe(self, subject: str, on_message: Callable[[Message], Awaitable[None]]):
        """
        Subscribe to a NATS subject and inform the callback when a message is received.
        Args:
            subject (str): The NATS subject to subscribe to.
            on_message (Callable[[Message], Awaitable[None]]): The callback to call when a message is received.
        """
        logger.debug(
            "Wandelbots Nova NATS integration is in BETA, You might experience issues and the API can change."
        )

        if not self.is_connected():
            logger.debug("NATS client is not connected. Skipping subscription.")
            return

        if self._nats_client is None:
            logger.debug("NATS client is None. Skipping subscription.")
            return

        async def data_mapper(msg: NatsLibMessage):
            message = Message(subject=msg.subject, data=msg.data)
            await on_message(message)

        await self._nats_client.subscribe(subject, cb=data_mapper)
        # ensure the sub is sent to server before returning
        await self._nats_client.flush()
