from __future__ import annotations

import asyncio
from typing import Any, Callable

from decouple import config

from nova.cell.cell import Cell
from nova.core.gateway import ApiGateway
from nova.events import cycle_event_handler, cycle_failed, cycle_finished, cycle_started
from nova.logging import logger
from nova.nats import NatsClient

LOG_LEVEL = config("LOG_LEVEL", default="INFO")
CELL_NAME = config("CELL_NAME", default="cell", cast=str)


# TODO: could also extend NovaDevice
class Nova:
    """A high-level Nova client for interacting with robot cells and controllers."""

    def __init__(
        self,
        *,
        host: str | None = None,
        access_token: str | None = None,
        username: str | None = None,
        password: str | None = None,
        version: str = "v1",
        verify_ssl: bool = True,
        nats_client_config: dict | None = None,
    ):
        """
        Initialize the Nova client.

        Args:
            host (str | None): The Nova API host.
            access_token (str | None): An access token for the Nova API.
            username (str | None): Username to authenticate with the Nova API.
            password (str | None): Password to authenticate with the Nova API.
            version (str): The API version to use (default: "v1").
            verify_ssl (bool): Whether or not to verify SSL certificates (default: True).
            nats_client_config (dict | None): Configuration dictionary for NATS client.
        """

        if host is None:
            host = config("NOVA_API", default=None)
        if access_token is None:
            access_token = config("NOVA_ACCESS_TOKEN", default=None)
        if username is None:
            username = config("NOVA_USERNAME", default=None)
        if password is None:
            password = config("NOVA_PASSWORD", default=None)

        self._api_client = ApiGateway(
            host=host,
            access_token=access_token,
            username=username,
            password=password,
            version=version,
            verify_ssl=verify_ssl,
        )

        self.nats = NatsClient(
            host=host, access_token=access_token, nats_client_config=nats_client_config
        )

    def cell(self, cell_id: str = CELL_NAME) -> Cell:
        """Returns the cell object with the given ID."""
        return Cell(self._api_client, cell_id)

    def connect_cycle_signals(self) -> None:
        """
        Connect the cycle event signals to the NATS event handler with the nats client.

        This creates a mapping of signal handlers for this Nova instance
        so they can be properly disconnected later.
        """
        # Store handlers for this instance so we can disconnect them later
        self._signal_handlers = {}

        def create_handler(nats_client: NatsClient) -> Callable[..., None]:
            """Create a handler function that captures the nats client"""

            def handler(sender: Any, **kwargs: Any) -> None:
                """Synchronous handler that schedules async work"""
                logger.debug(f"Cycle event handler called with sender: {sender}, kwargs: {kwargs}")
                message = kwargs.get("message")
                if message is None:
                    logger.debug("No message provided to cycle event handler")
                    return
                logger.debug(f"Calling cycle_event_handler with message: {message}")
                # blinker needs sync functions but sending nats message is async, so we need to do this
                asyncio.get_running_loop().create_task(
                    cycle_event_handler(sender, message=message, nats_client=nats_client)
                )

            return handler

        for signal_obj in [cycle_started, cycle_finished, cycle_failed]:
            event_type = signal_obj.name
            logger.debug(f"Connecting NATS event handler for {event_type} event")
            handler = create_handler(self.nats)
            signal_obj.connect(handler)
            # Store the handler so we can disconnect it later
            self._signal_handlers[signal_obj] = handler
            logger.debug(
                f"After connecting: {event_type} signal has {len(signal_obj.receivers)} receivers"
            )

    def disconnect_cycle_signals(self) -> None:
        """
        Disconnect the cycle event signals from the NATS event handler.

        This only disconnects the handlers that were connected by this Nova instance,
        not all handlers from all Nova instances.
        """
        if not hasattr(self, "_signal_handlers"):
            logger.debug("No signal handlers to disconnect")
            return

        for signal_obj, handler in self._signal_handlers.items():
            event_type = signal_obj.name
            logger.debug(f"Disconnecting NATS event handler for {event_type} event")
            logger.debug(
                f"Before disconnecting: {event_type} signal has {len(signal_obj.receivers)} receivers"
            )
            # Disconnect only this specific handler, not all receivers
            signal_obj.disconnect(handler)
            logger.debug(
                f"After disconnecting: {event_type} signal has {len(signal_obj.receivers)} receivers"
            )

        # Clear the handlers mapping for this instance
        self._signal_handlers.clear()

    async def connect(self):
        # ApiGateway doesn't need an explicit connect call, it's initialized in constructor
        await self.nats.connect()

    async def close(self):
        """Closes the underlying API client session and NATS client."""
        await self.nats.close()
        return await self._api_client.close()

    async def __aenter__(self):
        # Configure any active viewers
        try:
            from nova.viewers import _configure_active_viewers

            _configure_active_viewers(self)
        except ImportError:
            pass

        await self.connect()

        # Connect cycle event signals to NATS (only for context-managed usage)
        self.connect_cycle_signals()

        return self

    async def __aexit__(self, exc_type, exc_val, exc_tb):
        # Disconnect cycle event signals
        self.disconnect_cycle_signals()

        await self.close()
