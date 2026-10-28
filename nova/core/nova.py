from __future__ import annotations

import asyncio

import nats

from nova.cell.cell import Cell
from nova.config import CELL_NAME, NovaConfig, default_config
from nova.logging import logger

from .gateway import ApiGateway

# Backoff for re-establishing the NATS connection after nats-py has exhausted its own
# built-in reconnect attempts (e.g. when the server keeps closing the connection, as
# happens when an auth-callout-issued user JWT expires and the server stays unreachable
# for longer than nats-py's reconnect window).
_NATS_REOPEN_INITIAL_DELAY_SECS = 2.0
_NATS_REOPEN_MAX_DELAY_SECS = 60.0


class Nova:
    """A high-level Nova client for interacting with robot cells and controllers."""

    def __init__(self, config: NovaConfig | None = None):
        """
        Initialize the Nova client.

        Args:
            config (NovaConfig | None): The Nova configuration.
        """

        self._config = config or default_config

        # many users rely on this private field, we will remove that after some time in v2
        self._api_client = ApiGateway(self._config)
        self.apis = self._api_client

        self.nats = nats.NATS()
        self._nats_reopen_task: asyncio.Task | None = None
        self._closing = False

    @property
    def config(self) -> NovaConfig:
        return self._config

    @property
    def api(self) -> ApiGateway:
        return self._api_client

    def cell(self, cell_id: str = CELL_NAME) -> Cell:
        """Returns the cell object with the given ID."""
        return Cell(self._api_client, cell_id, nats_client=self.nats)

    def is_connected(self) -> bool:
        return self.nats is not None and self.nats.is_connected

    async def open(self):
        """
        Opens the NOVA instance. Configures attached viewers and connects to the NATS server.
        """
        # `nats.NATS.connect()` is not safe to call concurrently. Guard it and make `open()`
        # idempotent so callers (e.g. runner + decorator) can safely call it.
        if self.is_connected():
            return

        self._closing = False

        # Configure any active viewers
        try:
            from nova.viewers import _configure_active_viewers

            _configure_active_viewers(self)
        except ImportError:
            pass

        # Report connection health and keep the connection alive across server-initiated
        # disconnects (e.g. an auth-callout-issued user JWT expiring), without overriding
        # any callback the caller explicitly configured via nats_client_config.
        nats_config = dict(self._config.nats_client_config or {})
        nats_config.setdefault("error_cb", self._on_nats_error)
        nats_config.setdefault("disconnected_cb", self._on_nats_disconnected)
        nats_config.setdefault("reconnected_cb", self._on_nats_reconnected)
        nats_config.setdefault("closed_cb", self._on_nats_closed)

        # ApiGateway doesn't need an explicit connect call, it's initialized in constructor
        await self.nats.connect(**nats_config)

    async def _on_nats_error(self, exc: Exception) -> None:
        logger.warning(f"NATS connection error: {exc}")

    async def _on_nats_disconnected(self) -> None:
        logger.warning("NATS connection lost, attempting to reconnect")

    async def _on_nats_reconnected(self) -> None:
        logger.info("NATS connection restored")

    async def _on_nats_closed(self) -> None:
        """Called by nats-py once it has exhausted its own reconnect attempts.

        This only fires on an unintended, server- or network-caused closure; `close()`
        sets `_closing` first so a deliberate shutdown never triggers it.
        """
        if self._closing:
            return

        logger.error("NATS reconnect attempts exhausted, connection closed; retrying")
        if self._nats_reopen_task is None or self._nats_reopen_task.done():
            self._nats_reopen_task = asyncio.create_task(self._reopen_with_backoff())

    async def _reopen_with_backoff(self) -> None:
        delay = _NATS_REOPEN_INITIAL_DELAY_SECS
        while not self._closing:
            await asyncio.sleep(delay)
            try:
                await self.open()
                logger.info("NATS connection re-established")
                return
            except Exception as exc:
                logger.warning(f"NATS reconnect attempt failed: {exc}")
                delay = min(delay * 2, _NATS_REOPEN_MAX_DELAY_SECS)

    async def connect(self):
        """[Deprecated] Opens the NOVA instance
        Use open() instead.
        """
        await self.open()

    async def close(self):
        """Closes the underlying API client session and NATS client."""
        self._closing = True
        if self._nats_reopen_task is not None:
            self._nats_reopen_task.cancel()
        try:
            if self.nats is not None and self.nats.is_connected:
                await self.nats.drain()
            return await self._api_client.close() if self._api_client is not None else None
        except Exception as e:
            logger.error(f"Error closing Nova: {e}", exc_info=True)

    async def __aenter__(self):
        await self.open()
        return self

    async def __aexit__(self, exc_type, exc_val, exc_tb):
        await self.close()
