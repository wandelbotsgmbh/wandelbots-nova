from __future__ import annotations

from typing import Any, Self

import nats

from nova.cell.cell import Cell
from nova.config import CELL_NAME, NovaConfig, default_config
from nova.logging import logger

from .gateway import ApiGateway


class Nova:
    """A high-level Nova client for interacting with robot cells and controllers."""

    _api_client: ApiGateway
    apis: ApiGateway
    nats: nats.NATS

    def __init__(self, config: NovaConfig | None = None) -> None:
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

    async def open(self) -> None:
        """
        Opens the NOVA instance. Configures attached viewers and connects to the NATS server.
        """
        # `nats.NATS.connect()` is not safe to call concurrently. Guard it and make `open()`
        # idempotent so callers (e.g. runner + decorator) can safely call it.
        if self.is_connected():
            return

        # Configure any active viewers
        try:
            from nova.viewers import _configure_active_viewers

            _configure_active_viewers(self)
        except ImportError:
            pass

        # ApiGateway doesn't need an explicit connect call, it's initialized in constructor
        await self.nats.connect(**(self._config.nats_client_config or {}))

    async def connect(self) -> None:
        """[Deprecated] Opens the NOVA instance
        Use open() instead.
        """
        await self.open()

    async def close(self) -> None:
        """Closes the underlying API client session and NATS client."""
        try:
            if self.nats is not None and self.nats.is_connected:
                await self.nats.drain()
            return await self._api_client.close() if self._api_client is not None else None
        except Exception as e:
            logger.error(f"Error closing Nova: {e}", exc_info=True)

    async def __aenter__(self) -> Self:
        await self.open()
        return self

    async def __aexit__(
        self, exc_type: type[BaseException] | None, exc_val: BaseException | None, exc_tb: Any
    ) -> None:
        await self.close()
