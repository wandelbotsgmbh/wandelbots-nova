from __future__ import annotations

import nats

from nova.cell.cell import Cell
from nova.config import CELL_NAME, NovaConfig, default_config

from .gateway import ApiGateway


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
        return self.nats.is_connected

    async def connect(self):
        # ApiGateway doesn't need an explicit connect call, it's initialized in constructor
        await self.nats.connect(**(self._config.nats_client_config or {}))

    async def close(self):
        """Closes the underlying API client session and NATS client."""
        await self.nats.drain()
        return await self._api_client.close()

    async def __aenter__(self):
        # Configure any active viewers
        try:
            from nova.viewers import _configure_active_viewers

            _configure_active_viewers(self)
        except ImportError:
            pass

        await self.connect()

        return self

    async def __aexit__(self, exc_type, exc_val, exc_tb):
        await self.close()
