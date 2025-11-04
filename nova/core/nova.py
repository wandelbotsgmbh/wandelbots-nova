from __future__ import annotations

from nova.cell.cell import Cell

# backward compatibility
from nova.config import (  # noqa: F401
    CELL_NAME,
    LOG_LEVEL,
    NOVA_ACCESS_TOKEN,
    NOVA_API,
    NOVA_PASSWORD,
    NOVA_USERNAME,
    NovaConfig,
    default_config,
)
from nova.core.gateway import ApiGateway
from nova.nats import NatsClient


class Nova:
    """A high-level Nova client for interacting with robot cells and controllers."""

    def __init__(self, config: NovaConfig | None = None):
        """
        Initialize the Nova client.

        Args:
            config (NovaConfig | None): The Nova configuration.
        """

        self._config = config or default_config
        self._api_client = ApiGateway(
            host=self._config.host,
            access_token=self._config.access_token,
            username=self._config.username,
            password=self._config.password,
            verify_ssl=self._config.verify_ssl,
        )
        self.nats = NatsClient(nats_client_config=self._config.nats_client_config)

    @property
    def config(self) -> NovaConfig:
        return self._config

    def cell(self, cell_id: str = CELL_NAME) -> Cell:
        """Returns the cell object with the given ID."""
        return Cell(self._api_client, cell_id, nats_client=self.nats)

    def is_connected(self) -> bool:
        return self.nats.is_connected()

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

        return self

    async def __aexit__(self, exc_type, exc_val, exc_tb):
        await self.close()
