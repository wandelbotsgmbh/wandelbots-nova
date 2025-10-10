from __future__ import annotations

from decouple import config as env_config
from pydantic import BaseModel, Field

from nova.cell.cell import Cell
from nova.core.gateway import ApiGateway
from nova.nats import NatsClient

LOG_LEVEL = env_config("LOG_LEVEL", default="INFO")
CELL_NAME = env_config("CELL_NAME", default="cell", cast=str)
NOVA_API = env_config("NOVA_API", default=None)
NOVA_ACCESS_TOKEN = env_config("NOVA_ACCESS_TOKEN", default=None)
NOVA_USERNAME = env_config("NOVA_USERNAME", default=None)
NOVA_PASSWORD = env_config("NOVA_PASSWORD", default=None)


class NovaConfig(BaseModel):
    """
    Configuration for connecting to the Nova API.

    Args:
        host (str | None): The Nova API host.
        access_token (str | None): An access token for the Nova API.
        username (str | None): [Deprecated] Username to authenticate with the Nova API.
        password (str | None): [Deprecated] Password to authenticate with the Nova API.
        version (str): The API version to use (default: "v1").
        verify_ssl (bool): Whether or not to verify SSL certificates (default: True).
        nats_client_config (dict | None): Configuration dictionary for NATS client.
    """

    host: str | None = Field(default=None, description="Nova API host.")
    access_token: str | None = None
    username: str | None = Field(default=None, deprecated=True)
    password: str | None = Field(default=None, deprecated=True)
    verify_ssl: bool = Field(default=True)
    nats_client_config: dict | None = None


class Nova:
    """A high-level Nova client for interacting with robot cells and controllers."""

    def __init__(self, *, config: NovaConfig | None = None):
        """
        Initialize the Nova client.

        Args:
            config (NovaConfig | None): The Nova configuration.
        """

        config = config or NovaConfig(
            host=NOVA_API,
            access_token=NOVA_ACCESS_TOKEN,
            username=NOVA_USERNAME,
            password=NOVA_PASSWORD,
        )

        self._config = config
        self._api_client = ApiGateway(
            host=config.host,
            access_token=config.access_token,
            username=config.username,
            password=config.password,
            verify_ssl=config.verify_ssl,
        )

        self.nats = NatsClient(
            host=config.host,
            access_token=config.access_token,
            nats_client_config=config.nats_client_config,
        )

    @property
    def config(self) -> NovaConfig:
        return self._config

    def cell(self, cell_id: str = CELL_NAME) -> Cell:
        """Returns the cell object with the given ID."""
        return Cell(self._api_client, cell_id, nats_client=self.nats)

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
