from __future__ import annotations

from decouple import config

from nova.cell.cell import Cell
from nova.core.gateway import ApiGateway
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
