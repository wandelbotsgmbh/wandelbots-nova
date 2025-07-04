from __future__ import annotations

from decouple import config

from nova.cell.cell import Cell
from nova.core.gateway import ApiGateway
from nova.events import nats

LOG_LEVEL = config("LOG_LEVEL", default="INFO")
CELL_NAME = config("CELL_NAME", default="cell", cast=str)


# TODO: could also extend NovaDevice
class Nova:
    """A high-level Nova client for interacting with robot cells and controllers."""

    _api_client: ApiGateway

    def __init__(
        self,
        *,
        host: str | None = None,
        access_token: str | None = None,
        username: str | None = None,
        password: str | None = None,
        version: str = "v1",
        verify_ssl: bool = True,
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
        """

        self._api_client = ApiGateway(
            host=host,
            access_token=access_token,
            username=username,
            password=password,
            version=version,
            verify_ssl=verify_ssl,
        )

    def cell(self, cell_id: str = CELL_NAME) -> Cell:
        """Returns the cell object with the given ID."""
        return Cell(self._api_client, cell_id)

    async def close(self):
        """Closes the underlying API client session."""
        # hardcoded for now, later stuff like NATS might become devices
        await nats.close()
        return await self._api_client.close()

    async def __aenter__(self):
        # Configure any active viewers
        try:
            from nova.viewers import _configure_active_viewers

            _configure_active_viewers(self)
        except ImportError:
            pass
        return self

    async def __aexit__(self, exc_type, exc_val, exc_tb):
        await self.close()
