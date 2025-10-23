from urllib.parse import urlparse

from decouple import config
from pydantic import BaseModel, Field, model_validator

LOG_LEVEL = config("LOG_LEVEL", default="INFO")
CELL_NAME = config("CELL_NAME", default="cell", cast=str)
NOVA_API = config("NOVA_API", default=None)
NOVA_ACCESS_TOKEN = config("NOVA_ACCESS_TOKEN", default=None)
NOVA_USERNAME = config("NOVA_USERNAME", default=None)
NOVA_PASSWORD = config("NOVA_PASSWORD", default=None)
INTERNAL_CLUSTER_NOVA_API = "http://api-gateway.wandelbots.svc.cluster.local:8080"


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
    access_token: str | None = Field(default=None, description="Access token for Nova API.")
    username: str | None = Field(default=None, deprecated=True)
    password: str | None = Field(default=None, deprecated=True)
    verify_ssl: bool = Field(default=True)
    nats_client_config: dict | None = Field(
        default=None,
        description="Client configuration to pass to the nats library. See: https://nats-io.github.io/nats.py/modules.html#nats.aio.client.Client.connect",
    )

    @model_validator(mode="after")
    def calculate_nats_connection_string(self) -> "NovaConfig":
        """
        Automatically derive the NATS client configuration if not explicitly set.
        """
        # user has explicitly set the servers
        if self.nats_client_config is not None and "servers" in self.nats_client_config:
            return self

        self.nats_client_config = self.nats_client_config or {}

        # there is an environment variable NATS_BROKER set, use that
        nats_broker_env = config("NATS_BROKER", default=None)
        if nats_broker_env:
            self.nats_client_config["servers"] = nats_broker_env
            return self

        # there is no host set, cannot derive NATS config
        if not self.host:
            return self

        parsed_host = urlparse(self.host)
        if parsed_host.scheme == "http":
            self.nats_client_config["servers"] = (
                f"ws://{parsed_host.hostname}:{parsed_host.port or 80}/api/nats"
            )
            return self

        if parsed_host.scheme == "https" and self.access_token:
            self.nats_client_config["servers"] = (
                f"wss://{self.access_token}@{parsed_host.hostname}:{parsed_host.port or 443}/api/nats"
            )
            return self

        # for backward compatiblity
        if self.host and self.access_token and not parsed_host.scheme:
            self.nats_client_config["servers"] = (
                f"wss://{self.access_token}@{self.host}:{443}/api/nats"
            )

        return self


default_config = NovaConfig(
    host=NOVA_API, access_token=NOVA_ACCESS_TOKEN, username=NOVA_USERNAME, password=NOVA_PASSWORD
)
