from pydantic import BaseModel, Field


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
