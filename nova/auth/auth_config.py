from dataclasses import dataclass

from decouple import config


@dataclass
class Auth0Config:
    """Configuration for Auth0 authentication"""

    domain: str | None = None
    client_id: str | None = None
    audience: str | None = None

    @classmethod
    def from_env(cls) -> "Auth0Config":
        """Create Auth0Config from environment variables"""
        return cls(
            domain=config("NOVA_AUTH0_DOMAIN", default=None),
            client_id=config("NOVA_AUTH0_CLIENT_ID", default=None),
            audience=config("NOVA_AUTH0_AUDIENCE", default=None),
        )

    def is_complete(self) -> bool:
        """Check if all required fields are set and not None"""
        return bool(self.domain and self.client_id and self.audience)

    def get_validated_config(self) -> tuple[str, str, str]:
        """Get validated config values, ensuring they are not None"""
        if not self.is_complete():
            raise ValueError("Auth0 configuration is incomplete")
        return self.domain, self.client_id, self.audience  # type: ignore
