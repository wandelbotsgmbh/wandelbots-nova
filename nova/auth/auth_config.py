from dataclasses import dataclass

from nova.config import NOVA_AUTH0_AUDIENCE, NOVA_AUTH0_CLIENT_ID, NOVA_AUTH0_DOMAIN


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
            domain=NOVA_AUTH0_DOMAIN, client_id=NOVA_AUTH0_CLIENT_ID, audience=NOVA_AUTH0_AUDIENCE
        )

    def is_complete(self) -> bool:
        """Check if all required fields are set and not None"""
        return bool(self.domain and self.client_id and self.audience)

    def get_validated_config(self) -> tuple[str, str, str]:
        """Get validated config values, ensuring they are not None"""
        if not self.is_complete():
            raise ValueError("Auth0 configuration is incomplete")
        return self.domain, self.client_id, self.audience  # type: ignore
