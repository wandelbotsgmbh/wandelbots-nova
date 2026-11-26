"""The one place that reads private nova SDK attributes (``MotionGroup._api_client``,
``._cell``, ``._controller_id``); internal to novapolicy."""

from __future__ import annotations

from typing import TYPE_CHECKING
from urllib.parse import quote

if TYPE_CHECKING:
    from nova.cell.motion_group import MotionGroup
    from nova.core.gateway import ApiGateway


def get_api_gateway(mg: MotionGroup) -> ApiGateway:
    """Get the API gateway from a motion group."""
    return mg._api_client  # type: ignore[return-value]


def get_cell(mg: MotionGroup) -> str:
    """Get the cell identifier from a motion group."""
    return mg._cell  # type: ignore[return-value]


def get_controller_id(mg: MotionGroup) -> str:
    """Get the controller identifier from a motion group."""
    return mg._controller_id  # type: ignore[return-value]


def get_ws_target(mg: MotionGroup, path: str) -> tuple[str, dict[str, str]]:
    """Websocket URL and headers for an API path, from the gateway's own config.

    ``path`` starts at ``/cells/...``: the configured base already includes ``/api/v2``. Basic
    auth goes in the URL, an OAuth2 token (the real one, not the generated ``"******"``) in
    ``Authorization``.
    """
    config = get_api_gateway(mg)._api_client.configuration  # type: ignore[attr-defined]
    host = config.host
    if host.startswith("https://"):
        scheme, bare = "wss", host.removeprefix("https://")
    else:
        scheme, bare = "ws", host.removeprefix("http://")
    headers: dict[str, str] = {}
    # Credentials apply to plain http hosts too, as for the REST calls.
    if config.username:
        user = quote(config.username, safe="")
        password = quote(config.password or "", safe="")
        return f"{scheme}://{user}:{password}@{bare}{path}", headers
    if config.access_token:
        headers["Authorization"] = f"Bearer {config.access_token}"
    return f"{scheme}://{bare}{path}", headers
