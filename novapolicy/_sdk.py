"""Internal adapter for nova SDK internals.

Centralizes all access to private/undocumented attributes of the nova SDK
so that if the SDK changes, only this file needs updating.

Do NOT import this from outside the policy package.

.. note::
    This module accesses ``MotionGroup._api_client``, ``._cell``, and
    ``._controller_id`` which are **not part of the public SDK API**.
    These attributes may change or be removed in future SDK releases.
    This package pins ``wandelbots-nova>=5.1.0,<6`` to limit exposure.
    If the SDK changes these internals, only this file needs updating.
"""

from __future__ import annotations

from typing import TYPE_CHECKING

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

    ``path`` is relative to the gateway's configured base, which already
    includes the API prefix (e.g. ``http://host/api/v2``), so it starts at
    ``/cells/...``. Repeating the prefix yields a 404 on the handshake.

    Mirrors how the generated websocket endpoints connect: basic auth embedded
    in the URL, OAuth2 as an ``Authorization`` header. One deliberate
    difference: the generated code sends a literal masked ``"******"`` as the
    bearer value, a code-gen artifact, so the real token is sent here instead.
    """
    config = get_api_gateway(mg)._api_client.configuration  # type: ignore[attr-defined]
    host = config.host
    headers: dict[str, str] = {}
    if host.startswith("https://"):
        bare = host.removeprefix("https://")
        if config.username:
            return f"wss://{config.username}:{config.password}@{bare}{path}", headers
        if config.access_token:
            headers["Authorization"] = f"Bearer {config.access_token}"
        return f"wss://{bare}{path}", headers
    return f"ws://{host.removeprefix('http://')}{path}", headers
