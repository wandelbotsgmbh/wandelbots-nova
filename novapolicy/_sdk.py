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
