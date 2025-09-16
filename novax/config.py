"""
WARNING: This module is experimental, subject to change, don't use.

Centralized configuration for the `novax` package.

This module reads environment variables once (via `python-decouple`) and exposes
typed constants for use across the package. Keeping these in one place makes it
easy to understand how Novax behaves in different environments and modes.
"""

from __future__ import annotations

from decouple import config as _config

# BASE_PATH: Base prefix used to build program links and derive `APP_NAME`.
# - Type: str
# - Default: "/default/novax"
# - Example: "/cell/my-app"
# - Notes: The last path segment becomes `APP_NAME`.
BASE_PATH: str = _config("BASE_PATH", default="/default/novax")

# NOVAX_MOUNT_PATH: Sub-path under which the Novax API is mounted when embedded
# in another service (e.g., a gateway).
# - Type: str | None
# - Default: None
# - Example: "novax"
# - Notes: If set, links become `BASE_PATH/{NOVAX_MOUNT_PATH}/programs/...`.
NOVAX_MOUNT_PATH: str | None = _config("NOVAX_MOUNT_PATH", default=None)

# CELL_NAME: The Nova cell identifier used for program store sync and event
# routing subjects.
# - Type: str
# - Default: ""
# - Example: "cell"
# - Notes: If empty, Novax won't attach its lifespan to sync programs to store.
CELL_NAME: str = _config("CELL_NAME", default="")


def _derive_app_name(base_path: str) -> str:
    # Remove trailing slash to avoid empty segments, then split
    path = base_path.rstrip("/")
    if not path or "/" not in path:
        return "novax"
    return path.split("/")[-1] or "novax"


# APP_NAME: The application name derived from `BASE_PATH`'s last path segment.
# - Type: str
# - Default behavior: If `BASE_PATH` is blank or has no slashes, falls back to
#   "novax"; otherwise uses the last segment.
APP_NAME: str = _derive_app_name(BASE_PATH)

__all__ = ["BASE_PATH", "NOVAX_MOUNT_PATH", "CELL_NAME", "APP_NAME"]
