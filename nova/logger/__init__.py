"""Deprecated: use `nova.logging` instead of `nova.logger`.

This module remains as a compatibility shim and will be removed in a future
release. Import from `nova.logging` to silence this warning.
"""

from warnings import warn

from nova.logging import (
    _LOG_DATETIME_FORMAT,
    _LOG_FORMAT,
    _LOG_LEVEL,
    _LOGGER_NAME,
    _formatter,
    _handler,
    logger,
)

warn("`nova.logger` is deprecated; use `nova.logging` instead.", DeprecationWarning, stacklevel=2)
