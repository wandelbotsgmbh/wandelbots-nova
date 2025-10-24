"""Deprecated: use `nova.logging` instead of `nova.core.logging`.

This module re-exports the new logging definitions and issues a warning to help
users migrate. It will be removed in a future release.
"""

from warnings import warn

import nova.logging as _new_logging_module

# backward compatibility
from nova.config import LOG_DATETIME_FORMAT, LOG_FORMAT, LOG_LEVEL, LOGGER_NAME  # noqa: F401

warn(
    "`nova.core.logging` is deprecated and will be removed in a future release; import from `nova.logging` instead.",
    FutureWarning,
    stacklevel=2,
)

formatter = _new_logging_module._formatter
handler = _new_logging_module._handler
logger = _new_logging_module.logger
