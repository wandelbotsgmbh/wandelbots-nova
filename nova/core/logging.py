"""Deprecated: use `nova.logging` instead of `nova.core.logging`.

This module re-exports the new logging definitions and issues a warning to help
users migrate. It will be removed in a future release.
"""

from warnings import warn

import nova.logging as _new_logging_module

warn(
    "`nova.core.logging` is deprecated and will be removed in a future release; import from `nova.logging` instead.",
    FutureWarning,
    stacklevel=2,
)

LOG_LEVEL = _new_logging_module._LOG_LEVEL
LOG_FORMAT = _new_logging_module._LOG_FORMAT
LOG_DATETIME_FORMAT = _new_logging_module._LOG_DATETIME_FORMAT
LOGGER_NAME = _new_logging_module._LOGGER_NAME
formatter = _new_logging_module._formatter
handler = _new_logging_module._handler
logger = _new_logging_module.logger
