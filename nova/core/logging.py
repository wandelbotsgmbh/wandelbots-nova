import logging
import sys
from decouple import config
from typing import Any

LOG_LEVEL: str = config('LOG_LEVEL', default='INFO').upper()
LOG_FORMAT: str = config('LOG_FORMAT', default='%(asctime)s [%(levelname)s] %(name)s: %(message)s')
LOG_DATETIME_FORMAT: str = config('LOG_DATETIME_FORMAT', default='%Y-%m-%d %H:%M:%S')
LOGGER_NAME: str = config('LOGGER_NAME', default='wandelbots-nova')

# Setting up the underlying logger
_formatter: logging.Formatter = logging.Formatter(LOG_FORMAT, datefmt=LOG_DATETIME_FORMAT)
_handler: logging.StreamHandler = logging.StreamHandler(sys.stderr)
_handler.setLevel(LOG_LEVEL)
_handler.setFormatter(_formatter)

_logger: logging.Logger = logging.getLogger(LOGGER_NAME)
_logger.setLevel(LOG_LEVEL)
_logger.addHandler(_handler)

class LoggerWrapper:
    """
    A simple logger wrapper to restrict functionality.
    This will give us more control when we need to change something.
    """
    def __init__(self, logger: logging.Logger) -> None:
        self._logger: logging.Logger = logger

    def debug(self, msg: str, *args: Any, **kwargs: Any) -> None:
        self._logger.debug(msg, *args, **kwargs)

    def info(self, msg: str, *args: Any, **kwargs: Any) -> None:
        self._logger.info(msg, *args, **kwargs)

    def warning(self, msg: str, *args: Any, **kwargs: Any) -> None:
        self._logger.warning(msg, *args, **kwargs)

    def error(self, msg: str, *args: Any, **kwargs: Any) -> None:
        self._logger.error(msg, *args, **kwargs)

    def critical(self, msg: str, *args: Any, **kwargs: Any) -> None:
        self._logger.critical(msg, *args, **kwargs)

    def set_level(self, level: int) -> None:
        self._logger.setLevel(level)

logger: LoggerWrapper = LoggerWrapper(_logger)
