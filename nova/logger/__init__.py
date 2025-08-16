import logging
import sys

from decouple import config

_LOG_LEVEL: str = config("LOG_LEVEL", default="INFO").upper()
_LOG_FORMAT: str = config("LOG_FORMAT", default="%(asctime)s [%(levelname)s] %(name)s: %(message)s")
_LOG_DATETIME_FORMAT: str = config("LOG_DATETIME_FORMAT", default="%Y-%m-%d %H:%M:%S")
_LOGGER_NAME: str = config("LOGGER_NAME", default="wandelbots-nova")

# Setting up the underlying logger
_formatter: logging.Formatter = logging.Formatter(_LOG_FORMAT, datefmt=_LOG_DATETIME_FORMAT)
_handler: logging.StreamHandler = logging.StreamHandler(sys.stdout)
_handler.setLevel(_LOG_LEVEL)
_handler.setFormatter(_formatter)

logger: logging.Logger = logging.getLogger(_LOGGER_NAME)
logger.setLevel(_LOG_LEVEL)
logger.addHandler(_handler)
