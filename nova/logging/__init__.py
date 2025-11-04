import logging
import sys

from nova.config import LOG_DATETIME_FORMAT, LOG_FORMAT, LOG_LEVEL, LOGGER_NAME

# Setting up the underlying logger
_formatter: logging.Formatter = logging.Formatter(LOG_FORMAT, datefmt=LOG_DATETIME_FORMAT)
_handler: logging.StreamHandler = logging.StreamHandler(sys.stdout)
_handler.setLevel(LOG_LEVEL)
_handler.setFormatter(_formatter)

logger: logging.Logger = logging.getLogger(LOGGER_NAME)
logger.setLevel(LOG_LEVEL)
logger.addHandler(_handler)
