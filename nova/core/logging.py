import logging
import sys

from decouple import config

LOG_LEVEL: str = config("LOG_LEVEL", default="INFO").upper()
LOG_FORMAT: str = config("LOG_FORMAT", default="%(asctime)s [%(levelname)s] %(name)s: %(message)s")
LOG_DATETIME_FORMAT: str = config("LOG_DATETIME_FORMAT", default="%Y-%m-%d %H:%M:%S")
LOGGER_NAME: str = config("LOGGER_NAME", default="wandelbots-nova")

# Setting up the underlying logger
formatter: logging.Formatter = logging.Formatter(LOG_FORMAT, datefmt=LOG_DATETIME_FORMAT)
handler: logging.StreamHandler = logging.StreamHandler(sys.stdout)
handler.setLevel(LOG_LEVEL)
handler.setFormatter(formatter)

logger: logging.Logger = logging.getLogger(LOGGER_NAME)
logger.setLevel(LOG_LEVEL)
logger.addHandler(handler)
