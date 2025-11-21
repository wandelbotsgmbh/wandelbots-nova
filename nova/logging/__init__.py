import logging
import sys

from nova.config import LOG_DATETIME_FORMAT, LOG_FORMAT, LOG_LEVEL

# By default python prints to stderr and has no date format in logs
# this configuration is for beginner users to have a better logging experience out of the box
# experinced users can override this behaviour by configuring the logger themselfs after importing nova module
_formatter: logging.Formatter = logging.Formatter(LOG_FORMAT, datefmt=LOG_DATETIME_FORMAT)
_handler: logging.StreamHandler = logging.StreamHandler(sys.stdout)
_handler.setLevel(LOG_LEVEL)
_handler.setFormatter(_formatter)


# the logger name is specifically set to "nova" so that other modules can do:
# import logging
# logger = logging.getLogger(__name__)
# and use this logger as parent
logger: logging.Logger = logging.getLogger("nova")
logger.setLevel(LOG_LEVEL)
logger.addHandler(_handler)
