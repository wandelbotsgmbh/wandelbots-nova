import logging
import sys

from nova.config import LOG_DATETIME_FORMAT, LOG_FORMAT, LOG_LEVEL


def configure_logging():
    # Force root level (overrides any earlier basicConfig)
    logging.basicConfig(level=LOG_LEVEL, force=True)

    # Silence chatty libs explicitly
    logging.getLogger("websockets").setLevel(LOG_LEVEL)
    logging.getLogger("websockets.client").setLevel(LOG_LEVEL)
    logging.getLogger("websockets.protocol").setLevel(LOG_LEVEL)

    logging.getLogger("nats").setLevel(LOG_LEVEL)  # python-nats

    # By default python prints to stderr and has no date format in logs
    # this configuration is for beginner users to have a better logging experience out of the box
    # experienced users can override this behaviour by configuring the logger themselves after importing nova module
    _formatter = logging.Formatter(LOG_FORMAT, datefmt=LOG_DATETIME_FORMAT)
    _handler = logging.StreamHandler(sys.stdout)
    _handler.setLevel(LOG_LEVEL)
    _handler.setFormatter(_formatter)

    logger = logging.getLogger("nova")
    logger.setLevel(LOG_LEVEL)
    logger.addHandler(_handler)
    return logger


logger = configure_logging()
