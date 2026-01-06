import logging
import sys

from nova.config import LOG_DATETIME_FORMAT, LOG_FORMAT, LOG_LEVEL


def configure_logging():
    # Configure a single, consistent handler (stdout + our formatter).
    # Using `force=True` ensures we don't accumulate handlers across imports / reconfiguration.
    _formatter = logging.Formatter(LOG_FORMAT, datefmt=LOG_DATETIME_FORMAT)
    _handler = logging.StreamHandler(sys.stdout)
    _handler.setLevel(LOG_LEVEL)
    _handler.setFormatter(_formatter)

    logging.basicConfig(level=LOG_LEVEL, handlers=[_handler], force=True)

    # Silence chatty libs explicitly
    logging.getLogger("websockets").setLevel(LOG_LEVEL)
    logging.getLogger("websockets.client").setLevel(LOG_LEVEL)
    logging.getLogger("websockets.protocol").setLevel(LOG_LEVEL)
    logging.getLogger("nats").setLevel(LOG_LEVEL)  # python-nats

    # The `nova` logger itself should simply propagate to the root handler configured above.
    logger = logging.getLogger("nova")
    logger.setLevel(LOG_LEVEL)
    return logger


logger = configure_logging()
