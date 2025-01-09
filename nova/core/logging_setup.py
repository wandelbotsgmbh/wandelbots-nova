import sys
from loguru import logger


def configure_logging(log_level: str = "INFO"):
    # Remove any existing handlers (including default ones)
    logger.remove()

    # Add your console handler (you can also add file handlers here)
    logger.add(
        sys.stderr, level=log_level, format="<green>{time}</green> | <level>{message}</level>"
    )

    return logger
