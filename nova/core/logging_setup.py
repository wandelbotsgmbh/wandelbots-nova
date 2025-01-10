import sys

from loguru import logger


def configure_logging(log_level: str = "INFO"):
    logger.remove()
    logger.add(
        sys.stderr,
        level=log_level,
        format="<green>{time}</green> | <level>{level}</level> | <level>{message}</level>",
    )

    return logger
