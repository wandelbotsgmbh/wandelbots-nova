from decouple import config

from nova.logging import logger

# Read BASE_PATH environment variable and extract app name
BASE_PATH = config("BASE_PATH", default="/default/novax")
APP_NAME = BASE_PATH.split("/")[-1] if "/" in BASE_PATH else "novax"
logger.info(f"Extracted app name '{APP_NAME}' from BASE_PATH '{BASE_PATH}'")

# Create nats programs bucket name
CELL_NAME = config("CELL_NAME", default="")
