from decouple import config

from nova.logging import logger

# Read BASE_PATH environment variable and extract app name
BASE_PATH = config("BASE_PATH", default="/default/novax")
APP_NAME = BASE_PATH.split("/")[-1] if "/" in BASE_PATH else "novax"
logger.info(f"Extracted app name '{APP_NAME}' from BASE_PATH '{BASE_PATH}'")

# Create nats programs bucket name
CELL_NAME = config("CELL_NAME", default="")

# Optional public base URL (e.g. an ngrok tunnel) the NOVA service-manager should
# route start/stop requests to. Used for local dev so programs show up and run in
# remote NOVA without installing the app.
PROGRAM_ENDPOINT_URL = config("PROGRAM_ENDPOINT_URL", default="")
