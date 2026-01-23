from importlib import metadata

version: str
try:
    version = metadata.version("wandelbots-nova")
except metadata.PackageNotFoundError:
    # fallback if not installed in dev
    version = "0.0.0-dev"
