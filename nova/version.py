import importlib.metadata

try:
    version = importlib.metadata.version("wandelbots-nova")
except importlib.metadata.PackageNotFoundError:
    # fallback if not installed in dev
    version = "0.0.0-dev"
