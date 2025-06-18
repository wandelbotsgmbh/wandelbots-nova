import importlib

try:
    from importlib import metadata

    version = metadata.version("wandelscript")
except importlib.metadata.PackageNotFoundError:
    # fallback if not installed in dev
    version = "0.0.0-dev"
