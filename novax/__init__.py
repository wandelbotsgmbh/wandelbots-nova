try:
    from novax.novax import Novax
except ImportError as exc:
    # Only translate failures caused by the missing optional dependencies; let
    # any other ImportError (a real bug) surface unchanged.
    if exc.name in {"fastapi", "uvicorn", "decouple", "starlette"}:
        raise ImportError(
            "The 'novax' package requires the optional 'novax' extra, which is not installed.\n"
            "Install it with one of:\n"
            "  uv sync --extra novax\n"
            "  uv pip install 'wandelbots-nova[novax]'\n"
            "  pip install 'wandelbots-nova[novax]'"
        ) from exc
    raise

__all__ = ["Novax"]
