# Provide autogenerated NOVA API client with lazy loading
from typing import Any

_lazy_imports: dict[str, Any] = {}


def __getattr__(name: str) -> Any:
    """Lazy import wandelbots_api_client components on first access."""
    if name in _lazy_imports:
        return _lazy_imports[name]

    import wandelbots_api_client as wb

    _lazy_imports.update(
        {
            "models": wb.models,
            "api": wb.api,
            "api_client": wb.api_client,
            "configuration": wb.configuration,
            "exceptions": wb.exceptions,
            "ApiResponse": wb.ApiResponse,
            "ApiClient": wb.ApiClient,
            "Configuration": wb.Configuration,
            "OpenApiException": wb.OpenApiException,
            "ApiTypeError": wb.ApiTypeError,
            "ApiValueError": wb.ApiValueError,
            "ApiKeyError": wb.ApiKeyError,
            "ApiAttributeError": wb.ApiAttributeError,
            "ApiException": wb.ApiException,
        }
    )

    if name in _lazy_imports:
        return _lazy_imports[name]

    if hasattr(wb, name):
        _lazy_imports[name] = getattr(wb, name)
        return _lazy_imports[name]

    raise AttributeError(f"module '{__name__}' has no attribute '{name}'")
