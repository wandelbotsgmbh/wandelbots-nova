# nova/experimental/api/operation_types.py

from dataclasses import dataclass
from typing import Any, Awaitable, Callable


@dataclass
class APIOperation:
    """
    Represents a single API operation with separate v1 and v2 implementations.

    Each implementation is an async function with the signature:
        def impl(client: DeclarativeApiClient, *args, **kwargs) -> Any
    """

    name: str
    v1_impl: Callable[..., Awaitable[Any]]
    v2_impl: Callable[..., Awaitable[Any]]
