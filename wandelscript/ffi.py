"""Foreign Function Interface.

Use to register your own or 3rd party Python functions with Wandelscript
programs so that you can use those functions within your Wandelscript code.
"""

import inspect
from dataclasses import dataclass, is_dataclass
from functools import wraps
from typing import Any, Callable, TypeVar

import pydantic

from wandelscript.datatypes import as_builtin_type


@dataclass(frozen=True)
class ForeignFunction:
    """References a function that can be used inside a Wandelscript.

    Args:
        function: Handle to an arbitrary function, even an async function.
        pass_context (bool): Whether or not the function takes the current wandelscript context
            as the first parameter. Defaults to False.
    """

    function: Callable
    _name: str | None = None
    pass_context: bool = False

    def __post_init__(self):
        if not isinstance(self.function, Callable):
            raise TypeError(
                f"Function of {self} must be a Callable, got {type(self.function).__name__}"
            )

    @property
    def name(self) -> str:
        return self._name or self.function.__name__


FF_MARKER_ATTRIBUTE = "_wandelscript_foreign_function"

F = TypeVar("F", bound=Callable[..., Any])


def foreign_function(
    name: str | None = None, pass_context: bool = False, autoconvert_types: bool = True
) -> Callable[[F], F]:
    """Decorator to mark a function as a foreign function for Wandelscript.

    Args:
        fn: The function to mark.

    Returns:
        Callable: The same function, marked with ws_ff attribute.
    """

    def decorator(fn: F) -> F:
        if autoconvert_types:
            sig = inspect.signature(fn)

            @wraps(fn)
            def wrapper(*args: Any, **kwargs: Any) -> Any:
                bound = sig.bind_partial(*args, **kwargs)
                converted = {}
                for name, value in bound.arguments.items():
                    param = sig.parameters[name]
                    anno = param.annotation
                    if is_dataclass(anno) and isinstance(anno, type) and isinstance(value, dict):
                        converted[name] = anno(**value)
                    elif (
                        inspect.isclass(anno)
                        and issubclass(anno, pydantic.BaseModel)
                        and isinstance(value, dict)
                    ):
                        converted[name] = anno.model_validate(value)
                    else:
                        converted[name] = value

                result = fn(**converted)
                return as_builtin_type(result)

            fn_obj: F = wrapper  # type: ignore
        else:
            fn_obj = fn

        fn_name = name if name is not None else fn.__name__
        setattr(fn_obj, FF_MARKER_ATTRIBUTE, ForeignFunction(fn_obj, fn_name, pass_context))
        return fn_obj

    return decorator


def is_foreign_function(obj: object) -> bool:
    """Check if the object is a foreign function.

    Args:
        obj: The object to check.

    Returns:
        bool: True if the object is a foreign function, False otherwise.
    """
    return hasattr(obj, FF_MARKER_ATTRIBUTE) and isinstance(
        getattr(obj, FF_MARKER_ATTRIBUTE), ForeignFunction
    )


def get_foreign_function(obj: object) -> ForeignFunction | None:
    """Get the foreign function from an object.

    Args:
        obj: The object to check.

    Returns:
        ForeignFunction | None: The foreign function if it exists, None otherwise.
    """
    return getattr(obj, FF_MARKER_ATTRIBUTE, None)
