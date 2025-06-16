from typing import SupportsIndex

from wandelscript.metamodel import register_builtin_func


@register_builtin_func()
def replace(value: str, old: str, new: str, *, count: SupportsIndex = -1) -> str:
    """Replace a substring with another substring"""
    return value.replace(old, new, count)


@register_builtin_func()
def to_string(value) -> str:
    """
    This function is required due to a bug in wandelscript. The Fanuc TCP '3' is converted to float 3.0
    The reason is the ordering within pyjectory.serializer.FlatElementType.

    Args:
        value: to be converted to a string

    Returns:
        string without separator and trailing zeros
    """
    if isinstance(value, float) and value.is_integer():
        return str(int(value))
    return str(value)
