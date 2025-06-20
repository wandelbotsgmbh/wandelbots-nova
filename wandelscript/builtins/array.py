from collections.abc import Sequence, Sized

from wandelscript.metamodel import register_builtin_func


@register_builtin_func(name="len")
def len_(a: Sized) -> int:
    """Return the number of elements in a."""
    return len(a)


@register_builtin_func()
def reverse(list_: list) -> Sequence:
    """Reverse the order of a list."""
    list_.reverse()
    return list_
