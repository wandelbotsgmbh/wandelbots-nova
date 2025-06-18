import math

from wandelscript.metamodel import register_builtin_func

for func_name in ["sin", "cos", "tan", "sinh", "cosh", "tanh", "exp", "log", "sqrt"]:
    register_builtin_func(func_name)(getattr(math, func_name))


@register_builtin_func()
def modulo(a: int, b: int) -> int:
    """Modulo operator"""
    return a % b


@register_builtin_func(name="divmod")
def divmod_(a: int, b: int) -> tuple[int, int]:
    """Return a tuple containing the quotient and the remainder when argument1 (dividend) is divided by argument2
    (divisor).

    Args:
        a: The dividend
        b: The divisor

    Returns:
        A tuple containing the quotient and the remainder

    """
    return divmod(a, b)


@register_builtin_func()
def intdiv(a: int, b: int) -> int:
    """Return the integer division."""
    return a // b


@register_builtin_func()
def power(a: float, b: float) -> float:
    """Return a raised to the power of b."""
    return a**b


@register_builtin_func(name="abs")
def abs_(a) -> float:
    """Return the absolute value of a."""
    return abs(a)


@register_builtin_func(name="round")
def round_(a: float) -> int:
    """Return a rounded to the next integer."""
    return int(round(a))


@register_builtin_func()
def ceil(a: float) -> int:
    """Return the smallest integer greater than or equal to a."""
    return int(math.ceil(a))


@register_builtin_func()
def floor(a: float) -> int:
    """Return the largest integer less than or equal to a."""
    return int(math.floor(a))
