import time
from typing import Any

from nova.types import MotionSettings
from wandelscript.builtins import array, assoc, controller, fetch, math, pose, string, wait
from wandelscript.datatypes import Frame, as_builtin_type
from wandelscript.metamodel import register_builtin_func

register_builtin_func()(as_builtin_type)


@register_builtin_func(name="int")
def convert_to_int(value: float) -> int:
    r"""Convert float to int

    Args:
        value: the input float

    Returns:
        The input converted to an integer

    Example:
    >>> import asyncio
    >>> from wandelscript.metamodel import run_program
    >>> code = 'a = int(5.63)'
    >>> store = asyncio.run(run_program(code)).store
    >>> store['a']
    5
    """
    return int(value)


@register_builtin_func(name="string")
def convert_to_string(value: Any) -> str:
    """Convert any value to string"""
    return str(value)


@register_builtin_func(name="time")
def time_() -> float:
    """Return the current time in milliseconds."""
    return 1000 * time.time()


@register_builtin_func()
def python_print(a):
    """Print the input value with Python builtin print."""
    print(a)


def make_settings_modifier(name):
    varname = MotionSettings.field_to_varname(name)

    def modifier(ctx, val):
        previous = ctx.store.get(varname, MotionSettings.model_fields[name].default)
        ctx.store[varname] = val

        async def on_exit(_store):
            ctx.store[varname] = previous

        return on_exit

    return modifier


for field_name in MotionSettings.model_fields:
    register_builtin_func(name=field_name, pass_context=True)(make_settings_modifier(field_name))
    match field_name:
        case "tcp_velocity_limit":
            register_builtin_func(name="velocity", pass_context=True)(
                make_settings_modifier(field_name)
            )
        case "tcp_acceleration_limit":
            register_builtin_func(name="acceleration", pass_context=True)(
                make_settings_modifier(field_name)
            )
        case "position_zone_radius":
            register_builtin_func(name="blending", pass_context=True)(
                make_settings_modifier(field_name)
            )
        case _:
            pass


@register_builtin_func(pass_context=True)
def tcp(context, tcp_: str | Frame):
    """Set the TCP

    Args:
        context: The execution context
        tcp_: The name of the TCP or a Frame object

    Returns:
        A function that can be used to restore the previous TCP
    """
    tcp_name = tcp_ if isinstance(tcp_, str) else tcp_.name
    previous = context.store.get("__tcp__", None)
    context.store["__tcp__"] = tcp_name

    async def on_exit(_store):
        context.store["__tcp__"] = previous

    return on_exit


__all__ = [
    "convert_to_int",
    "convert_to_string",
    "time_",
    "python_print",
    "make_settings_modifier",
    "tcp",
    "array",
    "assoc",
    "controller",
    "fetch",
    "math",
    "pose",
    "string",
    "wait",
]
