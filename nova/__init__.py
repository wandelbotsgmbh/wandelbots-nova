from typing import Any

from nova.version import version

_lazy_imports: dict[str, Any] = {}


def __getattr__(name: str) -> Any:
    """Lazy import nova components on first access."""
    if name in _lazy_imports:
        return _lazy_imports[name]

    if name == "actions":
        from nova import actions

        _lazy_imports["actions"] = actions
        return actions
    elif name == "types":
        from nova import types

        _lazy_imports["types"] = types
        return types
    elif name == "api":
        import nova.api as api

        _lazy_imports["api"] = api
        return api
    elif name == "Cell":
        from nova.cell.cell import Cell

        _lazy_imports["Cell"] = Cell
        return Cell
    elif name == "Controller":
        from nova.core.controller import Controller

        _lazy_imports["Controller"] = Controller
        return Controller
    elif name == "logger":
        from nova.core.logging import logger

        _lazy_imports["logger"] = logger
        return logger
    elif name in ["MotionGroup", "combine_trajectories"]:
        from nova.core.motion_group import MotionGroup, combine_trajectories

        _lazy_imports.update(
            {"MotionGroup": MotionGroup, "combine_trajectories": combine_trajectories}
        )
        return _lazy_imports[name]
    elif name == "speed_up_movement_controller":
        from nova.core.movement_controller import speed_up as speed_up_movement_controller

        _lazy_imports["speed_up_movement_controller"] = speed_up_movement_controller
        return speed_up_movement_controller
    elif name == "Nova":
        from nova.core.nova import Nova

        _lazy_imports["Nova"] = Nova
        return Nova
    elif name == "program":
        from nova.runtime.function import wrap as program

        _lazy_imports["program"] = program
        return program
    elif name == "MotionSettings":
        from nova.types import MotionSettings

        _lazy_imports["MotionSettings"] = MotionSettings
        return MotionSettings

    raise AttributeError(f"module '{__name__}' has no attribute '{name}'")


__version__ = version

__all__ = [
    "Nova",
    "Cell",
    "MotionGroup",
    "combine_trajectories",
    "Controller",
    "speed_up_movement_controller",
    "api",
    "types",
    "actions",
    "MotionSettings",
    "logger",
    "program",
    "__version__",
]
