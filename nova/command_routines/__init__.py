"""Assemble NOVA CommandRoutine objects declaratively, and plan and execute them on NOVA.

Build a routine by passing an ordered list of commands to :func:`command_routine`.
The command, pose, path and trigger factories in :mod:`nova.command_routines.commands`
construct the individual pieces; :func:`with_settings` shares motion settings across a
group of motions. :func:`plan` and :func:`execute` send a resolved routine to NOVA.
"""

from nova.command_routines.command_routines import execute, plan
from nova.command_routines.commands import (
    at_distance,
    at_path_fraction,
    at_time,
    generated_motion,
    io_value,
    joint_position,
    local_pose,
    marker,
    motion,
    move_cartesian_ptp,
    move_circular,
    move_joint_ptp,
    move_linear,
    path_cartesian_ptp,
    path_circle,
    path_joint_ptp,
    path_line,
    pause_on_io,
    set_io,
    wait_for_io,
    wait_for_time,
)
from nova.command_routines.exceptions import (
    CommandRoutineError,
    RoutineRunFailed,
    UnsupportedRoutineCommand,
)
from nova.command_routines.routine import command_routine, with_settings

__all__ = [
    "CommandRoutineError",
    "RoutineRunFailed",
    "UnsupportedRoutineCommand",
    "command_routine",
    "with_settings",
    "plan",
    "execute",
    "motion",
    "generated_motion",
    "move_linear",
    "move_cartesian_ptp",
    "move_joint_ptp",
    "move_circular",
    "set_io",
    "wait_for_time",
    "wait_for_io",
    "pause_on_io",
    "marker",
    "io_value",
    "local_pose",
    "joint_position",
    "path_line",
    "path_cartesian_ptp",
    "path_joint_ptp",
    "path_circle",
    "at_path_fraction",
    "at_distance",
    "at_time",
]
