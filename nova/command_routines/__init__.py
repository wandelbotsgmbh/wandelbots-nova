"""Assemble NOVA CommandRoutine objects declaratively.

Build a routine by passing an ordered list of commands to :func:`command_routine`.
The command, pose, path and trigger factories in :mod:`nova.command_routines.commands`
construct the individual pieces; :func:`with_settings` shares motion settings across a
group of motions.
"""

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
    path_cartesian_ptp,
    path_circle,
    path_joint_ptp,
    path_line,
    pause_on_io,
    set_io,
    wait_for_io,
    wait_for_time,
)
from nova.command_routines.routine import command_routine, with_settings

__all__ = [
    "command_routine",
    "with_settings",
    "motion",
    "generated_motion",
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
