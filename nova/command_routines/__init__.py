"""Assemble NOVA CommandRoutine objects declaratively.

Build a routine by passing an ordered list of commands to :func:`command_routine`.
The command, pose, path and trigger factories in :mod:`nova.command_routines.commands`
construct the individual pieces; :func:`with_settings` shares motion settings across a
group of motions.

A built routine can be planned and executed directly: pass it wherever ``MotionGroup.plan``,
``execute`` or ``plan_and_execute`` expect actions. Under the hood it is converted to the
SDK's ``Action`` list via :func:`actions_from_command_routine`; see that function's docstring
for which parts of a routine are supported today. A routine loaded from a dataset export
(``api.models.GetDatasetResponse``) typically targets poses by id (``LocalPoseReference``);
:func:`resolve_dataset_poses` resolves the dataset's poses (and their coordinate-system
chains) to world-frame ``Pose`` objects, which can be passed as
``actions_from_command_routine``'s ``pose_resolver``, or as ``plan``/``execute``'s.
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
from nova.command_routines.dataset import get_command_routine, resolve_dataset_poses
from nova.command_routines.planning import (
    UnsupportedCommandRoutineFeature,
    actions_from_command_routine,
)
from nova.command_routines.routine import command_routine, with_settings

__all__ = [
    "command_routine",
    "with_settings",
    "actions_from_command_routine",
    "resolve_dataset_poses",
    "get_command_routine",
    "UnsupportedCommandRoutineFeature",
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
