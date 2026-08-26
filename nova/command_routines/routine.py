"""Assemble NOVA CommandRoutine objects declaratively.

``command_routine`` wraps the generated ``api.models.CommandRoutine`` constructor so a
routine reads as an ordered list of commands. ``with_settings`` stamps shared motion
settings onto a group of motion commands that do not define their own.
"""

from __future__ import annotations

from nova import api

_MOTION_COMMANDS = (api.models.ExplicitPathMotionCommand, api.models.GeneratedPathMotionCommand)


def command_routine(
    routine_id: str,
    *,
    commands: list[api.models.Command],
    motion_group_setup: api.models.MotionGroupSetup | None = None,
    tcp: str | None = None,
    start_joint_position: list[float] | None = None,
    default_motion_settings: api.models.MotionSettings | None = None,
    name: str | None = None,
    description: str | None = None,
    dataset: str | None = None,
    motion_group: api.models.MotionGroupReference | str | None = None,
    metadata: dict[str, str] | None = None,
) -> api.models.CommandRoutine:
    """Build a ``CommandRoutine`` from an ordered list of commands.

    Args:
        routine_id: The routine id (must start with a letter and contain only
            letters, digits and hyphens).
        commands: The ordered commands. Must contain at least one command.
        motion_group_setup: The baseline motion group setup, or ``None`` for an
            I/O-only routine.
        tcp: The routine-level tool-center-point identifier.
        start_joint_position: The starting joints a motion routine requires.
        default_motion_settings: The routine-level baseline motion settings.
        name: A human-readable routine name.
        description: A routine description.
        dataset: The owning dataset id.
        motion_group: The executing motion group, given as a reference or an id.
        metadata: String-valued annotations preserved on round-trip.

    Examples:
    >>> from nova.command_routines import set_io
    >>> routine = command_routine(
    ...     "pick-and-place",
    ...     commands=[set_io("digital_out[0]", True)],
    ... )
    >>> routine.command_routine, len(routine.commands)
    ('pick-and-place', 1)
    """
    return api.models.CommandRoutine(
        command_routine=routine_id,
        dataset=dataset,
        name=name,
        description=description,
        motion_group=(
            api.models.MotionGroupReference(id=motion_group)
            if isinstance(motion_group, str)
            else motion_group
        ),
        motion_group_setup=motion_group_setup,
        tcp=tcp,
        start_joint_position=start_joint_position,
        default_motion_settings=default_motion_settings,
        commands=commands,
        metadata=metadata,
    )


def with_settings(
    settings: api.models.MotionSettings, commands: list[api.models.Command]
) -> list[api.models.Command]:
    """Apply ``settings`` to motion commands in ``commands`` that lack their own.

    Returns a new list; motion commands that already define ``motion_settings`` and
    non-motion commands are passed through unchanged.

    Examples:
    >>> from nova.command_routines import move_linear
    >>> commands_with_settings = with_settings(
    ...     api.models.MotionSettings(),
    ...     [move_linear("a")],
    ... )
    >>> commands_with_settings[0].motion_settings is not None
    True
    """
    result: list[api.models.Command] = []
    for command in commands:
        if isinstance(command, _MOTION_COMMANDS) and command.motion_settings is None:
            result.append(command.model_copy(update={"motion_settings": settings}))
        else:
            result.append(command)
    return result
