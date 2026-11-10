"""Assemble NOVA CommandRoutine objects declaratively.

``command_routine`` builds a ``CommandRoutine`` (the generated ``api.models.CommandRoutine``
with resolve, plan, execute and stop) so a
routine reads as an ordered list of commands. ``with_settings`` stamps shared motion
settings onto a group of motion commands that do not define their own.
"""

from __future__ import annotations

from typing import TYPE_CHECKING

from nova import api
from nova.command_routines.exceptions import error_from_api_exception
from nova.config import CELL_NAME

if TYPE_CHECKING:
    from nova.cell.motion_group import MotionGroup
    from nova.core.nova import Nova

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
) -> CommandRoutine:
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
    return CommandRoutine(
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


class CommandRoutine(api.models.CommandRoutine):
    """A CommandRoutine that NOVA plans and runs on a motion group.

    Extends the generated ``api.models.CommandRoutine`` (all its fields are available) with
    resolve, plan, execute and stop. NOVA plans the whole routine with its own context (TCP, start joint
    position, motion group setup, default motion settings) and runs that plan; the routine is sent as it
    is, without being converted into an action list. Poses must be inline poses in the ``world`` frame, resolve a
    routine stored in a Dataset first with :meth:`resolve`.

    Examples:
    >>> from nova.command_routines import set_io
    >>> routine = command_routine("io", commands=[set_io("OUT#1", True)])
    >>> routine.command_routine
    'io'
    """

    @classmethod
    async def resolve(
        cls,
        nova: Nova,
        dataset: api.models.DatasetId,
        command_routine: str,
        *,
        cell: str = CELL_NAME,
        revision: int | None = None,
    ) -> CommandRoutine:
        """Fetch a command routine of a dataset stored on the NOVA instance, with its poses resolved.

        Every local pose reference and every pose in a dataset frame becomes an inline pose in the
        `world` frame, so the routine runs without the dataset.

        Args:
            nova: A NOVA instance.
            dataset: Identifier of the dataset that holds the routine.
            command_routine: Identifier of the routine within the dataset.
            cell: The cell the dataset belongs to.
            revision: Revision to use. When omitted, the latest revision is used.

        Raises:
            DatasetNotFoundError: The dataset, its revision or the routine does not exist.
            DatasetError: A pose of the routine cannot be resolved; the error names the pose.
        """
        from nova.datasets.datasets import _dataset_error

        try:
            resolved = await nova.api.datasets_api.resolve_dataset_command_routine(
                cell=cell, dataset=str(dataset), command_routine=command_routine, revision=revision
            )
        except api.ApiException as exc:
            raise _dataset_error(exc) from exc
        return cls.model_validate(resolved.model_dump())

    def _motion_group_override(
        self, motion_group: MotionGroup
    ) -> api.models.MotionGroupReference | None:
        """The routine runs on ``motion_group`` whatever motion group it names; none if the same."""
        if self.motion_group is not None and self.motion_group.id == motion_group.id:
            return None
        return api.models.MotionGroupReference(id=motion_group.id)

    async def plan(self, motion_group: MotionGroup) -> api.models.PlanCommandRoutineResponse:
        """Let NOVA plan the routine for ``motion_group`` without moving the robot.

        NOVA plans it with the routine's own context; a routine that names another motion group is
        planned with the setup NOVA describes for ``motion_group``. The plan holds the trajectory
        (``joint_trajectory``, absent for an I/O-only routine) and where each command happens on it.
        What is drawn is what runs: pass the plan to :meth:`execute` and the run follows it without
        planning again.

        Raises:
            CommandRoutineError: NOVA rejects the routine; the error names the command.
            UnsupportedRoutineCommand: The routine uses a feature NOVA does not implement yet.
        """
        try:
            return await motion_group.api_client.command_routines_api.plan_command_routine(
                cell=motion_group.cell,
                plan_command_routine_request=api.models.PlanCommandRoutineRequest(
                    command_routine=self,
                    controller=motion_group.controller_id,
                    motion_group_override=self._motion_group_override(motion_group),
                ),
            )
        except api.ApiException as exc:
            raise error_from_api_exception(exc) from exc

    async def execute(
        self, motion_group: MotionGroup, plan: api.models.PlanCommandRoutineResponse | None = None
    ) -> api.models.CommandRoutineRun:
        """Start the routine on ``motion_group``; NOVA runs the plan and never plans again.

        Returns the run once NOVA has accepted it, not when the robot is done. :meth:`stop` stops it.

        Args:
            motion_group: The motion group to run on. It must be at the start of the plan's trajectory.
            plan: The plan to run, as returned by :meth:`plan`. When omitted, the routine is planned
                for ``motion_group`` first.

        Raises:
            CommandRoutineError: NOVA rejects the plan; the error names the command.
            UnsupportedRoutineCommand: The routine uses a feature NOVA does not implement yet.
        """
        if plan is None:
            plan = await self.plan(motion_group)
        placed = plan.model_copy(
            update={
                "controller": motion_group.controller_id,
                "motion_group": api.models.MotionGroupReference(id=motion_group.id),
            }
        )
        try:
            return await motion_group.api_client.command_routines_api.start_command_routine_run(
                cell=motion_group.cell,
                start_command_routine_run_request=api.models.StartCommandRoutineRunRequest.model_validate(
                    placed.model_dump()
                ),
            )
        except api.ApiException as exc:
            raise error_from_api_exception(exc) from exc

    @staticmethod
    async def stop(
        motion_group: MotionGroup, run: api.models.CommandRoutineRun | str
    ) -> api.models.CommandRoutineRun:
        """Stop a run: the robot stops and no further outputs of the run are written.

        Stopping a run that already ended succeeds and leaves its state unchanged.

        Args:
            motion_group: A motion group of the cell the run belongs to.
            run: The run, or its id, as returned by :meth:`execute`.
        """
        run_id = run if isinstance(run, str) else run.run
        try:
            return await motion_group.api_client.command_routines_api.stop_command_routine_run(
                cell=motion_group.cell, run=run_id
            )
        except api.ApiException as exc:
            raise error_from_api_exception(exc) from exc
