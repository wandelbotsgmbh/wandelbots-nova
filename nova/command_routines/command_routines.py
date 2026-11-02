from __future__ import annotations

import asyncio
import logging
from typing import TYPE_CHECKING

from nova import api
from nova.command_routines.exceptions import (
    CommandRoutineError,
    RoutineRunFailed,
    UnsupportedRoutineCommand,
)
from nova.config import CELL_NAME

if TYPE_CHECKING:
    from nova.core.nova import Nova


logger = logging.getLogger(__name__)

RUN_POLL_INTERVAL_SECS = 0.1
_FINAL_RUN_STATES = (
    api.models.CommandRoutineRunState.COMPLETED,
    api.models.CommandRoutineRunState.FAILED,
    api.models.CommandRoutineRunState.STOPPED,
)


def _command_routine_error(exc: api.ApiException) -> CommandRoutineError:
    """Map a raw API-client exception onto the stable `nova.command_routines` exception type.

    NOVA's message already names the command, its index and the reason, so it is reused as-is.
    A 501 means NOVA cannot execute a feature of the routine yet.
    """
    message = exc.data.message if isinstance(exc.data, api.models.Error) else str(exc)
    error_cls = UnsupportedRoutineCommand if exc.status == 501 else CommandRoutineError
    return error_cls(message)


async def plan(
    nova: Nova,
    command_routine: api.models.CommandRoutine,
    *,
    cell: str = CELL_NAME,
    controller: str | None = None,
) -> api.models.PlannedCommandRoutine:
    """Plan a resolved command routine on the NOVA instance, without moving the robot.

    NOVA plans the routine with its own context (motion group, motion group setup, TCP, start
    joint position, default motion settings). Use the returned trajectory to draw the path.

    Args:
        nova: A NOVA instance.
        command_routine: The routine to plan. Its poses must be resolved, see
            `nova.datasets.resolve_command_routine`.
        cell: The cell to plan in.
        controller: Plan for the motion group with the same index on this controller instead of
            the routine's own.

    Raises:
        UnsupportedRoutineCommand: The routine uses a feature NOVA cannot execute yet.
        CommandRoutineError: NOVA rejects the routine, e.g. an unresolved pose or an unreachable
            target; the error names the command.
    """
    try:
        return await nova.api.command_routine_execution_api.plan_command_routine(
            cell=cell, command_routine=command_routine, controller=controller
        )
    except api.ApiException as exc:
        raise _command_routine_error(exc) from exc


async def execute(
    nova: Nova,
    command_routine: api.models.CommandRoutine,
    *,
    cell: str = CELL_NAME,
    controller: str | None = None,
) -> api.models.CommandRoutineRun:
    """Run a resolved command routine on the NOVA instance and wait until it has completed.

    NOVA plans and runs every command of the routine with its own context: motions, set_io,
    wait_for_time, wait_for_io, pause_on_io and markers. The robot must be at the routine's start
    joint position. Cancelling the call stops the run.

    Args:
        nova: A NOVA instance.
        command_routine: The routine to run. Its poses must be resolved, see
            `nova.datasets.resolve_command_routine`.
        cell: The cell to run in.
        controller: Run on the motion group with the same index on this controller instead of the
            routine's own.

    Returns:
        The completed run.

    Raises:
        UnsupportedRoutineCommand: The routine uses a feature NOVA cannot execute yet.
        CommandRoutineError: NOVA rejects the start, e.g. an unresolved pose, an unreachable
            target, a busy motion group or a robot that is not at the start joint position.
        RoutineRunFailed: The run failed or was stopped from elsewhere.
    """
    routines = nova.api.command_routine_execution_api

    async def start() -> api.models.CommandRoutineRun:
        try:
            return await routines.start_command_routine_run(
                cell=cell, command_routine=command_routine, controller=controller
            )
        except api.ApiException as exc:
            raise _command_routine_error(exc) from exc

    # Shielded, so that a run starting while the call is cancelled is still stopped.
    starting = asyncio.ensure_future(start())
    try:
        run = await asyncio.shield(starting)
        while run.state not in _FINAL_RUN_STATES:
            await asyncio.sleep(RUN_POLL_INTERVAL_SECS)
            run = await routines.get_command_routine_run(cell=cell, run=run.run)
    except BaseException:
        # Cancelled, or the run can no longer be followed: never leave it running.
        run = await starting
        logger.info(f"Stopping routine run {run.run}")
        await routines.stop_command_routine_run(cell=cell, run=run.run)
        raise

    if run.state != api.models.CommandRoutineRunState.COMPLETED:
        raise RoutineRunFailed(run)
    return run
