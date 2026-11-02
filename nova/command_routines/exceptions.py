from nova import api


class CommandRoutineError(Exception):
    """Base class for all nova.command_routines errors."""


class UnsupportedRoutineCommand(CommandRoutineError):
    """The routine uses a feature NOVA cannot execute yet; the message names the command."""


class RoutineRunFailed(CommandRoutineError):
    """A routine run ended without completing: it failed or was stopped from elsewhere."""

    def __init__(self, run: api.models.CommandRoutineRun):
        self.run = run
        reason = f": {run.error}" if run.error else ""
        super().__init__(f"Routine run {run.run} ended {run.state.value}{reason}")
