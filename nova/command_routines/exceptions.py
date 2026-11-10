from nova import api


class CommandRoutineError(Exception):
    """Base class for all nova.command_routines errors."""


class UnsupportedRoutineCommand(CommandRoutineError):
    """The routine uses a feature NOVA cannot execute yet; the message names the command."""


def error_from_api_exception(exc: api.ApiException) -> CommandRoutineError:
    """Map a raw API-client exception onto the stable `nova.command_routines` exception type.

    NOVA's message already names the command, its index and the reason, so it is reused as-is.
    A 501 means NOVA cannot execute a feature of the routine yet.
    """
    failure = (api.models.Error, api.models.CommandRoutineFailure)
    message = exc.data.message if isinstance(exc.data, failure) else str(exc)
    error_cls = UnsupportedRoutineCommand if exc.status == 501 else CommandRoutineError
    return error_cls(message)
