from nova import api
from nova.types import Pose


class ControllerNotFound(Exception):
    def __init__(self, controller: str):
        super().__init__(f"Controller {controller} not found.")


class PlanTrajectoryFailed(Exception):
    def __init__(
        self,
        error: api.models.PlanTrajectoryFailedResponse | api.models.PlanCollisionFreeFailedResponse,
        motion_group_id: str,
    ):
        """
        Create a PlanTrajectoryFailed exception.

        Args:
            error:           The failure response.
            motion_group_id: The ID of the motion group that caused the exception, e.g. `0@controller`
        """
        self._error = error
        self._motion_group_id = motion_group_id
        super().__init__(
            f"Plan trajectory on {motion_group_id} failed: {error.model_dump_json(indent=2)}"
        )

    def to_pretty_string(self) -> str:
        """Give a more lightweight representation of the error, omitting some gritty details."""
        return f"Plan trajectory on {self._motion_group_id} failed: {self._error.model_dump_json(indent=2, exclude={'joint_trajectory'})}"

    @property
    def error(
        self,
    ) -> api.models.PlanTrajectoryFailedResponse | api.models.PlanCollisionFreeFailedResponse:
        """Return the original PlanTrajectoryFailedResponse object."""
        return self._error


class InitMovementFailed(Exception):
    def __init__(self, error: api.models.InitializeMovementResponse):
        self._error = error
        super().__init__(f"Initial movement failed: {error.model_dump_json(indent=2)}")

    @property
    def error(self) -> api.models.InitializeMovementResponse:
        """Return the original InitializeMovementResponseInitResponse object."""
        return self._error


class ErrorDuringMovement(Exception):
    """Raised when an error occurs during movement execution."""

    def __init__(self, message: str):
        self._message = message
        super().__init__(f"Error during movement: {message}")

    @property
    def message(self) -> str:
        """Return the error message."""
        return self._message


class LoadPlanFailed(Exception):
    def __init__(self, error: api.models.AddTrajectoryError):
        self._error = error
        super().__init__(f"Load plan failed: {error.model_dump_json(indent=2)}")

    @property
    def error(self) -> api.models.AddTrajectoryError:
        """Return the original PlanSuccessfulResponse object."""
        return self._error


class InconsistentCollisionScenes(Exception):
    """Raised when actions have different collision scenes."""

    def __init__(self, message: str):
        self._message = message
        super().__init__(message)

    @property
    def message(self) -> str:
        """Return the error message."""
        return self._message


class ControllerCreationFailed(Exception):
    """Raised when controller creation fails during declarative setup."""

    def __init__(self, controller_name: str, error: str):
        self.controller_name = controller_name
        self.error = error
        super().__init__(f"Failed to create controller '{controller_name}': {error}")


class AsyncActionError(Exception):
    """Raised when an async action fails during trajectory execution.

    This exception captures detailed context about the action failure including
    where in the trajectory it occurred and whether it was blocking motion.

    Attributes:
        action_name: Name of the async action that failed.
        trigger_location: Path parameter where action was triggered.
        completion_location: Path parameter when error occurred (may differ from trigger).
        cause: The underlying exception that caused the failure.
        was_blocking: Whether the action was blocking robot motion.
    """

    def __init__(
        self,
        action_name: str,
        trigger_location: float,
        completion_location: float | None,
        cause: Exception,
        was_blocking: bool,
    ):
        self.action_name = action_name
        self.trigger_location = trigger_location
        self.completion_location = completion_location
        self.cause = cause
        self.was_blocking = was_blocking

        location_info = f"triggered at {trigger_location:.3f}"
        if completion_location is not None and completion_location != trigger_location:
            location_info += f", failed at {completion_location:.3f}"

        blocking_info = " (blocking)" if was_blocking else " (parallel)"

        super().__init__(
            f"Async action '{action_name}'{blocking_info} failed ({location_info}): {cause}"
        )

    def __cause__(self) -> Exception:
        """Return the underlying cause exception."""
        return self.cause


# extends ValueError for backwards compatibility, otherwise it could extend Exception directly
class NoInverseKinematicsSolutionFound(ValueError):
    """Raised when no inverse kinematics solution can be found for a target pose."""

    def __init__(self, pose: Pose):
        self._pose = pose
        super().__init__(f"No inverse kinematics solution found for target pose {pose}")

    @property
    def pose(self) -> Pose:
        """Return the target pose that could not be solved."""
        return self._pose
