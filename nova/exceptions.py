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


class UnexpectedTrajectoryState(ErrorDuringMovement):
    """The controller reported a trajectory state that contradicts what the SDK commanded.

    Raised by the trajectory cursor when a motion-group state frame cannot be reconciled
    with the execution it is tracking — e.g. the controller reports ``RUNNING`` while the
    SDK believes the trajectory is paused or finished and has not issued a start, or a
    one-shot ``execute()`` is paused by someone other than the SDK. The execution is torn
    down instead of being left in a state that can never complete.
    """

    def __init__(
        self,
        message: str,
        *,
        machine_state: str | None = None,
        frame: api.models.MotionGroupState | None = None,
    ):
        self.machine_state = machine_state
        self.frame = frame
        super().__init__(message)


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
