import json

import wandelbots_api_client as wb


class ControllerNotFoundException(Exception):
    def __init__(self, controller: str):
        super().__init__(f"Controller {controller} not found.")


class PlanTrajectoryFailed(Exception):
    def __init__(self, error: wb.models.PlanTrajectoryFailedResponse):
        self._error = error
        super().__init__(f"Plan trajectory failed: {json.dumps(error.to_dict(), indent=2)}")

    @property
    def error(self) -> wb.models.PlanTrajectoryFailedResponse:
        """Return the original PlanTrajectoryFailedResponse object."""
        return self._error


class InitMovementFailed(Exception):
    def __init__(self, error: wb.models.InitializeMovementResponseInitResponse):
        self._error = error
        super().__init__(f"Initial movement failed: {json.dumps(error.to_dict(), indent=2)}")

    @property
    def error(self) -> wb.models.InitializeMovementResponseInitResponse:
        """Return the original InitializeMovementResponseInitResponse object."""
        return self._error


class LoadPlanFailed(Exception):
    def __init__(self, error: wb.models.PlanSuccessfulResponse):
        self._error = error
        super().__init__(f"Load plan failed: {json.dumps(error.to_dict(), indent=2)}")

    @property
    def error(self) -> wb.models.PlanSuccessfulResponse:
        """Return the original PlanSuccessfulResponse object."""
        return self._error
