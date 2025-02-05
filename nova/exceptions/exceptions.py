class MotionGroupConnectionException(Exception):
    def __init__(self, message):
        self.message = message
        super().__init__(self.message)


class PlanningFailedException(Exception):
    def __init__(self, message):
        self.message = message
        super().__init__(self.message)


class PlanningPartialSuccessWarning(Exception):
    def __init__(self, message, motion):
        self.motion = motion
        self.message = message
        super().__init__(self.message)


class MotionExecutionError(Exception):
    def __init__(self, message):
        self.message = message
        super().__init__(self.message)


class MotionExecutionTimedOutError(Exception):
    def __init__(self, message):
        self.message = message
        super().__init__(self.message)


class MotionExecutionInterruptedError(Exception):
    def __init__(self, message):
        self.message = message
        super().__init__(self.message)
