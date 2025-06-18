"""Excpetions raised in execution and signals to alter control flow"""

from dataclasses import dataclass


@dataclass(frozen=True)
class TextPosition:
    """A position in a wandelscript code"""

    line: int
    column: int


@dataclass(frozen=True)
class TextRange:
    """A region in a wandelscript code"""

    start: TextPosition
    end: TextPosition


class Signal(Exception):
    """Used to handle the control flow in the interpreter"""


class BreakSignal(Signal):
    """Do not continue the current loop"""


class ReturnSignal(Signal):
    """Signal to return from a function"""

    def __init__(self, value):
        super().__init__()
        self.value = value


class TerminationSignal(Signal):
    """This exception is used to indicate that the program needs is terminating"""


@dataclass
class ProgramError(Exception):
    """Generic error when checking, parsing, executing, or debugging the programs"""

    location: TextRange | None

    def dict(self):
        result = {"text": self.message()}
        if self.location:
            result["line"] = self.location.start.line
            result["column"] = self.location.start.column
        return result

    def message(self) -> str:
        return "Unexpected error"

    def __post_init__(self):
        if isinstance(self.location, TextRange):
            super().__init__(
                f"At line {self.location.start.line} column {self.location.start.column}: {self.message()}"
            )
        elif isinstance(self.location, TextPosition):
            super().__init__(
                f"At line {self.location.line} column {self.location.column}: {self.message()}"
            )
        else:
            super().__init__(self.message())


class ProgramRuntimeError(ProgramError):
    """Any runtime constraint is not fulfilled"""

    def message(self):
        return "Runtime error"


class ConfigurationError(ProgramRuntimeError):
    """Robot cell is not sufficiently configured

    E.g., robot pose not initialized or hardware not available
    """


@dataclass
class GenericRuntimeError(ProgramError):
    """Raised when an error is detected during wandelscript execution that
    cannot be described by a different exception."""

    text: str

    def message(self):
        return self.text


@dataclass
class NameError_(ProgramRuntimeError):
    """Any runtime constraint is not fulfilled"""

    name: str

    def message(self):
        return f"Variable or function not defined: {self.name}"


@dataclass
class UserError(ProgramRuntimeError):
    """Any runtime constraint is not fulfilled"""

    value: str

    def message(self):
        return f"User defined error: '{self.value}'"


@dataclass
class MotionError(ProgramRuntimeError):
    """Any runtime constraint is not fulfilled"""

    value: str

    def message(self):
        return self.value


@dataclass
class NotPlannableError(ProgramRuntimeError):
    """Any runtime constraint is not fulfilled"""

    value: str

    def message(self):
        return self.value


@dataclass
class ProgramSyntaxError(ProgramError):
    """A Wandelscript syntax error"""

    text: str | None = None

    def message(self):
        return self.text if self.text else "Unknown syntax error"


@dataclass
class NestedSyncError(ProgramSyntaxError):
    """A Wandelscript syntax error"""

    def message(self):
        return "Explicit and implicit sync within the robot context is not supported yet"


@dataclass
class WrongRobotError(GenericRuntimeError):
    """A Wandelscript syntax error"""
