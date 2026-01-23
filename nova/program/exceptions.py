from dataclasses import dataclass
from typing import Any


@dataclass(frozen=True)
class TextPosition:
    """A position in a program"""

    line: int
    column: int


@dataclass(frozen=True)
class TextRange:
    """A region in a program"""

    start: TextPosition
    end: TextPosition


@dataclass
class ProgramError(Exception):
    """Generic error when checking, parsing, executing, or debugging the programs"""

    location: TextRange | None

    def dict(self) -> dict[str, Any]:
        result: dict[str, Any] = {"text": self.message()}
        if self.location:
            result["line"] = self.location.start.line
            result["column"] = self.location.start.column
        return result

    def message(self) -> str:
        return "Unexpected error"

    def __post_init__(self) -> None:
        if isinstance(self.location, TextRange):
            super().__init__(
                f"At line {self.location.start.line} column {self.location.start.column}: {self.message()}"
            )
        else:
            super().__init__(self.message())


class ProgramRuntimeError(ProgramError):
    """Any runtime constraint is not fulfilled"""

    def message(self) -> str:
        return "Runtime error"


@dataclass
class NotPlannableError(ProgramRuntimeError):
    """Any runtime constraint is not fulfilled"""

    value: str

    def message(self) -> str:
        return self.value
