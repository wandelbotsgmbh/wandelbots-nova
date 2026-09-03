from __future__ import annotations

import contextvars
from typing import TYPE_CHECKING, Any

if TYPE_CHECKING:
    from nova import Nova
    from nova.datasets import Dataset

current_program_context_var: contextvars.ContextVar["ProgramContext | None"] = (
    contextvars.ContextVar("current_program_context_var", default=None)
)


class ProgramContext:
    """Context passed into every program execution."""

    def __init__(self, nova: Nova, program_id: str | None = None, dataset: Dataset | None = None):
        self._nova = nova
        self._program_id = program_id
        self._cell = nova.cell()
        self._dataset = dataset

    @property
    def nova(self) -> Nova:
        """Returns the Nova instance for the program."""
        return self._nova

    @property
    def cell(self):
        """Returns the default cell for the program, if available."""
        return self._cell

    @property
    def program_id(self) -> str | None:
        """Returns the program ID for the program."""
        return self._program_id

    @property
    def dataset(self) -> Dataset | None:
        """Returns the dataset requested for the program.

        This is resolved once, before the program body starts running, so
        accessing this property does not require awaiting anything.
        """
        return self._dataset

    def cycle(self, extra: dict[str, Any] | None = None):
        """Create a Cycle with program pre-populated in the extra data."""
        from nova.events import Cycle

        if self._cell is None:
            raise AttributeError(
                "ProgramContext.cell is not available; the provided Nova instance does not expose cell()."
            )

        merged_extra = {"program": self.program_id} if self.program_id else {}
        if extra:
            merged_extra.update(extra)
        return Cycle(cell=self.cell, extra=merged_extra)
