"""Invertible, element-wise unit operators for the observation and action path (contract M1).

Units and scale only, not normalization (docs/schema.md#units).
"""

from __future__ import annotations

from abc import ABC, abstractmethod
from enum import StrEnum
import math
from typing import TYPE_CHECKING, ClassVar

if TYPE_CHECKING:
    from collections.abc import Iterable, Sequence


class OpDirection(StrEnum):
    """How an operator behaves when the path is reversed."""

    BIJECTIVE = "bijective"
    """Exactly reversible: ``inverse(forward(v)) == v``. Safe on an action."""

    BIDIRECTIONAL = "bidirectional"
    """Not reversible, but meaningful in both directions (a clamp). Safe on an action."""

    FORWARD_ONLY = "forward_only"
    """Has no meaningful reverse. Rejected on a channel that is also an action."""


class ValueOp(ABC):
    """One element-wise conversion applied to observation and action values."""

    direction: ClassVar[OpDirection]

    @abstractmethod
    def forward(self, value: float) -> float:
        """Hardware value → policy value."""

    @abstractmethod
    def inverse(self, value: float) -> float:
        """Policy value → hardware value."""


class Rad2Deg(ValueOp):
    """Radians (NOVA) → degrees (the dataset). Exact in both directions."""

    direction: ClassVar[OpDirection] = OpDirection.BIJECTIVE

    def forward(self, value: float) -> float:  # ruff: ignore[no-self-use]
        return math.degrees(value)

    def inverse(self, value: float) -> float:  # ruff: ignore[no-self-use]
        return math.radians(value)

    def __repr__(self) -> str:
        return "Rad2Deg()"


class Scale(ValueOp):
    """Multiply by a constant. ``Scale(0.001)`` converts NOVA millimetres to metres."""

    direction: ClassVar[OpDirection] = OpDirection.BIJECTIVE

    def __init__(self, factor: float) -> None:
        if not math.isfinite(factor) or factor == 0:
            raise ValueError(f"Scale factor must be finite and non-zero, got {factor}")
        self.factor = factor

    def forward(self, value: float) -> float:
        return value * self.factor

    def inverse(self, value: float) -> float:
        return value / self.factor

    def __repr__(self) -> str:
        return f"Scale({self.factor})"


class Clamp(ValueOp):
    """Bound values to ``[low, high]``: the observation forward, the command in reverse.

    A software limit in the value path, not a safety system.
    """

    direction: ClassVar[OpDirection] = OpDirection.BIDIRECTIONAL

    def __init__(self, low: float, high: float) -> None:
        if not (math.isfinite(low) and math.isfinite(high)):
            raise ValueError(f"Clamp bounds must be finite, got [{low}, {high}]")
        if low > high:
            raise ValueError(f"Clamp low must not exceed high, got [{low}, {high}]")
        self.low = low
        self.high = high

    def forward(self, value: float) -> float:
        return min(max(value, self.low), self.high)

    def inverse(self, value: float) -> float:
        return self.forward(value)

    def __repr__(self) -> str:
        return f"Clamp({self.low}, {self.high})"


def apply_ops(values: Sequence[float], ops: Sequence[ValueOp]) -> list[float]:
    """Run every operator front-to-back over ``values``."""
    if not ops:
        return list(values)
    result = list(values)
    for op in ops:
        result = [op.forward(value) for value in result]
    return result


def apply_ops_inverse(values: Sequence[float], ops: Sequence[ValueOp]) -> list[float]:
    """Run every operator back-to-front, inverting each."""
    if not ops:
        return list(values)
    result = list(values)
    for op in reversed(ops):
        result = [op.inverse(value) for value in result]
    return result


def reject_forward_only(ops: Iterable[ValueOp], where: str) -> None:
    """Raise when a writable channel declares an operator with no reverse."""
    offenders = [repr(op) for op in ops if op.direction is OpDirection.FORWARD_ONLY]
    if offenders:
        msg = (
            f"{where} is writable, so its operators must be reversible; "
            f"{', '.join(offenders)} cannot be inverted. Set action=False, or drop them."
        )
        raise ValueError(msg)


__all__ = [
    "Clamp",
    "OpDirection",
    "Rad2Deg",
    "Scale",
    "ValueOp",
    "apply_ops",
    "apply_ops_inverse",
    "reject_forward_only",
]
