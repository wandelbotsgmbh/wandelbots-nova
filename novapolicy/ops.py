"""Invertible value operators for the observation and action path.

NOVA speaks radians and millimetres. A dataset does not necessarily: a policy
trained on degrees normalizes cleanly against its own statistics and then moves
the arm to the wrong place, because those statistics are in the *recording*
robot's units. Declaring the conversion makes it explicit and, crucially,
reversible — the same declaration that scales an observation on the way in
un-scales the action on the way out.

Operators are element-wise by construction, so a declaration covering several
motion groups applies identically whether the values arrive concatenated (an
observation) or split per group (an action chunk).

This is *not* normalization. LeRobot's policy server applies the checkpoint's
own normalization and unnormalization; duplicating it here would corrupt the
values twice over. What is unguarded, and what these operators are for, is units
and scale.
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
    """Not reversible, but meaningful applied in both directions — a clamp
    shapes the observation on the way in and limits the command on the way out.
    Safe on an action."""

    FORWARD_ONLY = "forward_only"
    """Has no meaningful reverse. Rejected on a channel that is also an action,
    because the executor would have nothing to send back to the robot."""


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
    """Bound values to ``[low, high]`` in both directions.

    Applied forward it bounds what the policy observes; applied in reverse it
    bounds the command before it reaches the robot, which is the safety-relevant
    direction. Clamping is monotone, so its position in an operator list is
    preserved by the reversal: ``[Rad2Deg(), Clamp(-90, 90)]`` clamps in degrees
    on the way in *and* on the way out.

    This is a software limit in the executor's own value path, not a safety
    system. Joint limits belong on the robot controller.
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
    """Run every operator back-to-front, inverting each.

    The reversal matters: ``[Scale(0.001), Clamp(-1, 1)]`` clamps in metres, so
    the inverse must clamp in metres *before* scaling back to millimetres.
    """
    if not ops:
        return list(values)
    result = list(values)
    for op in reversed(ops):
        result = [op.inverse(value) for value in result]
    return result


def reject_forward_only(ops: Iterable[ValueOp], where: str) -> None:
    """Raise when a writable channel declares an operator with no reverse.

    Direction is a property of the operator class, so this is the whole
    load-time check — a library-owned ``Rad2Deg`` or ``Scale`` is invertible by
    construction, and ``Scale`` validates its factor in ``__init__``.
    """
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
