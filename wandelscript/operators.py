"""Mapping operators from Wandelscript to Python"""

import operator
from enum import Enum
from typing import Any, TypeVar

from nova.types import Pose, Vector3d

T = TypeVar("T")


class Operator(Enum):
    """A baseclass for operators

    The child classes must define the enum cases like
        add: "+"
    where add is a valid member in the operator module and "+" is the string used in the xtext
    """

    def __str__(self):
        return str(self.value)


class UnaryOperator(Operator):
    def __call__(self, a: T) -> Any:
        op = getattr(operator, self.name)
        return op(a)


class BinaryOperator(Operator):
    def __call__(self, a: Any, b: Any) -> Any:
        op = getattr(operator, self.name)
        return op(a, b)


class LogicalOperator(BinaryOperator):
    """Logical : and "or" operators

    Example:
    >>> op = LogicalOperator("and")
    >>> op(True, False)
    False
    >>> str(op)
    'and'
    """

    and_ = "and"
    or_ = "or"

    def __call__(self, a: Any, b: Any) -> Any:
        """Oh my gawd

        This fudging system calls the operators twice. Once on the metamodel
        objects, returning Expressions and all, and once on the primitive types
        like ints and stuff. Since Python relies on custom `__bool__()`
        implementations for custom `and` and `or` behavior, and `__bool__()`
        *must* return `bool`, otherwise the runtime raises a `TypeError` - sic!
        - conditionally call custom functions when dealing with object that have
        those, i.e. `metamodel` objects, otherwise rely on `and` and `or` from
        the primitive types. Geez.

        Args:
            a: the left operand
            b: the right operand

        Returns:
            the logical combination of both operands
        """

        if self.value == "and":
            if hasattr(a, "_custom_and"):
                return a._custom_and(b)
            return a and b
        else:  # or
            if hasattr(a, "_custom_or"):
                return a._custom_or(b)
            return a or b


class AdditionOperator(BinaryOperator):
    """Addition and substraction operator

    Example:
    >>> op = AdditionOperator("+")
    >>> op(4, 5)
    9
    >>> str(op)
    '+'
    """

    add = "+"
    sub = "-"


def _to_pose_operand(value: Any) -> Any:
    """Coerce a geometric value operand of the `::` operator into a Pose.

    `Pose.__matmul__` only accepts a `Pose`, but Wandelscript's frame algebra also
    transforms plain positions/poses expressed as a `Vector3d` or a 3-/6-element numeric
    `tuple`/`list` (e.g. a `[...]` array literal, or a builtin returning a raw tuple).
    Those are converted to a `Pose` here so they can be chained with `::`.

    Everything else (`Pose`, `Frame`, `Closure`, ...) is returned unchanged so their own
    `@`/`__rmatmul__` handling still applies.
    """
    if isinstance(value, Vector3d):
        return Pose(value.to_tuple())
    if (
        isinstance(value, (tuple, list))
        and len(value) in (3, 6)
        and all(
            isinstance(component, (int, float)) and not isinstance(component, bool)
            for component in value
        )
    ):
        return Pose(tuple(value))
    return value


class MultiplicationOperator(BinaryOperator):
    """Multiplication, division and transformation chaining operator

    Example:
    >>> op = MultiplicationOperator("*")
    >>> op(4, 5)
    20
    >>> str(op)
    '*'
    """

    mul = "*"
    truediv = "/"
    matmul = "::"

    def __call__(self, a: Any, b: Any) -> Any:
        if self is MultiplicationOperator.matmul:
            a = _to_pose_operand(a)
            b = _to_pose_operand(b)
        return super().__call__(a, b)


class ComparisonOperator(BinaryOperator):
    """Operator to compare"""

    lt = "<"
    gt = ">"
    eq = "=="
    le = "<="
    ge = ">="
    ne = "!="


class Sign(UnaryOperator):
    pos = "+"
    neg = "-"


def invert(a: T) -> T:
    if isinstance(a, bool):
        return not a  # type: ignore
    return ~a  # type: ignore


class Inverse(UnaryOperator):
    inv = "~"

    def __call__(self, a: T) -> T:
        return invert(a)


class Not(UnaryOperator):
    not_ = "not"

    def __call__(self, a: T) -> Any:
        """Similar with the `LogicalOperator`s `and` and `or`, we have the
        function cater to both our `metamodel` types and to primitive types.
        Thus conditional call to `__custom__not__`. See the implementation for
        `LogicalOperator` above.

        Args:
            a: the operand

        Returns:
            the result of `not a`
        """
        if hasattr(a, "_custom_not"):
            return a._custom_not()
        return not a
