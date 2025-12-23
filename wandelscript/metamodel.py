from __future__ import annotations

import asyncio
import inspect
from abc import ABC, abstractmethod
from collections.abc import Callable
from dataclasses import dataclass
from functools import cache, reduce
from itertools import chain
from pathlib import Path as FilePath
from typing import Any, ClassVar, Generic, Literal, TypeVar

import anyio

import wandelscript.datatypes as t
import wandelscript.exception
from nova.actions.io import CallAction, ReadAction, ReadJointsAction, ReadPoseAction, WriteAction
from nova.cell.robot_cell import (
    AbstractRobot,
    AsyncCallableDevice,
    InputDevice,
    OutputDevice,
    RobotCell,
    RobotMotionError,
)
from nova.cell.simulation import SimulatedRobotCell, UnknownPose
from nova.types import MotionSettings, Pose, Vector3d
from wandelscript.exception import GenericRuntimeError, TextRange
from wandelscript.operators import (
    AdditionOperator,
    ComparisonOperator,
    Inverse,
    LogicalOperator,
    MultiplicationOperator,
    Not,
    Sign,
)
from wandelscript.runtime import ExecutionContext, Store
from wandelscript.utils.pose import pose_to_versor, versor_to_pose

ElementType = TypeVar("ElementType", bound=t.ElementType)


def add_orientation(strategy: str, position: Vector3d, previous_pose: Pose) -> Pose:
    """Augments a position with an orientation based on a strategy

    Might depend on futures also in last keyframes, ... and a strategy (like orientation relative to path)

    Args:
        position: the position which shall be augmented with an orientation
        strategy: the strategy how to derive the orientation
        previous_pose: the previous pose which is used by some strategies

    Returns:
        The path with orientation

    Raises:
        ValueError: for unknown strategies
    """

    if strategy == "last":
        # previous_orientation = QuaternionTensor.from_rotation_vector(previous_pose.orientation)
        orientation = previous_pose.orientation
    else:
        raise ValueError(f"Unexpected strategy {strategy}")
    return Pose(position=position, orientation=orientation)


class Factory(ABC):
    name: str

    def __init_subclass__(cls, func_name: str | None):
        super().__init_subclass__()
        if func_name is not None:
            cls.name = func_name
            # Note _cached_subclass is an attribute of cls not Factory, so different child classes have their own
            # cache
            cls._cached_subclasses[func_name] = cls  # type: ignore
        else:
            cls._cached_subclasses = {}  # type: ignore

    @classmethod
    def from_name(cls, name: str):
        return cls._cached_subclasses.get(name)  # type: ignore

    @classmethod
    def register_func(cls, context: ExecutionContext, name: str, func: Callable):
        context.store[name] = func


class Rule(ABC):
    """Abstract root class of wandelscript. Every statement is a rule"""

    _location = None

    async def __call__(self, context: ExecutionContext, **kwargs) -> Any:
        context.location_in_code = self.location
        interceptor_chain = reduce(
            lambda inner, outer: outer(inner, context),
            context.interceptors,
            self.call(context, **kwargs),
        )
        return await asyncio.shield(interceptor_chain)

    @property
    def location(self) -> TextRange:
        return self._location  # type: ignore

    def set_location(self, value: TextRange):
        """This allows writing of location (even in frozen dataclasses)

        Args:
            value: value to set
        """
        # TODO: provide location already during init instead of modifying it
        object.__setattr__(self, "_location", value)

    @abstractmethod
    async def call(self, context: ExecutionContext, **kwargs) -> Any:
        # return await self.call(store)
        ...


class Suite(Rule, ABC):
    """A suite is either a single statement or groups multiple statements"""


class Statement(Suite, ABC):
    """Base class for all actions in Wandelscript"""


@dataclass(frozen=True)
class Block(Suite):
    """A group of statements"""

    statements: tuple[Statement, ...]

    async def call(self, context: ExecutionContext, **kwargs):
        for statement in self.statements:
            await statement(context)


@dataclass(frozen=True)
class SwitchInterrupt(Statement):
    """Either activates or deactivated an interrupt"""

    action: str
    interrupt: str

    async def call(self, context: ExecutionContext, **kwargs):
        interrupt_instance = context.store[self.interrupt]
        if self.action == "activate":
            context.action_queue._on_motion_callbacks[self.interrupt] = interrupt_instance[2]  # pylint: disable=protected-access
        elif self.action == "deactivate":
            del context.action_queue._on_motion_callbacks[self.interrupt]  # pylint: disable=protected-access
        else:
            raise TypeError


@dataclass(frozen=True)
class Modifier(Rule):
    r"""Modify something and returns a function to undo this modification"""

    modifiers: tuple[FunctionCall, ...]

    async def call(self, context: ExecutionContext, **kwargs):
        exit_funcs = tuple(  # pylint: disable=consider-using-generator
            [await modifier(context) for modifier in self.modifiers]
        )

        async def on_exit(context):
            for f in exit_funcs:
                await f(context)

        return on_exit


@dataclass(frozen=True)
class Context(Statement):
    r"""The context

    Example:
    >>> code = '''
    ... home = (0, 0, 0, 0, 0, 0)
    ... with blending(20):
    ...     a = __ms_position_zone_radius
    ...     move via p2p() to home
    ...     move via line() to (1, 1, 1)
    ... '''
    >>> store = _run_program_debug(code).store
    >>> store['a']
    20
    """

    modifier: Modifier
    body: Suite

    async def call(self, context: ExecutionContext, **kwargs):
        on_exit = await self.modifier(context)
        await self.body(context)
        await on_exit(context)


@dataclass(frozen=True)
class Range(Rule):
    """A sequence"""

    start: Expression[int]
    end: Expression[int]
    interval_type: Literal["<"] | None = None

    async def call(self, context: ExecutionContext, **kwargs):
        start = await self.start(context)
        end = await self.end(context)
        if not self.interval_type:
            end += 1
        return range(start, end)


@dataclass(frozen=True)
class ForLoop(Statement):
    r"""The for-loop

    Example:
    >>> code = '''
    ... a = 0
    ... b = 0
    ... c = 0
    ... for i in 3 .. 5:
    ...     a = a + i
    ... for i in 3 ..< 5:
    ...     b = b + i
    ... for i in 3 ..< 5:
    ...     c = c + i
    ...     if i == 3:
    ...         break
    ... '''
    >>> store = asyncio.run(run_program(code)).store
    >>> store['a']
    12
    >>> store['b']
    7
    >>> store['c']
    3
    """

    name: str
    range: Range
    body: Suite

    async def call(self, context: ExecutionContext, **kwargs):
        for i in await self.range(context):
            context.store[self.name] = i
            try:
                await self.body(context)
            except wandelscript.exception.BreakSignal:
                break


@dataclass(frozen=True)
class RepeatLoop(Statement):
    r"""The repeat loop (anonymous simple for loop)

    Example:
    >>> code = '''
    ... a = 0
    ... repeat 5:
    ...     a = a + 2
    ... '''
    >>> store = asyncio.run(run_program(code)).store
    >>> store['a']
    10
    """

    count: Expression[int]
    body: Suite

    async def call(self, context: ExecutionContext, **kwargs):
        count = await self.count(context)
        for _ in range(count):
            try:
                await self.body(context)
            except wandelscript.exception.BreakSignal:
                break


@dataclass(frozen=True)
class RobotContext(Statement):
    r"""The robot context provides a way to specify the robot for a block of code

    Each block starts with `do with <robot>:` where the <robot> is the name of the robot that is moved in the block.
    No other can be moved within the block. The `and do with <second_robot>:` starts the block for the next robot.
    Here the <robot> acts as a master for the second motion of the <second_robot>. More blocks can follow.
    At the end a sync is executed implicitly. Please note that is not possible to nest these blocks.
    Also the sync command should not be used in a block.

    Also note that the difference between

        do with robot:
           ...
        and do with second_robot:
           ...

    and:

        do with robot:
           ...
        do with second_robot:
           ...

    In the latter example the robots move after each other and in the first example in parallel.

    Example:
    >>> code = '''
    ... do with controller[0]:
    ...     move to (0, 0, 0, 0, 0, 0)
    ...     move to (0, 0, 7, 0, 0, 0)
    ... and do with controller[1]:
    ...     move to (0, 0, 0, 0, 0, 0)
    ...     move to (0, 0, 11, 0, 0, 0)
    ... a = read(controller[0], 'Flange')
    ... b = read(controller[1], 'Flange')
    ... '''
    >>> store = _run_program_debug(code).store
    >>> store['a']
    Pose(position=Vector3d(x=0.0, y=0.0, z=7.0), orientation=Vector3d(x=0.0, y=0.0, z=0.0))
    >>> store['b']
    Pose(position=Vector3d(x=0.0, y=0.0, z=11.0), orientation=Vector3d(x=0.0, y=0.0, z=0.0))
    """

    robots: list[Expression]
    bodies: list[Suite]

    async def call(self, context: ExecutionContext, **kwargs):
        for robot, body in zip(self.robots, self.bodies):
            robot = await robot(context)
            if not isinstance(robot, AbstractRobot):
                raise GenericRuntimeError(
                    self.location, text=f"The device must be a robot but is a: {type(robot)}"
                )
            with context.with_robot(robot.id):
                await body(context)
        await context.sync()


@dataclass(frozen=True)
class SyncContext(Statement):
    r"""In the sync context the program and motion pointer are synchronized

    TODO: forbid motion commands to be called in the sync block

    Examples:
    >>> code = '''
    ... do:
    ...     move via p2p() to (0, 0, 0, 0, 0, 0)
    ...     move via line() to (1, 2, 3, 0, 0, 0)
    ... sync:
    ...     a = read(controller[0], 'pose')
    ... '''
    >>> store = _run_program_debug(code).store
    >>> store['a']
    Pose(position=Vector3d(x=1.0, y=2.0, z=3.0), orientation=Vector3d(x=0.0, y=0.0, z=0.0))

    >>> code = '''
    ... sync
    ... '''
    >>> _ = _run_program_debug(code)
    """

    do_body: Suite | None
    sync_body: Suite | None
    exception_handler: Suite | None

    async def call(self, context: ExecutionContext, **kwargs):
        try:
            if self.do_body:
                await self.do_body(context)
            await context.sync()
        except (RobotMotionError, wandelscript.exception.UserError) as error:
            if self.exception_handler:
                await self.exception_handler(context)
            else:
                raise error
        else:
            if self.sync_body:
                await self.sync_body(context)


@dataclass(frozen=True)
class Switch(Statement):
    r"""The switch-case-default statement

    Example:
    >>> code = '''
    ... a = 10/2-5
    ... switch a:
    ... case 0+1: a = 2-1
    ... case 0*10: a = -1
    ... default: a= 2
    ... '''
    >>> store = _run_program_debug(code).store
    >>> store['a']
    -1
    """

    switch_expression: Expression[int]
    case_expressions: tuple[Expression[int], ...]
    case_bodies: tuple[Suite, ...]
    default_body: Suite

    async def call(self, context: ExecutionContext, **kwargs):
        for expression, body in zip(self.case_expressions, self.case_bodies):
            switch_expression = await self.switch_expression(context)
            if switch_expression == await expression(context):
                await body(context)
                break
        else:
            await self.default_body(context)


@dataclass(frozen=True)
class Motion(Statement):
    r"""
    Example:
    >>> code = '''
    ... home = (0, 0, 0, 0, 0, 0)
    ... move via p2p() to home with blending(10)
    ... '''
    >>> _ = _run_program_debug(code)
    """

    connector: Connector
    end: Expression[Pose] | Expression[Vector3d]
    modifier: Modifier | None = None
    tcp: Expression[t.Frame] | None = None
    frame_relation: FrameRelation | None = None

    def __post_init__(self):
        if self.connector is None:
            object.__setattr__(self, "connector", Connector("p2p", ()))

    async def call(self, context: ExecutionContext, **kwargs):
        tcp: Any
        on_exit = await self.modifier(context) if self.modifier else None
        end = await self.end(context)
        # TODO: this is a hack to get the location to the default connector set in __post_init__
        if self.connector.location is None:
            self.connector.set_location(self.location)  # type: ignore
        if self.frame_relation:
            assert self.tcp is None
            source = await self.frame_relation.source(context)
            target = await self.frame_relation.target(context)

            if isinstance(source, AbstractRobot):
                start = context.action_queue.last_pose(source.id)
                await self.connector(
                    context, start=start, end=end, tool=target.name, robot=source.id
                )
                return
            if isinstance(end, Vector3d):
                raise wandelscript.exception.ProgramSyntaxError(
                    location=self.location, text="No position is supported when here"
                )
            fs = context.store.frame_system.copy()
            fs[
                (await self.frame_relation.target(context)).name,
                (await self.frame_relation.source(context)).name,
            ] = pose_to_versor(end)
            end = versor_to_pose(fs.eval(context.store.ROBOT.name, context.store.FLANGE.name))
            tcp = None  # TODO can we still allow this
        elif self.tcp:
            tcp = await self.tcp(context)
            tcps_in_cell = await context.robot_cell.tcps

            # validate TCP
            if tcp.name not in tcps_in_cell:
                raise wandelscript.exception.UserError(
                    location=None,
                    value=f"No robot with the  tool: '{tcp.name}'. Available tools: {list(tcps_in_cell.keys())}",
                )
            if context.active_robot not in tcps_in_cell[tcp.name]:
                raise wandelscript.exception.WrongRobotError(
                    location=None,
                    text=f"Tool '{tcp.name}' is not attached to the active robot '{context.active_robot}'",
                )
            tcp = tcp.name  # TODO messy
        else:
            tcp = context.default_tcp
            if tcp is None:
                msg = "No tool is defined. Please define one using the 'tcp' function!"
                raise wandelscript.exception.UserError(location=self.location, value=msg)

        robot_identifier = context.active_robot
        start = context.action_queue.last_pose(robot_identifier)
        await self.connector(
            context,
            start=start,
            end=end,
            tool=tcp if tcp is not None else None,
            robot=robot_identifier,
        )
        if on_exit:
            await on_exit(context)  # pylint: disable=not-callable


@dataclass(frozen=True)
class Pass(Statement):
    """A no operation

    Example:
    >>> code = 'pass'
    >>> _ = _run_program_debug(code)
    """

    async def call(self, context: ExecutionContext, **kwargs):
        pass


@dataclass(frozen=True)
class RaiseException(Statement):
    """Raise an exception

    Example:
    >>> code = 'raise "Tool not working"'
    >>> _run_program_debug(code)
    Traceback (most recent call last):
     ...
    wandelscript.exception.UserError: User defined error: 'Tool not working'
    """

    value: Expression[str]

    async def call(self, context: ExecutionContext, **kwargs):
        value = await self.value(context)
        raise wandelscript.exception.UserError(location=None, value=value)


class Atom(Rule, Generic[ElementType], ABC):
    r"""The elementary class to handle mathematical expressions

    Example:
    >>> a, b = ConstantInt(3), ConstantInt(5)
    >>> asyncio.run(run_rule((a + b)))
    8
    >>> asyncio.run(run_rule((a - b)))
    -2
    >>> asyncio.run(run_rule((a * b)))
    15
    >>> asyncio.run(run_rule((a / b)))
    0.6
    >>> asyncio.run(run_rule((a < b)))
    True
    >>> asyncio.run(run_rule((a <= b)))
    True
    >>> asyncio.run(run_rule((a > b)))
    False
    >>> asyncio.run(run_rule((a >= b)))
    False
    >>> asyncio.run(run_rule((a == b)))
    False
    >>> asyncio.run(run_rule((a != b)))
    True
    >>> code = '''
    ... a = (0, 0, 5, 0, 0, 0)
    ... b = (1, 2, 3, 0, 0, 0)
    ... c = a :: b
    ... '''
    >>> store = _run_program_debug(code).store
    >>> store['c']
    Pose(position=Vector3d(x=1.0, y=2.0, z=8.0), orientation=Vector3d(x=0.0, y=0.0, z=0.0))
    """

    @abstractmethod
    async def call(self, context: ExecutionContext, **kwargs) -> ElementType:
        pass

    def to_expression(self) -> Expression[ElementType]:
        return Expression(a=self)

    def simplify(self) -> Atom[ElementType]:
        """Simplify the expressions (e.g., removing unnecessary nesting, i.e. removing unnecessary parenthesis)

        Returns:
            A simpler representation of the same mathematical term
        """
        return self

    def __mul__(self, other: Atom | Multiplication | Unary) -> Atom[Any]:
        if isinstance(other, float):  # type: ignore
            return self * ConstantFloat(other)  # type: ignore
        if isinstance(other, int):  # type: ignore
            return self * ConstantInt(other)  # type: ignore
        other = other.to_expression()
        return Expression(a=Multiplication(a=self, b=(other,), op=(MultiplicationOperator.mul,)))

    def __matmul__(self, other: Atom | Multiplication | Unary) -> Atom[Any]:
        if isinstance(other, (float, int)):  # type: ignore
            return NotImplemented  # type: ignore
        other = other.to_expression()
        return Expression(a=Multiplication(a=self, b=(other,), op=(MultiplicationOperator.matmul,)))

    def __truediv__(self, other: Atom | Multiplication) -> Atom[Any]:
        other = other.to_expression()
        return Expression(
            a=Multiplication(a=self, b=(other,), op=(MultiplicationOperator.truediv,))
        )

    def __add__(self, other: Atom[ElementType] | Multiplication[ElementType]) -> Atom[ElementType]:
        other = other.to_expression()
        return Expression(a=Addition(a=self, b=(other,), op=(AdditionOperator.add,)))

    def __sub__(self, other: Atom[ElementType] | Multiplication[ElementType]) -> Atom[ElementType]:
        other = other.to_expression()
        return Expression(a=Addition(a=self, b=(other,), op=(AdditionOperator.sub,)))

    def __pos__(self):
        return self

    def __neg__(self):
        return Expression(a=Addition(a=self, b=(), op=(), op_a=Sign.neg))

    def __invert__(self):
        return Expression(a=Unary(self, Inverse.inv))

    def __lt__(self, other: Atom[ElementType] | Multiplication[ElementType]) -> Atom[ElementType]:
        other = other.to_expression()
        return Expression(a=self, b=(other,), op=(ComparisonOperator.lt,))

    def __le__(self, other: Atom[ElementType] | Multiplication[ElementType]) -> Atom[ElementType]:
        other = other.to_expression()
        return Expression(a=self, b=(other,), op=(ComparisonOperator.le,))

    def __gt__(self, other: Atom[ElementType] | Multiplication[ElementType]) -> Atom[ElementType]:
        other = other.to_expression()
        return Expression(a=self, b=(other,), op=(ComparisonOperator.gt,))

    def __ge__(self, other: Atom[ElementType] | Multiplication[ElementType]) -> Atom[ElementType]:
        other = other.to_expression()
        return Expression(a=self, b=(other,), op=(ComparisonOperator.ge,))

    def __eq__(self, other: Atom[ElementType] | Multiplication[ElementType]) -> Atom[ElementType]:  # type: ignore
        other = other.to_expression()
        return Expression(a=self, b=(other,), op=(ComparisonOperator.eq,))

    def __ne__(self, other) -> Atom[ElementType]:  # type: ignore
        other = other.to_expression()
        return Expression(a=self, b=(other,), op=(ComparisonOperator.ne,))

    def _custom_not(self):
        """Python's `not uses `MyClass.__bool__(), however, `__bool__()`, must
        return a bool, otherwise throws a TypeError. Particularly, `__bool__()`
        can not return an expression. Thus, implement our own custom function
        for `not` operations.

        Returns:
            an Expression
        """
        return Expression(a=Unary(self, Not.not_))

    def _custom_and(self, other):
        other = other.to_expression()
        return Expression(a=Addition(a=self, b=(other,), op=(LogicalOperator.and_,)))

    def _custom_or(self, other):
        other = other.to_expression()
        return Expression(a=Addition(a=self, b=(other,), op=(LogicalOperator.or_,)))


@dataclass(frozen=True, eq=False)
class Constant(Atom[ElementType]):
    r"""Stores (typed) values

    Example:
    >>> code = '''
    ... a = (1.0, 2.0, 3.0)
    ... b = 3
    ... c = 4.5
    ... '''
    >>> store = _run_program_debug(code).store
    >>> store['a']
    Vector3d(x=1.0, y=2.0, z=3.0)
    >>> store['b']
    3
    >>> store['c']
    4.5
    """

    value: ElementType

    async def call(self, context: ExecutionContext, **kwargs) -> ElementType:
        return self.value

    def __str__(self):
        return str(self.value)


class ConstantFloat(Constant[float]):
    """A simple float value


    Example:
    >>> code = '''
    ... a = 1.3
    ... b = pi
    ... '''
    >>> store = _run_program_debug(code).store
    >>> store['a']
    1.3
    >>> store['b']
    3.141592653589793
    """


class String(Constant[str]):
    """A simple string value"""


class ConstantInt(Constant[int]):
    """A simple int value"""


class Bool(Constant[bool]):
    """A simple boolean value"""


class ConstantPosition(Constant[Vector3d]):
    """A simple vector value"""


class ConstantOrientation(Constant[Vector3d]):
    """A simple vector value"""


class ConstantPose(Constant[Pose]):
    """A simple pose value"""


@dataclass(frozen=True, eq=False)
class Array(Atom[tuple[t.ElementType, ...]]):
    """A list of elements (which can have different type)

    Example:
    >>> code = '''
    ... a = [1, 2, 3]
    ... '''
    >>> store = _run_program_debug(code).store
    >>> store['a']
    (1, 2, 3)
    """

    value: tuple[Atom[float] | Atom[str] | Atom[Pose], ...]

    async def call(self, context: ExecutionContext, **kwargs) -> tuple[t.ElementType, ...]:
        return tuple([await v(context) for v in self.value])  # pylint: disable=consider-using-generator


@dataclass(frozen=True, eq=False)
class KeyValuePair(Statement):
    key: str
    value: Atom

    async def call(self, context: ExecutionContext, **kwargs) -> tuple[str, t.ElementType]:
        return self.key, await self.value(context)


@dataclass(frozen=True, eq=False)
class Record(Atom[dict[str, t.ElementType]]):
    """A dictionary of key-value pairs, where values can have different types.

    Example:
    >>> code = '''
    ... record = { key1: 1, key2: "value", key3: (1, 2, 3) }
    ... a = record['key2']
    ... b = record.key3
    ... '''
    >>> store = _run_program_debug(code).store
    >>> store['record']
    {'key1': 1, 'key2': 'value', 'key3': Vector3d(x=1.0, y=2.0, z=3.0)}
    >>> store['a']
    'value'
    >>> store['b']
    Vector3d(x=1.0, y=2.0, z=3.0)
    """

    items: tuple[KeyValuePair, ...]

    async def call(self, context: ExecutionContext, **kwargs) -> dict[str, t.ElementType]:
        return {pair.key: await pair.value(context) for pair in self.items}


@dataclass(frozen=True, eq=False)
class PropertyAccess(Atom[ElementType]):
    variable: Atom[tuple[ElementType], ...]  # type: ignore
    key: str

    async def call(self, context: ExecutionContext, **kwargs) -> ElementType:
        variable = await self.variable(context)
        return variable[self.key]


@dataclass(frozen=True, eq=False)
class ExpressionsList(Atom[t.ElementType]):
    """A list of elements (which can have different type)"""

    value: tuple[Atom[float], ...]

    async def call(self, context: ExecutionContext, **kwargs) -> Vector3d | Pose:
        values = tuple([float(await v(context)) for v in self.value])
        if len(values) == 3:
            return Vector3d.from_tuple(values)
        if len(values) == 6:
            return Pose(values)
        raise wandelscript.exception.ProgramSyntaxError(
            None, f"Unexpected number of elements: {len(self.value)}"
        )

    def __str__(self):
        return "[" + ",".join(map(str, self.value)) + "]"


@dataclass(frozen=True, eq=False)
class Break(Statement):
    """Break statement to stop the current loop and continue after that loop

    Example:
    >>> code = '''
    ... a = 10
    ... for i in 3 .. 5:
    ...     break
    ...     a = 5
    ... '''
    >>> store = _run_program_debug(code).store
    >>> store["a"]
    10
    """

    async def call(self, context: ExecutionContext, **kwargs):
        raise wandelscript.exception.BreakSignal


@dataclass(frozen=True, eq=False)
class Expression(Atom[ElementType]):
    r"""The (top-level) type of handling various expressions

    Example:
    >>> code = '''
    ... a = 1
    ... b = 2
    ... c = (a < b)
    ... d = (a >= b)
    ... '''
    >>> store = _run_program_debug(code).store
    >>> store['c']
    True
    >>> store['d']
    False
    """

    a: Atom[ElementType]
    b: tuple[Atom[ElementType], ...] = ()
    op: tuple[ComparisonOperator, ...] = ()

    async def call(self, context: ExecutionContext, **kwargs) -> ElementType:
        result_a = await self.a(context)
        # assert isinstance(result, (bool, float, int, str, Vector, Pose)), result
        if self.b:
            assert len(self.b) == 1, "Otherwise not implemented"
            result_b = await self.b[0](context)
            return self.op[0](result_a, result_b)
        return result_a

    def simplify(self) -> Atom[ElementType]:
        return self if self.b else self.a.simplify()

    def __str__(self):
        return "".join(map(str, [self.a, *chain(*zip(self.op, self.b))]))


@dataclass(frozen=True, eq=False)
class Unary(Atom[ElementType]):
    """Inverting (for poses)

    Example:
    >>> code = '''
    ... a = ~(1.0, 2.0, 3.0, 0, 0, 0)
    ... '''
    >>> store = _run_program_debug(code).store
    >>> store['a']
    Pose(position=Vector3d(x=-1.0, y=-2.0, z=-3.0), orientation=Vector3d(x=0.0, y=0.0, z=0.0))
    """

    a: Atom[ElementType]
    op: Inverse | Not | None = None

    async def call(self, context: ExecutionContext, **kwargs) -> ElementType:
        result = await self.a(context)
        if self.op:
            result = self.op(result)
        return result

    def to_expression(self) -> Expression[ElementType]:
        return Expression(a=self)


@dataclass(frozen=True)
class Conditional(Statement):
    r"""The if-elif-else condition

    Example:
    >>> code = '''
    ... a = 0
    ... if 2 > 1:
    ...     a = 10
    ... else:
    ...     a = 1
    ... '''
    >>> store = asyncio.run(run_program(code)).store
    >>> store['a']
    10
    """

    condition: Expression[bool]
    body: Suite
    elif_condition: tuple[Expression[bool], ...]
    elif_body: tuple[Suite, ...]
    else_body: Suite | None = None

    async def call(self, context: ExecutionContext, **kwargs):
        for condition, body in zip(
            [self.condition, *self.elif_condition], [self.body, *self.elif_body]
        ):
            if await condition(context):
                await body(context)
                break
        else:
            if self.else_body is not None:
                await self.else_body(context)


@dataclass(frozen=True)
class Print(Statement):
    r"""A simple Python print statement that print a given text

    Example:
    >>> code = '''
    ... print("Hello Wandelscript")
    ... '''
    >>> _ = _run_program_debug(code)
    Hello Wandelscript
    """

    text: Expression[str]

    async def call(self, context: ExecutionContext, **kwargs):
        text = str(await self.text(context))
        print(text)


@dataclass(frozen=True)
class WhileLoop(Statement):
    r"""A while loop

    Example:
    >>> code = '''
    ... i = 0
    ... while i < 100:
    ...     i = i + 13
    ... '''
    >>> store = _run_program_debug(code).store
    >>> store['i']
    104
    """

    condition: Expression[bool]
    body: Suite

    async def call(self, context: ExecutionContext, **kwargs):
        while await self.condition(context):
            try:
                await self.body(context)
            except wandelscript.exception.BreakSignal:
                break


@dataclass(frozen=True, eq=False)
class Return(Statement):
    """Return statement to stop executing a function and optionally returning a value"""

    result: Expression

    async def call(self, context: ExecutionContext, **kwargs):
        result = await self.result(context)
        raise wandelscript.exception.ReturnSignal(result)


@dataclass(frozen=True)
class Connector(Rule):
    """A motion connecting two poses by a continuous path"""

    name: str
    args: tuple[Expression, ...] = ()

    class Impl(Factory, ABC, func_name=None):
        @dataclass
        class Args:
            pass

        @abstractmethod
        def __call__(
            self, start: Pose, end: Pose, args: Any, motion_settings: MotionSettings
        ) -> Motion | tuple[Motion, ...]:
            pass

    async def call(self, context: ExecutionContext, **kwargs):
        async def kwargs_packed(start: Pose | None, end: Pose | Vector3d, *, tool: str, robot: str):
            if cls := self.Impl.from_name(self.name):
                func = cls()
                args = cls.Args(*[(await var(context)) for var in self.args])
                motion_settings = context.store.get_motion_settings()
                if isinstance(end, Vector3d):
                    # TODO: follow-up handle more orientation strategies
                    end = add_orientation("last", end, start)
                context.action_queue.push(
                    motions=func(start, end, args, motion_settings),
                    tool=tool,
                    motion_group_id=robot,
                )
            elif func_ := context.store.get(self.name, None):
                func = func_
                func(start, end, context, [var(context) for var in self.args])
            else:
                raise wandelscript.exception.NameError_(location=self.location, name=self.name)

        await kwargs_packed(**kwargs)  # pylint: disable=missing-kwoa


def register_builtin_func(name: str | None = None, pass_context: bool = False):
    """Decorator to make Python functions callable from wandelscript.

    Args:
        name: how the function is named inside wandelscript.
        pass_context: if the decorated function should receive the
            execution context (currently the store) as the first argument.

    Returns:
        The decorator.
    """

    def decorator(func):
        FunctionCall.register_builtin(func, name, pass_context)
        return func

    return decorator


def register_debug_func(func):
    FunctionCall.register_builtin(func, name=None, pass_context=True)
    return func


@dataclass(frozen=True, eq=False)
class Addition(Atom[ElementType]):
    r"""A addition expression with two parameters

    Example:
    >>> code = '''
    ... a = 5
    ... b = 3
    ... c = a + b
    ... d = a - b
    ... e = + a
    ... f = -a
    ... '''
    >>> store = _run_program_debug(code).store
    >>> store['c']
    8
    >>> store['d']
    2
    >>> store['e']
    5
    >>> store['f']
    -5
    """

    a: Atom[ElementType]
    op_a: Sign | None = None
    b: tuple[Atom[ElementType], ...] = ()
    op: tuple[AdditionOperator, ...] = ()

    async def call(self, context: ExecutionContext, **kwargs) -> ElementType:
        # TODO: can we make this more generic?
        result = await self.a(context)
        if self.op_a:
            result = self.op_a(result)
        # assert isinstance(result, (bool, float, int, str, Vector, Pose)), result
        for value, operation in zip(self.b, self.op):
            result = operation(result, await value(context))
        return result

    def simplify(self) -> Atom[ElementType]:
        return self if self.b else self.a.simplify()

    def __str__(self):
        return "".join(map(str, [self.a, *chain(*zip(self.op, self.b))]))


@dataclass(frozen=True, eq=False)
class Multiplication(Atom[ElementType]):
    a: Atom[ElementType]
    # (Union[float, int] is not a element type)
    b: tuple[Atom[float | int], ...] = ()
    op: tuple[MultiplicationOperator, ...] = ()

    async def call(self, context: ExecutionContext, **kwargs) -> ElementType:
        result = await self.a(context)
        for value, operation in zip(self.b, self.op):
            result = operation(result, await value(context))
        return result

    def __mul__(self, other: Atom | "Multiplication") -> Atom:
        return self.to_expression() * other.to_expression()

    def __truediv__(self, other: Atom | "Multiplication") -> Atom:
        return self.to_expression() / other.to_expression()

    def __add__(self, other: Atom | "Multiplication"):
        return self.to_expression() + other.to_expression()

    def __sub__(self, other: Atom | "Multiplication"):
        return self.to_expression() - other.to_expression()

    def to_expression(self) -> Expression[ElementType]:
        return Expression(a=self)

    def simplify(self) -> Atom[ElementType]:
        return self if self.b else self.a.simplify()

    def __str__(self):
        return "".join(map(str, [self.a, *chain(*zip(self.op, self.b))]))


@dataclass(frozen=True, eq=False)
class Reference(Atom[ElementType]):
    """Access to a variable in the namespace

    Example:
    >>> code = '''
    ... a = 3
    ... b = a * 2
    ... '''
    >>> store = _run_program_debug(code).store
    >>> store["b"]
    6
    """

    name: str

    async def call(self, context: ExecutionContext, **kwargs) -> ElementType:
        try:
            return context.store[self.name]
        except KeyError as error:
            raise wandelscript.exception.NameError_(
                location=self.location, name=self.name
            ) from error

    def to_expression(self) -> Expression[ElementType]:
        return Expression(a=self)


@dataclass(frozen=True, eq=False)
class FrameRelation(Atom[Pose]):
    """The pose describing the relation between two frames (coordinate systems)

    Example:
    >>> code = '''
    ... a = frame("a")
    ... b = frame("b")
    ... c = frame("c")
    ... [a | b] = (0, 0, 10, 0, 0, 0)
    ... [b | c] = (0, 10, 10, 0, 0, 0)
    ... pose = [a | c]
    ... '''
    >>> store = _run_program_debug(code).store
    >>> store["pose"]
    Pose(position=Vector3d(x=0.0, y=10.0, z=20.0), orientation=Vector3d(x=0.0, y=0.0, z=0.0))
    """

    target: Reference
    source: Reference

    async def call(self, context: ExecutionContext, **kwargs) -> Pose:
        target = await self.target(context)
        source = await self.source(context)
        if isinstance(target, t.Frame) and isinstance(source, t.Frame):
            fs = context.store.frame_system.copy()
            if (
                current_pose_of_robot := context.action_queue.last_pose(context.active_robot)
            ) is not None:
                fs[context.store.ROBOT.name, context.store.FLANGE.name] = pose_to_versor(
                    current_pose_of_robot
                )
            return versor_to_pose(fs.eval(target.name, source.name))
        if isinstance(target, t.Frame) ^ isinstance(source, t.Frame):
            raise TypeError("Either both or neither of the two arguments must be of type 'Frame'")
        raise TypeError("Both arguments must be of type 'Frame'")
        # See: https://code.wabo.run/ai/wandelbrain/-/blob/main/packages/pyjectory/pyjectory/visiontypes/body.py
        # return versor_to_pose(estimate_pose(source, target)[0])


@dataclass(frozen=True, eq=False)
class Assignment(Atom[ElementType], Statement):
    r"""Assign a value to a variable

    Example:
    >>> code = '''
    ... a = 2 + 2
    ... b = 'a string'
    ... c = (d = 2) + 4
    ... e = [1, 2, 3]
    ... '''
    >>> store = _run_program_debug(code).store
    >>> store["a"]
    4
    >>> store["b"]
    'a string'
    >>> store["c"]
    6
    >>> store["d"]
    2
    >>> store["e"]
    (1, 2, 3)
    """

    name: str | tuple[str] | FrameRelation
    value: Atom[ElementType]

    async def call(self, context: ExecutionContext, **kwargs) -> ElementType:
        value = await self.value(context)
        if isinstance(self.name, (list, tuple)):
            if isinstance(value, (list, tuple)):
                context.store.update_local(zip(self.name, value))
            else:
                raise TypeError
            return tuple(value)  # type: ignore
        if isinstance(self.name, FrameRelation):
            if not isinstance(value, Pose):
                raise TypeError(f"Unexpected type: Expected Pose but received: {type(value)}")
            try:
                target = await self.name.target(context)
                assert isinstance(target, t.Frame)
            except wandelscript.exception.NameError_:
                target = context.store[self.name.target.name] = t.Frame(
                    self.name.target.name, context.store.frame_system
                )
                assert isinstance(target, t.Frame)
            try:
                source = await self.name.source(context)
            except wandelscript.exception.NameError_:
                source = context.store[self.name.source.name] = t.Frame(
                    self.name.source.name, context.store.frame_system
                )
            assert isinstance(target, t.Frame), (target, source)
            assert isinstance(source, t.Frame), (target, source)

            context.store.frame_system[target.name, source.name] = pose_to_versor(value)
            return value
        context.store[self.name] = value
        return context.store[self.name]


BUILTINS = (FilePath(__file__).parent / "builtins.ws").open(encoding="utf-8").read()
# PLUGINS_ADDONS = (FilePath(__file__).parent / "plugins_addons.ws").open(encoding="utf-8").read()
PLUGINS = BUILTINS  # + PLUGINS_ADDONS


@dataclass(frozen=True)
class RootBlock(Rule):
    body: Block

    async def call(self, context: ExecutionContext, **kwargs):
        await plugins()(
            context
        )  # TODO move to somewhere where it is called not for every test (maybe at startup)
        await self.body(context)
        await context.sync()


@dataclass(frozen=True)
class Program:
    r"""The root of the meta-model

    Example:
    >>> import numpy as np
    >>> code = '''
    ... tcp("Flange")
    ... move via p2p() to (0, 0, 0, 0, 0, 0)
    ... move via line() to (1, 1, 0, 0, -pi, 0)
    ... move via arc((1, 2, 0, 0, -pi, 0)) to (2, 2, 0, 0, -pi, 0)
    ... move via line() to (2, 2, 1, -pi, 0, 0)
    ... '''
    >>> program = Program.from_code(code)
    >>> with np.printoptions(precision=2, suppress=True):
    ...     trajectory = program.simulate()
    >>> assert len(trajectory) > 0
    """

    body: RootBlock

    async def __call__(self, context: ExecutionContext):
        for tcp in await context.robot_cell.tcps:
            context.store[tcp] = t.Frame(tcp, context.store.frame_system)
        await self.body(context)

    @classmethod
    def from_file(cls, filename: str) -> Program:
        return cls.from_code(open(filename, encoding="utf-8").read())

    def simulate(self, run_args: dict[str, t.ElementType] | None = None):
        context = asyncio.run(run_program(self, run_args, default_robot="0@controller"))
        return context.motion_group_recordings

    @staticmethod
    def from_code(code: str) -> Program:
        raise NotImplementedError


@dataclass(frozen=True)
class Parameters(Rule):
    """Parameters (e.g., of a function)

    Example:
    >>> code = '''
    ... def func(param1, param2):
    ...     return param1 + param2
    ... a = func(3, 4)
    ... '''
    >>> store = _run_program_debug(code).store
    >>> store["a"]
    7
    """

    names: tuple[str, ...]

    async def call(self, context: ExecutionContext, **kwargs) -> tuple[str, ...]:
        return self.names


@dataclass(frozen=True)
class Arguments(Rule):
    r"""A list of arguments

    Example:
    >>> code = "a = [1, 2, 3, 4, [5, 6]]\n"
    >>> context = _run_program_debug(code)
    >>> context.store["a"]
    (1, 2, 3, 4, (5, 6))
    """

    data: tuple[Expression, ...]

    async def call(self, context: ExecutionContext, **kwargs) -> tuple:
        # pylint: disable=consider-using-generator
        return tuple([await expression(context) for expression in self.data])


@dataclass(frozen=True, eq=False)
class FunctionCall(Atom[ElementType], Statement):
    r"""
    Example: an inefficient(!) way to implement the power function
    >>> import wandelscript.builtins
    >>> code = '''
    ... def power2(a, e):
    ...     if e:
    ...         result = a * power2(a, e - 1)
    ...     else:
    ...         result = 1
    ...     return result
    ... a = power2(3, 4)
    ... b = power(3, 4)
    ... '''
    >>> store = _run_program_debug(code).store
    >>> store['a']
    81
    >>> store['b']
    81
    """

    @dataclass(frozen=True)
    class Builtin:
        """Describes a builtin wandelscript function.

        Attributes:
            func: the python callable representing the wandelscript function
            name: how the function is named inside wandelscript
            pass_context: whether func should receive the execution context as the first argument
        """

        func: Callable
        name: str
        pass_context: bool = False

    name: str
    arguments: Arguments
    _builtins: ClassVar[dict[str, Builtin]] = {}

    @classmethod
    def register_builtin(cls, func: Callable, name: str | None, pass_context: bool = False):
        """Registers a Python callable as a function callable from wandelscript.

        Args:
            func: the python callable representing the wandelscript function
            name: how the function is named inside wandelscript. If None or the empty string "" is given,
            use the function's __name__.
            pass_context: whether func should receive the execution context as the first argument
        """
        script_func_name = name if name else func.__name__
        cls._builtins[script_func_name] = cls.Builtin(func, script_func_name, pass_context)

    def __post_init__(self, *_args):
        assert isinstance(self.arguments, Arguments)

    async def call(self, context: ExecutionContext, **kwargs) -> ElementType:
        arguments = await self.arguments(context)

        if self.name in self._builtins:
            builtin = self._builtins[self.name]
            call_args = [context] + list(arguments) if builtin.pass_context else arguments
            if inspect.iscoroutinefunction(builtin.func):
                return await builtin.func(*call_args)
            return builtin.func(*call_args)
        if self.name == "planned_pose":
            result = context.action_queue.last_pose(context.active_robot)
            if result is None:
                raise RuntimeError(
                    "Before planned pose can be used, a move commands needs to be executed"
                )
            return result
        if self.name == "frame":
            assert len(arguments) == 1
            assert isinstance(arguments[0], str)
            return t.Frame(arguments[0], context.store.frame_system)  # type: ignore
        if func := context.store.get(self.name, None):
            if isinstance(func, Pose):
                if isinstance(arguments[0], Pose):
                    assert len(arguments) == 1
                    return versor_to_pose(pose_to_versor(func).apply(pose_to_versor(arguments[0])))
                return pose_to_versor(func).apply(*arguments)
            return await func(context, *arguments)
        print(self._builtins)
        raise wandelscript.exception.NameError_(
            location=self.location, name=self.name
        )  # (f"Function is not defined: {self.name}")


@dataclass(frozen=True)
class Interrupt(Statement):
    r"""Being able to react to specific signals, e. g. when waiting for an input from an asynchronous device for
    every automation process. An interrupt can be used to listen to an event and executing statements when the
    event occurs.

    Note:
        We don't recommend to use this interrupt for safety critical events since those needs to be handled
        directly on the robot controller.

    A interrupt can be defined with
    ```
    interrupt <name>() when <condition>:
        <statements>
    ```

    A interrupt needs to be activated with `activate <name>` in order to listen to the event and can be deactivated
    with `deactivate <name>`.

    TODO: refine & describe condition syntax

    Example:
    >>> code = '''
    ... b = 20
    ... interrupt inter1() when is_equal("input4", 2 + b):
    ...     a = 12
    ... home = (0, 0, 0, 0, 0, 0)
    ... activate inter1
    ... move via p2p() to home
    ... deactivate inter1
    ... '''
    >>> _ = _run_program_debug(code)
    """

    name: str
    parameters: Parameters
    condition: str
    arguments: Arguments
    body: Suite

    async def call(self, context: ExecutionContext, **kwargs):
        async def func(store, *args):
            init_locals = dict(zip(await self.parameters(context), args))
            with context.new_call_frame(store, init_locals):
                try:
                    await self.body(context)
                except wandelscript.exception.ReturnSignal as e:
                    result = e.value
                else:
                    result = None
                return result

        arguments = await self.arguments(context)
        context.store[self.name] = (self.condition, arguments, t.Closure(context.store, func))


@dataclass(frozen=True)
class FunctionDefinition(Statement):
    """The definition of a function"""

    name: str
    parameters: Parameters
    body: Block

    async def call(self, context: ExecutionContext, **kwargs):
        async def func(store, *args):
            init_locals = dict(zip(await self.parameters(context), args))
            with context.new_call_frame(store, init_locals):
                try:
                    await self.body(context)
                except wandelscript.exception.ReturnSignal as e:
                    result = e.value
                else:
                    result = None
                return result

        context.store[self.name] = t.Closure(context.store, func)


@dataclass(frozen=True)
class MoveDefinition(Statement):
    # TODO: this class is not used and maintained right now
    name: str
    start: str
    end: str
    body: Suite
    parameters: Parameters

    async def call(self, context: ExecutionContext, **kwargs):
        async def func(start: Pose, end: Pose, store: Store, _args=()):
            init_locals = {start: start, end: end}
            if self.parameters:
                init_locals.update(zip(self.parameters.names, _args))
            with context.new_call_frame(store, init_locals):
                await self.body(context)

        Connector.Impl.register_func(context, self.name, func)


@dataclass(frozen=True, eq=False)
class IndexAccess(Atom[ElementType]):
    variable: Atom[tuple[ElementType], ...]  # type: ignore
    key: Expression

    async def call(self, context: ExecutionContext, **kwargs) -> ElementType:
        key = await self.key(context)
        # if not isinstance(key, int) or key < 0:
        #     raise ValueError(f"Key must be a positive integer but is: {key}")
        variable = await self.variable(context)
        return variable[key]


@dataclass(frozen=True, eq=False)
class Read(Atom[ElementType]):
    """A read statement

    Example:
    >>> code = 'a = read(controller, "a")'
    >>> store = _run_program_debug(code).store
    Get default_value from 'a'
    >>> store['a']
    'default_value'
    """

    device: Expression
    key: Expression[str]

    async def call(self, context: ExecutionContext, **kwargs) -> ElementType:
        key = await self.key(context)
        device = await self.device(context)

        if not isinstance(device, (InputDevice, AbstractRobot)):
            raise GenericRuntimeError(
                self.location, text=f"{device.id} does not support the read operation"
            )
        # classify read action according to user input
        try:
            if isinstance(device, AbstractRobot):
                if key == "pose":
                    action = ReadPoseAction(device_id=device.id, tcp=context.default_tcp)
                elif key == "joints":
                    action = ReadJointsAction(device_id=device.id)
                else:
                    # read pose from robot with given tcp offset
                    if isinstance(key, t.Frame):
                        key = key.name
                    action = ReadPoseAction(device_id=device.id, tcp=key)
            else:
                action = ReadAction(device_id=device.id, key=key)

            return await context.action_queue.run_action(action)
        except UnknownPose as e:
            raise wandelscript.exception.ConfigurationError(location=self.location) from e


@dataclass(frozen=True)
class Write(Statement, Generic[ElementType]):
    r"""A write statement

    Example:
    >>> code = 'write(controller, "a", 12 * 2)'
    >>> context = _run_program_debug(code)
    Set 'a' to 24
    >>> asyncio.run(context.robot_cell['controller'].read('a'))
    Get 24 from 'a'
    24
    """

    device: Expression
    key: Expression[str]
    value: Expression

    async def call(self, context: ExecutionContext, **kwargs):
        key = await self.key(context)
        value = await self.value(context)
        device = await self.device(context)
        if not isinstance(device, OutputDevice):
            raise GenericRuntimeError(
                self.location, text=f"{device.id} does not support the write operation"
            )

        if not isinstance(key, str):
            raise GenericRuntimeError(
                self.location,
                text=f"Key must be a string but is: {key=}. Use correct order: write(<device>, <key: str>, <value>)",
            )

        # classify write action according to user input
        action = WriteAction(device_id=device.id, key=key, value=value)

        # run write action based on sync context
        if context.action_queue.is_empty():
            await context.action_queue.run_action(action)
        else:
            # attach action to path segment in order to trigger on path
            context.action_queue.attach_action(action, context.active_robot)


@dataclass(frozen=True, eq=False)
class Call(Atom[ElementType], Statement):
    r"""A call statement that calls a function on a device

    Example:
    >>> code = 'a = call(sensor, "key", 12 * 2, 3)'
    >>> store = _run_program_debug(code).store
    >>> store['a']
    ('key', (24, 3))
    """

    device: Expression
    key: Expression[str]
    arguments: Arguments

    def __post_init__(self, *_args):
        assert isinstance(self.arguments, Arguments)

    async def call(self, context: ExecutionContext, **kwargs):
        key = await self.key(context)
        arguments = await self.arguments(context)
        device = await self.device(context)
        if not isinstance(device, AsyncCallableDevice):
            raise GenericRuntimeError(
                self.location, text=f"{device.id} does not support the call operation."
            )
        try:
            action = CallAction(device_id=device.id, key=key, arguments=arguments)
            result = await context.action_queue.run_action(action)
        except TypeError as e:
            # accepting the leakage of the Python error message for now
            raise GenericRuntimeError(self.location, text=str(e)) from e
        return result


@dataclass(frozen=True)
class Wait(Statement):
    """Wait for a given time in ms

    >>> code = "wait 3 + 4"
    >>> _ = _run_program_debug(code)
    Wait for 7 ms
    """

    duration: Expression[float]

    async def call(self, context: ExecutionContext, **kwargs):
        duration = await self.duration(context)
        await context.wait(duration)


@cache
def plugins() -> Block:
    return Program.from_code(PLUGINS).body.body


async def run_program(
    program: Program | str,
    cell: RobotCell | None = None,
    default_robot: str | None = None,
    default_tcp: str | None = None,
    run_args: dict[str, t.ElementType] | None = None,
    debug: bool = True,
) -> ExecutionContext:
    if isinstance(program, str):
        program = Program.from_code(program)
    if cell is None:
        cell = SimulatedRobotCell()
    stop_event = anyio.Event()
    context = ExecutionContext(
        cell,
        stop_event,
        default_robot=default_robot,
        default_tcp=default_tcp,
        run_args=run_args,
        debug=debug,
    )
    async with cell:
        await program(context)
    return context


def _run_program_debug(
    program: Program | str, default_robot: str | None = "0@controller"
) -> ExecutionContext:
    return asyncio.run(
        run_program(program, default_robot=default_robot, default_tcp="Flange", debug=True)
    )


async def run_rule(rule: Rule, **kwargs):
    stop_event = anyio.Event()
    context = ExecutionContext(robot_cell=SimulatedRobotCell(), stop_event=stop_event)
    return await rule(context, **kwargs)


# pylint: disable=too-many-lines
