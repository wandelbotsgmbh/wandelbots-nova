from __future__ import annotations

import asyncio
import contextvars
from collections.abc import AsyncIterable, Callable, Coroutine, Generator, Iterator, Mapping
from contextlib import contextmanager
from functools import singledispatch
from math import inf, isinf
from typing import Any

import anyio
from aiostream import stream
from loguru import logger

import wandelscript.metamodel as metamodel
from nova.actions import Action, CombinedActions
from nova.actions.container import ActionLocation
from nova.actions.io import CallAction, ReadAction, ReadJointsAction, ReadPoseAction, WriteAction
from nova.actions.motions import Motion
from nova.cell.robot_cell import AbstractRobot, Device, RobotCell
from nova.types import MotionSettings, MotionState, Pose
from wandelscript import exception as wsexception
from wandelscript.datatypes import ElementType, Frame, as_builtin_type
from wandelscript.exception import MotionError, NotPlannableError
from wandelscript.ffi import ForeignFunction
from wandelscript.frames import FrameSystem
from wandelscript.utils.runtime import stoppable_run
from wandelscript.utils.serializer import encode, is_encodable

DEFAULT_CALL_STACK_SIZE = 64
"""Default size of the call stack. Currently arbitrary."""

current_execution_context_var: contextvars.ContextVar = contextvars.ContextVar(
    "current_execution_context_var"
)


class Store:
    """Wandelscript runtime store to hold variables and their values"""

    def __init__(self, init_vars: Mapping[str, Any] | None = None, parent: Store | None = None):
        self.frame_system: FrameSystem = FrameSystem() if parent is None else parent.frame_system
        self._parent: Store | None = parent
        self._data: dict[str, Any] = {}
        # self._data.update(**self.environment.robot_cell)
        self.FLANGE = Frame("Flange", self.frame_system)
        self.ROBOT = Frame("robot_", self.frame_system)

        if init_vars:
            self._data.update(init_vars)

    def __getitem__(self, name: str) -> Any:
        scope_of_name = self.scope_of_name(name)
        if scope_of_name is None:
            raise KeyError(name)

        if scope_of_name is self:
            return self._data[name]

        return scope_of_name[name]

    def __setitem__(self, name: str, value: Any):
        scope_of_name = self.scope_of_name(name)
        if scope_of_name is None or scope_of_name is self:
            self._data[name] = value
        else:
            scope_of_name[name] = value

    def __contains__(self, name: str) -> bool:
        return self.scope_of_name(name) is not None

    def contains_local(self, name: str) -> bool:
        return name in self._data

    def get(self, name: str, default=None) -> Any:
        try:
            return self[name]
        except KeyError:
            return default

    def update_local(self, other: Mapping[str, Any]):
        self._data.update(other)

    def scope_of_name(self, name: str) -> Store | None:
        return next((scope for scope in self.scope_chain() if scope.contains_local(name)), None)

    def scope_chain(self) -> Generator[Store]:
        yield self
        if self._parent:
            yield from self._parent.scope_chain()

    def descent(self, init_vars: Mapping[str, Any] | None = None) -> Store:
        return Store(init_vars=init_vars, parent=self)

    @property
    def data(self) -> dict[str, Any]:
        return self._data.copy()

    @property
    def data_dict(self) -> dict[str, ElementType]:
        serialized_store = {k: encode(v) for k, v in self.data.items() if is_encodable(v)}
        serialized_store = {
            k: v for k, v in serialized_store.items() if not isinstance(v, float) or not isinf(v)
        }
        return serialized_store

    def get_motion_settings(self) -> MotionSettings:
        """Return the motion settings from the current scope

        Returns:
            The motion settings
        """
        return MotionSettings(
            **{
                field: self[MotionSettings.field_to_varname(field)]
                for field in MotionSettings.model_fields
                if MotionSettings.field_to_varname(field) in self
            }
        )


class ExecutionContext:
    _default_robot: str | None
    _active_robot: str | None = None
    _robot_ids: list[str]
    # Maps the motion group id to the list of recorded motion lists
    # Each motion list is a path the was planned separately
    # TODO: maybe we should make it public and helper methods to access the data
    motion_group_recordings: list[list[MotionState]]

    def __init__(  # pylint: disable=too-many-positional-arguments
        self,
        robot_cell: RobotCell,
        stop_event: anyio.Event,
        default_robot: str | None = None,
        default_tcp: str | None = None,
        run_args: dict[str, ElementType] | None = None,
        foreign_functions: dict[str, ForeignFunction] | None = None,
        debug: bool = False,
    ):
        self.motion_group_recordings = []
        self.robot_cell: RobotCell = robot_cell
        self._robot_ids = robot_ids = robot_cell.get_motion_group_ids()

        if default_robot is None:
            # If no default robot is set
            #   1) there is only one robot in the cell: use that one
            #   2) multiple robots in the cell: no default robot is set. When trying to execute code outside of a robot
            #       context an error will be raised
            self._default_robot = robot_ids[0] if len(robot_ids) == 1 else None
        else:
            # If a default robot is set, use it
            self._default_robot = default_robot

        if run_args is None:
            run_args = {}

        run_args.update(__tcp__=default_tcp, **robot_cell.devices)  # type: ignore

        for name, ff in (foreign_functions or {}).items():
            metamodel.register_builtin_func(name=name, pass_context=ff.pass_context)(ff.function)

        self.call_stack = CallStack(DEFAULT_CALL_STACK_SIZE)
        self.call_stack.push(Store(run_args))
        self.interceptors: list[Interceptor] = []
        self.stop_event: anyio.Event = stop_event
        # this will be continuously updated by the metamodel when the program is executed
        self.location_in_code: wsexception.TextRange | None = None
        self.debug = debug
        # This holds references to the tasks created by asyncio.create_task() during program execution.
        # This is necessary because of https://docs.python.org/3/library/asyncio-task.html#asyncio.create_task only
        # creating weak references for the in the event loop
        # also it might be sensible to store the data in thread local storage because it is probably
        # event_loop dependent
        self.asyncio_task_handles: set[asyncio.Task] = set()
        # creating the ActionQueue last since we pass self and therefore need to be initialized properly
        self.action_queue: ActionQueue = ActionQueue(self)

    def is_in_robot_context(self) -> bool:
        return self._active_robot is not None

    @property
    def default_robot(self) -> str | None:
        return self._default_robot

    @property
    def default_tcp(self) -> str | None:
        # TODO how can the __tcp__ naming be connected to the corresponding builtin function (tcp)
        return self.store.get("__tcp__", None)

    @property
    def active_robot(self) -> str:
        """Return the robot that should be used for execution.

        1. In a robot context:
            The robot that is currently active is returned.
        2: Outside of a robot context:
            The default robot is returned. If no default robot is set an error is raised.

        Returns:
            The robot that should be used for execution.

        Raises:
            WrongRobotError: If no default robot is set and the execution is outside of a robot context.

        """
        if not self._robot_ids:
            raise wsexception.WrongRobotError(
                text="No robot found in robot cell.", location=self.location_in_code
            )
        if self._active_robot is not None:
            return self._active_robot
        if self._default_robot is None:
            raise wsexception.WrongRobotError(
                text="No default robot found. Cannot execute outside of a robot context.",
                location=self.location_in_code,
            )
        return self._default_robot

    @contextmanager
    def with_robot(self, robot: str):
        if self.is_in_robot_context():
            raise ValueError(
                f"Cannot change to robot '{robot}' while another robot '{self.active_robot}' is active"
            )
        self._active_robot = robot
        try:
            yield self
        finally:
            self._active_robot = None

    @property
    def store(self):
        return self.call_stack.current_frame

    @contextmanager
    def new_call_frame(self, store: Store, init_locals: dict) -> Iterator[None]:
        self.call_stack.push(store.descent(init_vars=init_locals))
        yield
        self.call_stack.pop()

    async def sync(self):
        # TODO do we maybe also want to sync more things
        if self.is_in_robot_context():
            raise wsexception.NestedSyncError(location=self.location_in_code)
        await self.action_queue.run(self.stop_event)

    async def wait(self, duration: int):
        await self.robot_cell.timer(duration)

    def get_device(self, name: str) -> Device:
        return self.robot_cell[name]

    def get_motion_group(self, name=None) -> AbstractRobot:
        name = name or self.active_robot
        try:
            return self.robot_cell.get_motion_group(name)
        except KeyError as exc:
            raise wsexception.WrongRobotError(
                location=self.location_in_code, text=f"Unknown robot: '{name}'"
            ) from exc

    async def read_pose(self, robot_name: str, tcp: str | None = None) -> Pose:
        tcp = tcp or await self.get_motion_group(robot_name).active_tcp_name()
        robot_state = await self.get_motion_group(robot_name).get_state(tcp)
        return robot_state.pose

    async def read_joints(self, robot_name: str) -> tuple[float, ...] | None:
        tcp = await self.get_motion_group(robot_name).active_tcp_name()
        robot_state = await self.get_motion_group(robot_name).get_state(tcp)
        return robot_state.joints


class CallStack:
    """The call stack for a wandelscript program execution
    Attributes:
        max_frames: the maximum call depth.
    """

    def __init__(self, max_frames: int):
        self.max_frames: int = max_frames
        self._frames: list[Store] = []

    @property
    def current_frame(self):
        return self._frames[-1]

    def push(self, frame: Store) -> None:
        if len(self._frames) < self.max_frames:
            self._frames.append(frame)
        else:
            location = current_execution_context().location_in_code
            raise wsexception.GenericRuntimeError(
                location=location, text=f"Maximum call stack size of {self.max_frames} exceeded."
            )

    def pop(self) -> Store:
        return self._frames.pop()


Interceptor = Callable[[Coroutine, ExecutionContext], Coroutine]


def current_execution_context() -> ExecutionContext:
    return current_execution_context_var.get()


@singledispatch
def run_action(arg, context: ExecutionContext):
    raise NotImplementedError(f"_run not implemented for Action {type(arg)}")


@run_action.register(WriteAction)
async def _(arg: WriteAction, context: ExecutionContext) -> None:
    device = context.robot_cell.devices.get(arg.device_id)
    return await device.write(arg.key, arg.value)  # type: ignore


@run_action.register(ReadAction)
async def _(arg: ReadAction, context: ExecutionContext):
    device = context.robot_cell.devices.get(arg.device_id)
    return as_builtin_type(await device.read(arg.key))  # type: ignore


@run_action.register(ReadPoseAction)
async def _(arg: ReadPoseAction, context: ExecutionContext):
    return await context.read_pose(arg.device_id, arg.tcp)


@run_action.register(ReadJointsAction)
async def _(arg: ReadJointsAction, context: ExecutionContext):
    return await context.read_joints(arg.device_id)


@run_action.register(CallAction)
async def _(arg: CallAction, context: ExecutionContext) -> None:
    device = context.robot_cell.devices.get(arg.device_id)
    return await device(arg.key, *arg.arguments)  # type: ignore


class ActionQueue:
    """Collect actions from the program and processes them

    Implementation detail:
        _record: buffer the motion between the motion pointer and program pointer

    TODO: find a better name for the class since ActionQueue is misleading and "Action" is used in different contexts
        - ProgramExecutor, Runtime, ProgramRunner, ...
    """

    MOTION_LIMIT_IN = 10000  # maximal length of motion trajectory to be used for planning
    MOTION_LIMIT_OUT = inf  # maximal length of path history to be stored

    def __init__(self, execution_context: ExecutionContext):
        self._execution_context = execution_context
        self._stop_event = execution_context.stop_event
        # A dictionary of robot id with corresponding TCP name
        self._tcp: dict[str, str] = {}
        # Collected motion trajectory of the corresponding robot names
        self._record: dict[str, CombinedActions] = {}
        self._last_motions: dict[str, Motion] = {}
        self._on_motion_callbacks: dict[str, Any] = {}
        self._path_history: list[CombinedActions] = []

    def reset(self):
        self._tcp.clear()
        self._record = {}
        self._path_history = []
        self._on_motion_callbacks = {}

    def is_empty(self) -> bool:
        return not any(self._record.values())

    async def trigger_actions(
        self, motion_iter: AsyncIterable[MotionState], actions: list[ActionLocation]
    ) -> AsyncIterable[MotionState]:
        actions = sorted(actions, key=lambda action: action.path_parameter)
        async for motion_state in motion_iter:
            if self._stop_event.is_set():
                logger.info("Stop event set. Stopping motions...")
                break
            await self._run_callbacks(motion_state)

            last_action_index = 0
            for index, action_container in enumerate(actions):
                if action_container.path_parameter <= motion_state.path_parameter:
                    await self.run_action(action_container.action)
                    last_action_index = index + 1
                else:
                    break
            del actions[:last_action_index]
            yield motion_state

    async def _run(self):
        """The collected queue gets executed"""

        # get current collision setup
        # collision_scene = await self._execution_context.robot_cell.get_current_collision_scene()

        # plan & move
        planned_motions = {}
        for motion_group_id in self._record:  # pylint: disable=consider-using-dict-items
            container = self._record[motion_group_id]
            if len(container.motions) > 0:
                if self._execution_context.debug:
                    # This can raise MotionError
                    self._update_path_history(container)

                motion_group = self._execution_context.get_motion_group(motion_group_id)
                tcp = self._tcp.get(motion_group_id, None) or await motion_group.active_tcp_name()

                # TODO: not only pass motions, do we need the CombinedActions anymore?
                joint_trajectory = await motion_group.plan(
                    actions=container.motions,
                    tcp=tcp,
                    start_joint_position=None,
                    optimizer_setup=None,
                )
                motion_iter = motion_group.stream_execute(
                    joint_trajectory=joint_trajectory, tcp=tcp, actions=container.motions
                )
                planned_motions[motion_group_id] = self.trigger_actions(
                    motion_iter, container.actions.copy()
                )
                # When the motion trajectory is empty, execute the actions
                for action_container in container.actions:
                    await self.run_action(action_container.action)

        if planned_motions:
            combine = stream.merge(*planned_motions.values())

            # Append a empty list to the motion group recordings to start a new recording
            self._execution_context.motion_group_recordings.append([])

            async with combine.stream() as streamer:
                async for motion_state in streamer:
                    self._execution_context.motion_group_recordings[-1].append(motion_state)

        self._record.clear()
        self._tcp.clear()

    async def run_action(self, action: Action):
        return await run_action(action, self._execution_context)

    async def run(self, stop_event: anyio.Event):
        """Execute the collected motions and actions.
        This function is not reentrant.

        Args:
            stop_event: an event that can be set to stop the execution
        """

        async def stopper():
            assert isinstance(stop_event, anyio.Event)
            await stop_event.wait()
            await self._execution_context.robot_cell.stop()

        self._stop_event = stop_event
        await stoppable_run(self._run(), stopper())

    async def _run_callbacks(self, motion_state):
        for callback in self._on_motion_callbacks.values():
            await callback(None, motion_state.path_parameter, motion_state.state.pose)

    def last_pose(self, robot: str) -> Pose | tuple[float, ...] | None:
        """Return the last pose of the collected motion trajectory

        Args:
            robot: the robot to get the last pose from

        Returns:
            The last pose of the collected motion trajectory
        """
        if last_motion := self._last_motions.get(robot, None):
            return last_motion.target
        return None

    # TODO: should be combined with push method
    def attach_action(self, action: Action, motion_group_id: str):
        """Append a new action to the queue

        Args:
            action: the action to append
            motion_group_id: the robot to append the action to
        """
        if motion_group_id not in self._record:
            self._record[motion_group_id] = CombinedActions()
        self._record[motion_group_id].append(action)  # type: ignore

    def push(self, motions: Motion | tuple[Motion, ...], tool: str, motion_group_id: str):
        """Append a new motion to the queue.

        Args:
            motions: the motion command. If this is only a vector path, the orientation is automatically added
                based on the orientation strategy
            tool: the tool center point (TCP)
            motion_group_id: the robot to append the motion to

        Raises:
            MotionError: If motion is not possible o supported
        """
        assert isinstance(motion_group_id, str)
        if motion_group_id not in self._tcp:
            if tool is not None:
                self._tcp[motion_group_id] = tool
        else:
            if tool and self._tcp[motion_group_id] != tool:
                raise MotionError(
                    location=self._execution_context.location_in_code,
                    value=f"Changing the tcp in one motion is not supported: changed from {self._tcp[motion_group_id]} to {tool}",
                )

        if not isinstance(motions, tuple):
            motions = (motions,)

        for motion in motions:
            if motion_group_id not in self._record:
                self._record[motion_group_id] = CombinedActions()

            if len(self._record[motion_group_id]) >= self.MOTION_LIMIT_IN:
                raise MotionError(
                    location=self._execution_context.location_in_code,
                    value="Maximum motion queue size exceeded. Won't plan program.",
                )

            self._record[motion_group_id].append(motion)
            self._last_motions[motion_group_id] = motion

    def _update_path_history(self, path: CombinedActions):
        """Update the path history for debugging

        Args:
            path: the trajectory to append to the history

        Raises:
            MotionError: If the motion queue size exceeds the limit
        """
        self._path_history.append(path)
        if len(self._path_history) > self.MOTION_LIMIT_OUT:
            raise MotionError(
                location=self._execution_context.location_in_code,
                value="Maximum motion queue size exceeded. Won't plan program.",
            )


class PlannableActionQueue(ActionQueue):
    """ActionQueue intended to ignore all actions. Is used for the /plan/ endpoint."""

    MOTION_LIMIT_IN = 1000  # maximal length of motion trajectory to be used for planning
    MOTION_LIMIT_OUT = 1000  # maximal length of path history to be stored

    async def run_action(self, action: Action):
        # TODO: we only want to support reading from the run_args
        # in the story https://wandelbots.atlassian.net/browse/WOS-1924 the we will update the run_args to be
        # a record in Wandelscript. On the long run we will work on removing the support for the etcd database.
        if isinstance(action, ReadAction):
            device = self._execution_context.robot_cell[action.device_id]
            # allow reading from run_args
            if device.configuration.type in ("database",):
                return await run_action(action, self._execution_context)
        raise NotPlannableError(
            location=self._execution_context.location_in_code,
            value="Actions are not supported in the plan endpoint to avoid critical side effects.",
        )
