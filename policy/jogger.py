"""Position-controlled jogging for one or more motion groups.

Provides ``jog_joints()`` and ``jog_tcp()`` — async context managers that
open PID-controlled jogging sessions. The user sets target positions in a
loop; the PID controller continuously streams velocity commands to track
them.

Errors are detected automatically and raised through the ``async for`` loop:

- ``MotionError`` — joint limit or self-collision
- ``EmergencyStopError`` — e-stop, protective stop, safety violation
- ``GuardStopError`` — safety guard triggered
- ``RuntimeError`` — jogging connection lost
"""

from __future__ import annotations

import asyncio
import contextlib
import logging
from typing import TYPE_CHECKING, overload

from policy.estop import EstopMonitor, check_estop, check_sessions
from policy.pid_jogging_session import PidJoggingSession
from policy.types import PolicyRunnerConfig

if TYPE_CHECKING:
    from collections.abc import AsyncIterator

    from nova.cell.motion_group import MotionGroup
    from nova.types import Pose, RobotState
    from policy.types import SafetyGuard

logger = logging.getLogger(__name__)

_CARTESIAN_DIMS = 6


# ---------------------------------------------------------------------------
# Base jogger (shared lifecycle, error detection, state reading)
# ---------------------------------------------------------------------------


class _BaseJogger:
    """Shared logic for joint and TCP joggers."""

    def __init__(
        self,
        mg_list: list[MotionGroup],
        sessions: dict[MotionGroup, PidJoggingSession],
    ) -> None:
        self._mg_list = mg_list
        self._multi = len(mg_list) > 1
        self._sessions = sessions
        self._estop: EstopMonitor | None = None

    def _expected_dims(self, mg: MotionGroup) -> int | None:
        """Expected target dimension for a motion group. Override in subclass."""
        session = self._sessions.get(mg)
        return session._num_joints if session else None

    def _validate_and_push(self, mg: MotionGroup, values: list[float]) -> None:
        """Validate target dimensions and push to session."""
        expected = self._expected_dims(mg)
        if expected is not None and len(values) != expected:
            msg = (
                f"Target has {len(values)} values but motion group "
                f"'{mg.id}' expects {expected}"
            )
            raise ValueError(msg)
        session = self._sessions.get(mg)
        if session is not None:
            session.update_chunk(steps=[values], dt_ms=0.0)

    def state(self) -> dict[MotionGroup, RobotState] | RobotState | None:
        """Get current robot state(s).

        Returns a single ``RobotState`` for single-MG joggers,
        or ``dict[MotionGroup, RobotState]`` for multi-MG.
        """
        if not self._multi:
            return self._sessions[self._mg_list[0]].current_state
        result: dict[MotionGroup, RobotState] = {}
        for mg, session in self._sessions.items():
            s = session.current_state
            if s is not None:
                result[mg] = s
        return result if result else None

    async def __aenter__(self) -> _BaseJogger:
        for session in self._sessions.values():
            await session.start()
        self._estop = EstopMonitor(self._mg_list)
        await self._estop.start()
        kind = self.__class__.__name__
        logger.info(
            "%s started (%d motion group%s)",
            kind,
            len(self._sessions),
            "s" if len(self._sessions) > 1 else "",
        )
        return self

    async def __aexit__(
        self,
        exc_type: type[BaseException] | None,
        exc_val: BaseException | None,
        exc_tb: object,
    ) -> bool:
        if self._estop is not None:
            await self._estop.stop()
            self._estop = None
        for session in self._sessions.values():
            with contextlib.suppress(Exception):
                await session.stop()
        logger.info("%s stopped", self.__class__.__name__)
        return False

    async def __aiter__(self) -> AsyncIterator[dict[MotionGroup, RobotState] | RobotState]:
        """Yield current state at ~100Hz. Raises on errors. Use ``break`` to stop."""
        while True:
            check_sessions(self._sessions)
            check_estop(self._estop)
            s = self.state()
            if s is not None:
                yield s
            await asyncio.sleep(0.01)


# ---------------------------------------------------------------------------
# Joint jogger
# ---------------------------------------------------------------------------


class JointJogger(_BaseJogger):
    """PID-controlled joint position jogger.

    Do not instantiate directly — use :func:`jog_joints`.
    """

    def __init__(
        self,
        motion_groups: list[MotionGroup],
        *,
        config: PolicyRunnerConfig | None = None,
        safety_guards: list[SafetyGuard] | None = None,
    ) -> None:
        cfg = config or PolicyRunnerConfig()
        sessions: dict[MotionGroup, PidJoggingSession] = {}
        for mg in motion_groups:
            sessions[mg] = PidJoggingSession(
                motion_group=mg, config=cfg, safety_guards=safety_guards,
            )
        super().__init__(motion_groups, sessions)
        self._target: dict[MotionGroup, list[float]] | None = None

    @property
    def target(self) -> dict[MotionGroup, list[float]] | list[float] | None:
        """Current target. Set to update the tracking target."""
        if self._target is None:
            return None
        if not self._multi:
            return self._target.get(self._mg_list[0])
        return self._target

    @target.setter
    def target(self, value: dict[MotionGroup, list[float]] | list[float] | None) -> None:
        if value is None:
            self._target = None
            return
        if isinstance(value, list):
            if self._multi:
                msg = "For multiple motion groups, pass a dict[MotionGroup, list[float]]"
                raise TypeError(msg)
            mg = self._mg_list[0]
            self._validate_and_push(mg, value)
            self._target = {mg: value}
        elif isinstance(value, dict):
            for mg, joints in value.items():
                self._validate_and_push(mg, joints)
            self._target = value
        else:
            msg = f"Expected list[float] or dict[MotionGroup, list[float]], got {type(value)}"
            raise TypeError(msg)

    async def __aenter__(self) -> JointJogger:
        await super().__aenter__()
        return self


# ---------------------------------------------------------------------------
# TCP jogger
# ---------------------------------------------------------------------------


class TcpJogger(_BaseJogger):
    """PID-controlled TCP pose jogger.

    Do not instantiate directly — use :func:`jog_tcp`.
    """

    def __init__(
        self,
        motion_groups: dict[MotionGroup, str],
        *,
        config: PolicyRunnerConfig | None = None,
        safety_guards: list[SafetyGuard] | None = None,
    ) -> None:
        cfg = config or PolicyRunnerConfig()
        sessions: dict[MotionGroup, PidJoggingSession] = {}
        for mg, tcp in motion_groups.items():
            sessions[mg] = PidJoggingSession(
                motion_group=mg, config=cfg, tcp=tcp, mode="cartesian",
                safety_guards=safety_guards,
            )
        mg_list = list(motion_groups.keys())
        super().__init__(mg_list, sessions)
        self._target: dict[MotionGroup, Pose] | None = None

    def _expected_dims(self, mg: MotionGroup) -> int | None:  # noqa: ARG002
        return _CARTESIAN_DIMS

    @property
    def target(self) -> dict[MotionGroup, Pose] | Pose | None:
        """Current target. Set to update the tracking target."""
        if self._target is None:
            return None
        if not self._multi:
            return self._target.get(self._mg_list[0])
        return self._target

    @target.setter
    def target(self, value: dict[MotionGroup, Pose] | Pose | None) -> None:
        if value is None:
            self._target = None
            return

        from nova.types import Pose  # noqa: PLC0415

        if isinstance(value, Pose):
            if self._multi:
                msg = "For multiple motion groups, pass a dict[MotionGroup, Pose]"
                raise TypeError(msg)
            mg = self._mg_list[0]
            self._validate_and_push(mg, list(value.position) + list(value.orientation))
            self._target = {mg: value}
        elif isinstance(value, dict):
            for mg, pose in value.items():
                self._validate_and_push(mg, list(pose.position) + list(pose.orientation))
            self._target = value
        else:
            msg = f"Expected Pose or dict[MotionGroup, Pose], got {type(value)}"
            raise TypeError(msg)

    async def __aenter__(self) -> TcpJogger:
        await super().__aenter__()
        return self


# ---------------------------------------------------------------------------
# Public constructors
# ---------------------------------------------------------------------------


@overload
def jog_joints(
    motion_groups: MotionGroup,
    *,
    config: PolicyRunnerConfig | None = ...,
    safety_guards: list[SafetyGuard] | None = ...,
) -> JointJogger: ...


@overload
def jog_joints(
    motion_groups: list[MotionGroup],
    *,
    config: PolicyRunnerConfig | None = ...,
    safety_guards: list[SafetyGuard] | None = ...,
) -> JointJogger: ...


def jog_joints(
    motion_groups: MotionGroup | list[MotionGroup],
    *,
    config: PolicyRunnerConfig | None = None,
    safety_guards: list[SafetyGuard] | None = None,
) -> JointJogger:
    """Create a PID-controlled joint position jogger.

    Args:
        motion_groups: Single motion group or list for multi-robot control.
        config: PID controller configuration. Uses training-time defaults if None.
        safety_guards: Optional callbacks run on every PID tick.
            Each receives a ``GuardState`` and must return ``True`` to continue.

    Returns:
        A :class:`JointJogger` async context manager.

    Raises:
        MotionError: Joint limit or self-collision detected.
        EmergencyStopError: E-stop or protective stop.
        GuardStopError: A safety guard triggered.
        RuntimeError: Jogging connection lost.

    Example::

        async with jog_joints(mg) as jogger:
            async for state in jogger:
                jogger.target = [0.1, -1.5, 1.0, -0.5, 0.0, 0.0]
    """
    if not isinstance(motion_groups, list):
        motion_groups = [motion_groups]
    return JointJogger(motion_groups, config=config, safety_guards=safety_guards)


@overload
def jog_tcp(
    motion_groups: MotionGroup,
    *,
    tcp: str,
    config: PolicyRunnerConfig | None = ...,
    safety_guards: list[SafetyGuard] | None = ...,
) -> TcpJogger: ...


@overload
def jog_tcp(
    motion_groups: dict[MotionGroup, str],
    *,
    config: PolicyRunnerConfig | None = ...,
    safety_guards: list[SafetyGuard] | None = ...,
) -> TcpJogger: ...


def jog_tcp(
    motion_groups: MotionGroup | dict[MotionGroup, str],
    *,
    tcp: str = "",
    config: PolicyRunnerConfig | None = None,
    safety_guards: list[SafetyGuard] | None = None,
) -> TcpJogger:
    """Create a PID-controlled TCP pose jogger.

    Args:
        motion_groups: Single motion group (with ``tcp`` kwarg) or
            ``dict[MotionGroup, str]`` mapping each group to its TCP name.
        tcp: TCP name when passing a single motion group.
        config: PID controller configuration. Uses cartesian defaults if None.
        safety_guards: Optional callbacks run on every PID tick.

    Returns:
        A :class:`TcpJogger` async context manager.

    Raises:
        MotionError: Joint limit or self-collision detected.
        EmergencyStopError: E-stop or protective stop.
        GuardStopError: A safety guard triggered.
        RuntimeError: Jogging connection lost.

    Example::

        async with jog_tcp(mg, tcp="Flange") as jogger:
            async for state in jogger:
                jogger.target = Pose(500, 200, 300, 0, 3.14, 0)
    """
    if isinstance(motion_groups, dict):
        return TcpJogger(motion_groups, config=config, safety_guards=safety_guards)
    return TcpJogger({motion_groups: tcp}, config=config, safety_guards=safety_guards)
