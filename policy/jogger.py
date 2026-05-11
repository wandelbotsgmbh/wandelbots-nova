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
from policy.pidjogging import PidJoggingSession
from policy.types import PidConfig

if TYPE_CHECKING:
    from collections.abc import AsyncIterator

    from nova.cell.motion_group import MotionGroup
    from nova.types import Pose, RobotState
    from policy.types import SafetyGuard

logger = logging.getLogger(__name__)

_CARTESIAN_DIMS = 6  # x, y, z, rx, ry, rz — fixed by NOVA jogging API


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
            with contextlib.suppress(asyncio.CancelledError, OSError, RuntimeError):
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
        config: PidConfig | None = None,
        safety_guards: list[SafetyGuard] | None = None,
    ) -> None:
        cfg = config or PidConfig()
        sessions: dict[MotionGroup, PidJoggingSession] = {}
        for mg in motion_groups:
            sessions[mg] = PidJoggingSession(
                motion_group=mg, config=cfg, safety_guards=safety_guards,
            )
        super().__init__(motion_groups, sessions)
        self._target: dict[MotionGroup, list[float]] | None = None

    @property
    def target(self) -> dict[MotionGroup, list[float]] | list[float] | None:
        """Current target (read-only). Use :meth:`set_target` to update."""
        if self._target is None:
            return None
        if not self._multi:
            return self._target.get(self._mg_list[0])
        return self._target

    def _push_target(self, mg: MotionGroup, value: list, dt_ms: float) -> list[float]:
        """Push a single or chunk target to a session. Returns the final target."""
        session = self._sessions.get(mg)
        if session is None:
            return value if not isinstance(value[0], list) else value[-1]
        if value and isinstance(value[0], list):
            session.update_chunk(steps=value, dt_ms=dt_ms)
            return value[-1]
        self._validate_and_push(mg, value)
        return value

    def set_target(
        self,
        target: (
            list[float]
            | list[list[float]]
            | dict[MotionGroup, list[float] | list[list[float]]]
        ),
        *,
        dt_ms: float = 0.0,
    ) -> None:
        """Set the tracking target.

        Args:
            target: Joint positions to track.
                - ``list[float]`` — single target (one motion group)
                - ``list[list[float]]`` — chunk of future targets (one motion group)
                - ``dict[MotionGroup, ...]`` — per-MG targets or chunks
            dt_ms: Time between chunk steps in milliseconds.
                Only used when ``target`` contains chunks (nested lists).
                Enables interpolation and feedforward velocity.
        """
        if isinstance(target, list):
            if self._multi:
                msg = "For multiple motion groups, pass a dict[MotionGroup, ...]"
                raise TypeError(msg)
            mg = self._mg_list[0]
            final = self._push_target(mg, target, dt_ms)
            self._target = {mg: final}
        elif isinstance(target, dict):
            self._target = self._target or {}
            for mg, mg_value in target.items():
                self._target[mg] = self._push_target(mg, mg_value, dt_ms)
        else:
            msg = f"Expected list or dict, got {type(target)}"
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
        config: PidConfig | None = None,
        tcp_velocity_limit: float = 250.0,
        tcp_orientation_velocity_limit: float = 1.5,
        safety_guards: list[SafetyGuard] | None = None,
    ) -> None:
        cfg = config or PidConfig()
        # Build per-axis velocity limits for cartesian mode: [tx, ty, tz, rx, ry, rz]
        tcp_limits = [
            tcp_velocity_limit, tcp_velocity_limit, tcp_velocity_limit,
            tcp_orientation_velocity_limit, tcp_orientation_velocity_limit,
            tcp_orientation_velocity_limit,
        ]
        tcp_cfg = PidConfig(
            velocity_limit=tcp_limits,
            tolerance=cfg.tolerance,
            p_gain=cfg.p_gain,
            i_gain=cfg.i_gain,
            d_gain=cfg.d_gain,
            ff_gain=cfg.ff_gain,
            integral_limit=cfg.integral_limit,
            state_rate_ms=cfg.state_rate_ms,
        )
        sessions: dict[MotionGroup, PidJoggingSession] = {}
        for mg, tcp in motion_groups.items():
            sessions[mg] = PidJoggingSession(
                motion_group=mg, config=tcp_cfg, tcp=tcp, mode="cartesian",
                safety_guards=safety_guards,
            )
        mg_list = list(motion_groups.keys())
        super().__init__(mg_list, sessions)
        self._target: dict[MotionGroup, Pose] | None = None

    def _expected_dims(self, mg: MotionGroup) -> int | None:  # noqa: ARG002
        return _CARTESIAN_DIMS

    @property
    def target(self) -> dict[MotionGroup, Pose] | Pose | None:
        """Current target (read-only). Use :meth:`set_target` to update."""
        if self._target is None:
            return None
        if not self._multi:
            return self._target.get(self._mg_list[0])
        return self._target

    def set_target(self, target: Pose | dict[MotionGroup, Pose]) -> None:
        """Set the TCP tracking target.

        Args:
            target: TCP pose to track.
                - ``Pose`` — single motion group
                - ``dict[MotionGroup, Pose]`` — per-MG targets
        """
        from nova.types import Pose  # noqa: PLC0415

        if isinstance(target, Pose):
            if self._multi:
                msg = "For multiple motion groups, pass a dict[MotionGroup, Pose]"
                raise TypeError(msg)
            mg = self._mg_list[0]
            self._validate_and_push(mg, list(target.position) + list(target.orientation))
            self._target = {mg: target}
        elif isinstance(target, dict):
            for mg, pose in target.items():
                self._validate_and_push(mg, list(pose.position) + list(pose.orientation))
            self._target = target
        else:
            msg = f"Expected Pose or dict[MotionGroup, Pose], got {type(target)}"
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
    config: PidConfig | None = ...,
    safety_guards: list[SafetyGuard] | None = ...,
) -> JointJogger: ...


@overload
def jog_joints(
    motion_groups: list[MotionGroup],
    *,
    config: PidConfig | None = ...,
    safety_guards: list[SafetyGuard] | None = ...,
) -> JointJogger: ...


def jog_joints(
    motion_groups: MotionGroup | list[MotionGroup],
    *,
    config: PidConfig | None = None,
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
                jogger.set_target([0.1, -1.5, 1.0, -0.5, 0.0, 0.0])
    """
    if not isinstance(motion_groups, list):
        motion_groups = [motion_groups]
    return JointJogger(motion_groups, config=config, safety_guards=safety_guards)


@overload
def jog_tcp(
    motion_groups: MotionGroup,
    *,
    tcp: str,
    config: PidConfig | None = ...,
    tcp_velocity_limit: float = ...,
    tcp_orientation_velocity_limit: float = ...,
    safety_guards: list[SafetyGuard] | None = ...,
) -> TcpJogger: ...


@overload
def jog_tcp(
    motion_groups: dict[MotionGroup, str],
    *,
    config: PidConfig | None = ...,
    tcp_velocity_limit: float = ...,
    tcp_orientation_velocity_limit: float = ...,
    safety_guards: list[SafetyGuard] | None = ...,
) -> TcpJogger: ...


def jog_tcp(
    motion_groups: MotionGroup | dict[MotionGroup, str],
    *,
    tcp: str = "",
    config: PidConfig | None = None,
    tcp_velocity_limit: float = 250.0,
    tcp_orientation_velocity_limit: float = 1.5,
    safety_guards: list[SafetyGuard] | None = None,
) -> TcpJogger:
    """Create a PID-controlled TCP pose jogger.

    Args:
        motion_groups: Single motion group (with ``tcp`` kwarg) or
            ``dict[MotionGroup, str]`` mapping each group to its TCP name.
        tcp: TCP name when passing a single motion group.
        config: PID controller configuration.
        tcp_velocity_limit: TCP translation velocity limit in mm/s.
        tcp_orientation_velocity_limit: TCP rotation velocity limit in rad/s.
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
                jogger.set_target(Pose(500, 200, 300, 0, 3.14, 0))
    """
    if isinstance(motion_groups, dict):
        return TcpJogger(
            motion_groups, config=config,
            tcp_velocity_limit=tcp_velocity_limit,
            tcp_orientation_velocity_limit=tcp_orientation_velocity_limit,
            safety_guards=safety_guards,
        )
    return TcpJogger(
        {motion_groups: tcp}, config=config,
        tcp_velocity_limit=tcp_velocity_limit,
        tcp_orientation_velocity_limit=tcp_orientation_velocity_limit,
        safety_guards=safety_guards,
    )
