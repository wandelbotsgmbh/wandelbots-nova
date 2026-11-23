"""Position-controlled jogging for one or more motion groups: ``jog_joints()`` and ``jog_tcp()``.

Faults raise from the ``async for`` loop: ``MotionError`` (joint limit, self-collision),
``EmergencyStopError`` (e-stop, protective stop), ``RuntimeError`` (connection lost). A fired stop
condition ends the loop normally; its name is on ``jogger.stop_condition_triggered``.
"""

from __future__ import annotations

import asyncio
import logging
import time
from typing import TYPE_CHECKING, Self, TypeAlias, cast, overload

from novapolicy.estop import EstopMonitor
from novapolicy.jogging.clock import ServerMs
from novapolicy.jogging.lifecycle import SessionLifecycle, move_to_start_joint_position
from novapolicy.jogging.live_buffer import (
    LIVE_LEAD_MS,
    trajectory_elapsed_ms,
)
from novapolicy.jogging.waypoint_session import WaypointJoggingSession
from novapolicy.types import WaypointConfig

if TYPE_CHECKING:
    from collections.abc import AsyncIterator, Sequence

    from nova.cell.motion_group import MotionGroup
    from nova.types import Pose, RobotState
    from novapolicy.rerun import PolicyRerunLogger
    from novapolicy.types import JoggingMode, StopCondition

    _Target: TypeAlias = list[float] | Pose
    """A live target: joint positions, or a TCP pose."""

logger = logging.getLogger(__name__)

_CARTESIAN_DIMS = 6  # x, y, z, rx, ry, rz — fixed by the NOVA waypoint pose

_Chunk: TypeAlias = list[list[float]]


def _is_chunk(value: object) -> bool:
    """Whether ``value`` is a chunk of future steps (a non-empty list of lists), not one target."""
    return isinstance(value, list) and bool(value) and isinstance(value[0], list)


# How far ahead of session time step zero is placed, for a chunk and a live target alike [ms];
# must outlast the interval between pushes plus link latency, and stays constant (ADR 0013).
LEAD_MS = LIVE_LEAD_MS


# ---------------------------------------------------------------------------
# Base jogger (shared lifecycle, error detection, state reading)
# ---------------------------------------------------------------------------


class _BaseJogger:
    """Shared logic for joint and TCP joggers: one session per motion group, live targets and
    chunks. A subclass names its mode and how a target becomes waypoint values."""

    _MODE: JoggingMode = "joint"
    _TARGET = "list[float]"

    def __init__(
        self,
        motion_groups: dict[MotionGroup, str],
        *,
        config: WaypointConfig | None = None,
        stop_conditions: list[StopCondition] | None = None,
        start_joint_position: list[float] | dict[MotionGroup, list[float]] | None = None,
        pipeline: Sequence[object] | None = None,
    ) -> None:
        cfg = config or WaypointConfig()
        self._mg_list = list(motion_groups)
        self._multi = len(self._mg_list) > 1
        self._sessions = {
            mg: WaypointJoggingSession(
                motion_group=mg,
                config=cfg,
                tcp=tcp,
                mode=self._MODE,
                stop_conditions=stop_conditions,
                pipeline=pipeline,
            )
            for mg, tcp in motion_groups.items()
        }
        if start_joint_position is not None and not isinstance(start_joint_position, dict):
            start_joint_position = {self._mg_list[0]: start_joint_position}
        self._start_joint_position = start_joint_position
        self._target: dict[MotionGroup, _Target] | None = None
        self._lifecycle = SessionLifecycle(
            {mg.id: session for mg, session in self._sessions.items()}, self._mg_list
        )
        self._rerun: PolicyRerunLogger | None = None
        self._loop_t0: float | None = None
        self._ack0_ms: float = 0.0
        self._tick_ms: float | None = None
        self._timeline_ms: float = 0.0

    @property
    def elapsed(self) -> float:
        """Seconds of acknowledged motion since the loop first had state; ``0.0`` before.

        On the session clock, sampled once per loop iteration, quantised and never backwards
        (:func:`~novapolicy.jogging.live_buffer.trajectory_elapsed_ms`).
        """
        if self._loop_t0 is None:
            return 0.0
        now_ms = self._tick_ms if self._tick_ms is not None else self._acknowledged_ms()
        self._timeline_ms = trajectory_elapsed_ms(now_ms, self._ack0_ms, self._timeline_ms)
        return self._timeline_ms / 1000.0

    def _acknowledged_ms(self) -> float:
        """Acknowledged session "now" [ms], the minimum across arms: the slowest sets the pace."""
        return min(s.estimated_server_timestamp_ms for s in self._sessions.values())

    def _anchor(self, mg: MotionGroup) -> ServerMs | None:
        """Session-clock time of content produced at this iteration, ``ack0 + elapsed + LEAD_MS``,
        the same however often it is sent; ``None`` before the loop has state (place relative
        to "now" then)."""
        if mg not in self._sessions or self._loop_t0 is None:
            return None
        return ServerMs(int(self._ack0_ms + self.elapsed * 1000.0 + LEAD_MS))

    def _expected_dims(self, mg: MotionGroup) -> int | None:
        """Expected target dimension for a motion group. Override in subclass."""
        session = self._sessions.get(mg)
        return session.num_joints if session else None

    def _set_live_target(self, mg: MotionGroup, values: list[float]) -> None:
        """Validate one live target and send it as one waypoint, exactly as measured."""
        expected = self._expected_dims(mg)
        if expected is not None and len(values) != expected:
            msg = f"Target has {len(values)} values but motion group '{mg.id}' expects {expected}"
            raise ValueError(msg)
        session = self._sessions.get(mg)
        if session is not None:
            steps = [list(values)]
            anchor = self._anchor(mg)
            # When the target was produced, on the loop timeline all arms share. Each term is
            # rounded on its own: truncating the sum puts some targets a whole grid step early.
            produced_ms = (
                ServerMs(round(self._ack0_ms) + round(self.elapsed * 1000.0))
                if self._loop_t0 is not None
                else None
            )
            session.update_chunk(
                steps=steps,
                dt_ms=0.0,
                first_timestamp_ms=anchor,
                timestamp_offset_steps=0 if anchor is not None else 1,
                observed_at_ms=produced_ms,
                extend_buffer=False,
            )
        self._log_target(mg.id, [list(values)], 0.0)

    def _send_chunk(self, mg: MotionGroup, chunk: _Chunk, dt_ms: float) -> list[float]:
        """Send one motion group's chunk at :meth:`_anchor`; returns its final step."""
        session = self._sessions.get(mg)
        if session is not None:
            anchor = self._anchor(mg)
            session.update_chunk(
                steps=chunk,
                dt_ms=dt_ms,
                first_timestamp_ms=(
                    anchor if anchor is not None else session.estimated_server_timestamp_ms
                ),
                extend_buffer=True,  # HoldPadding pads chunks, never live targets
            )
            self._log_target(mg.id, chunk, dt_ms)
        return chunk[-1]

    def _dispatch_chunk(
        self, chunk: _Chunk | dict[MotionGroup, _Chunk], dt_ms: float
    ) -> dict[MotionGroup, list[float]]:
        """Validate and send a chunk per motion group; return each one's final step."""
        if dt_ms <= 0:
            msg = (
                "set_chunk needs a positive dt_ms — the spacing between the "
                f"chunk's steps — but got {dt_ms!r}."
            )
            raise ValueError(msg)

        if isinstance(chunk, dict):
            finals: dict[MotionGroup, list[float]] = {}
            for mg, steps in chunk.items():
                self._require_chunk(steps)
                finals[mg] = self._send_chunk(mg, steps, dt_ms)
            return finals

        if not isinstance(chunk, list):
            msg = f"Expected list[list[float]] or dict[MotionGroup, ...], got {type(chunk)}"
            raise TypeError(msg)
        self._require_chunk(chunk)
        if self._multi:
            msg = "For multiple motion groups, pass a dict[MotionGroup, list[list[float]]]"
            raise TypeError(msg)
        mg = self._mg_list[0]
        final = self._send_chunk(mg, chunk, dt_ms)
        return {mg: final}

    @staticmethod
    def _require_chunk(value: object) -> None:
        """Reject anything but a chunk, on the way into ``set_chunk``."""
        if _is_chunk(value):
            return
        if isinstance(value, list) and not value:
            msg = "set_chunk got an empty list; a chunk needs at least one step."
        elif isinstance(value, list):
            msg = (
                "set_chunk expects a chunk of future steps (list[list[float]]) but "
                "got a single target. Use set_target(...) for one target."
            )
        else:
            msg = f"set_chunk expects list[list[float]], got {type(value)}"
        raise TypeError(msg)

    @staticmethod
    def _reject_chunk(value: object) -> None:
        """Reject a chunk, on the way into ``set_target``."""
        if not _is_chunk(value):
            return
        msg = (
            "set_target takes one live target, not a chunk of future steps. Pass "
            "it to set_chunk(chunk, dt_ms=...) instead — a chunk carries its own "
            "horizon, so a LiveBuffer does not apply to it."
        )
        raise TypeError(msg)

    def _log_target(self, mg_id: str, steps: list[list[float]], dt_ms: float) -> None:
        """Log jogging target to Rerun as an action chunk; the tracking error is logged per
        state packet (see :meth:`PolicyRerunLogger.log_state_packet`)."""
        if self._rerun is None:
            return
        from novapolicy.types import ActionChunk  # ruff: ignore[import-outside-top-level]

        if self._MODE == "cartesian":
            chunk = ActionChunk(tcp={mg_id: steps}, dt_ms=dt_ms)
        else:
            chunk = ActionChunk(joints={mg_id: steps}, dt_ms=dt_ms)
        self._rerun.log_action_chunk(chunk, step=0)

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

    async def __aenter__(self) -> Self:
        await move_to_start_joint_position(self._start_joint_position)
        await self._lifecycle.start_sessions()
        self._rerun = await self._lifecycle.start_monitoring(EstopMonitor(self._mg_list))
        await self._lifecycle.wait_ready()
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
        # A fault, an e-stop or a fired stop condition wants the robot stopped now.
        await self._lifecycle.close(
            drain=exc_type is None and self.stop_condition_triggered is None
        )
        self._rerun = None
        logger.info("%s stopped", self.__class__.__name__)
        return False

    @property
    def stop_condition_triggered(self) -> str | None:
        """Name of the stop condition that ended the loop (normally, without an exception), or
        ``None``."""
        return self._lifecycle.stop_condition

    async def __aiter__(
        self,
    ) -> AsyncIterator[dict[MotionGroup, RobotState] | RobotState]:
        """Yield current state at ~100Hz. Raises on faults; a stop condition ends
        the loop normally (see :attr:`stop_condition_triggered`). Use ``break`` to stop.
        """
        while self._lifecycle.check() is None:
            s = self.state()
            if s is None:
                await asyncio.sleep(0.01)
                continue
            if self._loop_t0 is None:
                # Anchor at the first state, not at RUNNING (jogging.md, "Timing targets"), and
                # baseline the acknowledged clock there, so elapsed starts at zero.
                self._loop_t0 = time.monotonic()
                self._ack0_ms = self._acknowledged_ms()
            # One timeline sample per iteration, shared by everything the caller does with it.
            self._tick_ms = self._acknowledged_ms()
            yield s
            await asyncio.sleep(0.01)

    @property
    def target(self) -> _Target | dict[MotionGroup, _Target] | None:
        """Current target (read-only): one value for one motion group, else a dict. Use
        :meth:`set_target` or :meth:`set_chunk` to update."""
        if self._target is None:
            return None
        return self._target if self._multi else self._target.get(self._mg_list[0])

    def _values(self, target: object) -> list[float]:
        """A live target as waypoint values; ``TypeError`` for anything else."""
        if not isinstance(target, list):
            msg = f"Expected {self._TARGET} per motion group, got {type(target)}"
            raise TypeError(msg)
        return target

    def set_target(self, target: _Target | dict[MotionGroup, _Target]) -> None:
        """Set the live target — where the robot should be *now*: joint positions for
        :func:`jog_joints`, a ``Pose`` for :func:`jog_tcp`; a dict per motion group for several.

        Each target goes out as one waypoint; a ``LiveBuffer`` in the pipeline replays recent
        targets as a horizon instead.

        Raises:
            TypeError: If given a chunk of future steps (use :meth:`set_chunk`), a value of the
                wrong kind, or one value for several motion groups.
        """
        if isinstance(target, dict):
            targets = target
        else:
            self._reject_chunk(target)
            try:
                self._values(target)
            except TypeError:
                msg = f"Expected {self._TARGET} or dict[MotionGroup, {self._TARGET}], got {type(target)}"
                raise TypeError(msg) from None
            if self._multi:
                msg = f"For multiple motion groups, pass a dict[MotionGroup, {self._TARGET}]"
                raise TypeError(msg)
            targets = {self._mg_list[0]: target}
        for mg, value in targets.items():
            self._reject_chunk(value)
            self._set_live_target(mg, self._values(value))
        self._target = {**(self._target or {}), **targets} if isinstance(target, dict) else targets

    def set_chunk(
        self,
        chunk: list[list[float]] | dict[MotionGroup, list[list[float]]],
        *,
        dt_ms: float,
    ) -> None:
        """Send a chunk of future targets: joint positions, or TCP poses as raw
        ``[x, y, z, rx, ry, rz]`` in mm and rad; a dict per motion group for several.

        A ``LiveBuffer`` passes a chunk unchanged and clears its window; ``HoldPadding`` pads a
        short one, and ``DropLate`` drops steps whose moment has passed.

        Args:
            chunk: The future steps.
            dt_ms: Spacing between consecutive steps, in milliseconds. Required.

        Raises:
            TypeError: If given a single target. Use :meth:`set_target`.
            ValueError: If ``dt_ms`` is not positive.
        """
        finals = self._dispatch_chunk(chunk, dt_ms)
        # A dict updates only the groups it names; a bare chunk replaces outright.
        kept: dict[MotionGroup, _Target] = (
            dict(self._target or {}) if isinstance(chunk, dict) else {}
        )
        self._target = {**kept, **finals}


class JointJogger(_BaseJogger):
    """Joint position jogger. Do not instantiate directly — use :func:`jog_joints`."""

    def __init__(
        self,
        motion_groups: list[MotionGroup],
        *,
        config: WaypointConfig | None = None,
        stop_conditions: list[StopCondition] | None = None,
        start_joint_position: list[float] | dict[MotionGroup, list[float]] | None = None,
        pipeline: Sequence[object] | None = None,
    ) -> None:
        super().__init__(
            dict.fromkeys(motion_groups, ""),
            config=config,
            stop_conditions=stop_conditions,
            start_joint_position=start_joint_position,
            pipeline=pipeline,
        )


class TcpJogger(_BaseJogger):
    """TCP pose jogger. Do not instantiate directly — use :func:`jog_tcp`."""

    _MODE: JoggingMode = "cartesian"
    _TARGET = "Pose"

    def _expected_dims(self, mg: MotionGroup) -> int | None:  # ruff: ignore[unused-method-argument, no-self-use]
        return _CARTESIAN_DIMS

    def _values(self, target: object) -> list[float]:
        from nova.types import Pose  # ruff: ignore[import-outside-top-level]

        if not isinstance(target, Pose):
            self._reject_chunk(target)
            msg = f"Expected Pose per motion group, got {type(target)}"
            raise TypeError(msg)
        return [*target.position, *target.orientation]


# ---------------------------------------------------------------------------
# Public constructors
# ---------------------------------------------------------------------------


@overload
def jog_joints(
    motion_groups: MotionGroup,
    *,
    config: WaypointConfig | None = ...,
    stop_conditions: list[StopCondition] | None = ...,
    start_joint_position: list[float] | None = ...,
    pipeline: Sequence[object] | None = ...,
) -> JointJogger:
    pass


@overload
def jog_joints(
    motion_groups: list[MotionGroup],
    *,
    config: WaypointConfig | None = ...,
    stop_conditions: list[StopCondition] | None = ...,
    start_joint_position: dict[MotionGroup, list[float]] | None = ...,
    pipeline: Sequence[object] | None = ...,
) -> JointJogger:
    pass


def jog_joints(
    motion_groups: MotionGroup | list[MotionGroup],
    *,
    config: WaypointConfig | None = None,
    stop_conditions: list[StopCondition] | None = None,
    start_joint_position: list[float] | dict[MotionGroup, list[float]] | None = None,
    pipeline: Sequence[object] | None = None,
) -> JointJogger:
    """Create a joint position jogger using server-side waypoint jogging.

    Args:
        motion_groups: Single motion group or list for multi-robot control.
        config: Waypoint jogging configuration.
        stop_conditions: Callbacks run on every jogging tick; one returning ``True`` for its
            ``StopContext`` ends the loop.
        start_joint_position: Joint positions to PTP-move to first: a list for one robot, a dict
            per motion group for several.
        pipeline: The modules between the targets and NOVA, in order, exactly as given;
            ``None`` runs :func:`~novapolicy.jogging.modules.default_pipeline`. Extend that list
            to add one, e.g. ``[LiveBuffer(window_ms=500), *default_pipeline()]``.

    Returns:
        A :class:`JointJogger` async context manager.

    Raises:
        MotionError: Joint limit or self-collision detected.
        EmergencyStopError: E-stop or protective stop.
        RuntimeError: Jogging connection lost.

    Example::

        async with jog_joints(mg, start_joint_position=[0, -1.57, 1.57, -1.57, -1.57, 0]) as jogger:
            async for state in jogger:
                jogger.set_target([0.1, -1.5, 1.0, -0.5, 0.0, 0.0])
    """
    groups = (
        cast("list[MotionGroup]", motion_groups)
        if isinstance(motion_groups, list)
        else [motion_groups]
    )
    return JointJogger(
        groups,
        config=config,
        stop_conditions=stop_conditions,
        start_joint_position=start_joint_position,
        pipeline=pipeline,
    )


@overload
def jog_tcp(
    motion_groups: MotionGroup,
    *,
    tcp: str,
    config: WaypointConfig | None = ...,
    stop_conditions: list[StopCondition] | None = ...,
    start_joint_position: list[float] | None = ...,
    pipeline: Sequence[object] | None = ...,
) -> TcpJogger:
    pass


@overload
def jog_tcp(
    motion_groups: dict[MotionGroup, str],
    *,
    config: WaypointConfig | None = ...,
    stop_conditions: list[StopCondition] | None = ...,
    start_joint_position: dict[MotionGroup, list[float]] | None = ...,
    pipeline: Sequence[object] | None = ...,
) -> TcpJogger:
    pass


def jog_tcp(
    motion_groups: MotionGroup | dict[MotionGroup, str],
    *,
    tcp: str = "",
    config: WaypointConfig | None = None,
    stop_conditions: list[StopCondition] | None = None,
    start_joint_position: list[float] | dict[MotionGroup, list[float]] | None = None,
    pipeline: Sequence[object] | None = None,
) -> TcpJogger:
    """Create a TCP pose jogger using server-side waypoint jogging.

    Args:
        motion_groups: Single motion group (with ``tcp`` kwarg) or
            ``dict[MotionGroup, str]`` mapping each group to its TCP name.
        tcp: TCP name when passing a single motion group.
        config: As for :func:`jog_joints`.
        stop_conditions: As for :func:`jog_joints`.
        start_joint_position: As for :func:`jog_joints`.
        pipeline: As for :func:`jog_joints`.

    Returns:
        A :class:`TcpJogger` async context manager.

    Raises:
        MotionError: Joint limit or self-collision detected.
        EmergencyStopError: E-stop or protective stop.
        RuntimeError: Jogging connection lost.

    Example::

        async with jog_tcp(mg, tcp="Flange", start_joint_position=[1.17, -0.73, 1.75, -3.05, 0.87, 2.09]) as jogger:
            async for state in jogger:
                jogger.set_target(Pose(500, 200, 300, 0, 3.14, 0))
    """
    if isinstance(motion_groups, dict):
        return TcpJogger(
            cast("dict[MotionGroup, str]", motion_groups),
            config=config,
            stop_conditions=stop_conditions,
            start_joint_position=start_joint_position,
            pipeline=pipeline,
        )
    return TcpJogger(
        {motion_groups: tcp},
        config=config,
        stop_conditions=stop_conditions,
        start_joint_position=start_joint_position,
        pipeline=pipeline,
    )
