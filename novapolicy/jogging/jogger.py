"""Position-controlled jogging for one or more motion groups: ``jog_joints()`` and ``jog_tcp()``.

Faults raise from the ``async for`` loop: ``MotionError`` (joint limit, self-collision),
``EmergencyStopError`` (e-stop, protective stop), ``RuntimeError`` (connection lost). A fired stop
condition ends the loop normally; its name is on ``jogger.stop_condition_triggered``.
"""

from __future__ import annotations

import asyncio
import logging
import time
from typing import TYPE_CHECKING, Self, TypeAlias, cast, overload, override

from novapolicy.pipeline.modules.live_buffer import (
    LIVE_LEAD_MS,
    trajectory_elapsed_ms,
)
from novapolicy.transport.clock import ServerMs
from novapolicy.transport.lifecycle import SessionLifecycle, move_to_start_joint_position
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
        self._mg_list = list(motion_groups)
        self._multi = len(self._mg_list) > 1
        self._lifecycle = SessionLifecycle(
            config=config or WaypointConfig(), stop_conditions=stop_conditions, pipeline=pipeline
        )
        sessions = self._lifecycle.build({
            mg: (self._MODE, tcp) for mg, tcp in motion_groups.items()
        })
        self._sessions = {mg: sessions[mg.id] for mg in self._mg_list}
        if start_joint_position is not None and not isinstance(start_joint_position, dict):
            start_joint_position = {self._mg_list[0]: start_joint_position}
        self._start_joint_position = start_joint_position
        self._target: dict[MotionGroup, _Target] | None = None
        self._rerun: PolicyRerunLogger | None = None
        self._loop_t0: float | None = None
        self._ack0_ms: float = 0.0
        self._tick_ms: float | None = None
        self._timeline_ms: float = 0.0

    @property
    def elapsed(self) -> float:
        """Seconds of acknowledged motion since the loop first had state; ``0.0`` before.

        On the session clock, sampled once per loop iteration, quantised and never backwards
        (:func:`~novapolicy.pipeline.modules.live_buffer.trajectory_elapsed_ms`).
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
        # Each term rounded on its own, as for live targets: truncating the sum puts some chunks
        # 1 ms early, off the grid of the others, and the follower takes the 1 ms as a speed jump.
        return ServerMs(round(self._ack0_ms) + round(self.elapsed * 1000.0) + round(LEAD_MS))

    def _expected_dims(self, mg: MotionGroup) -> int | None:
        """Expected target dimension for a motion group. Override in subclass."""
        session = self._sessions.get(mg)
        return session.num_joints if session else None

    def _check_live_target(self, mg: MotionGroup, values: list[float]) -> None:
        """Raise ``ValueError`` for a live target of the wrong width or with a value that is
        not finite."""
        expected = self._expected_dims(mg)
        if expected is not None and len(values) != expected:
            msg = f"Target has {len(values)} values but motion group '{mg.id}' expects {expected}"
            raise ValueError(msg)
        session = self._sessions.get(mg)
        if session is not None:
            session.check_steps([values])

    def _set_live_target(self, mg: MotionGroup, values: list[float]) -> None:
        """Send one live target, checked by :meth:`_check_live_target`, as one waypoint,
        exactly as measured."""
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

    def _per_group(self, value: object, *, chunk: bool) -> dict[MotionGroup, object]:
        """``value`` per motion group: a dict as given, a bare value for the only one; each a
        chunk of future steps (``chunk``) or one live target, else ``TypeError``."""
        kind = "list[list[float]]" if chunk else self._TARGET
        if isinstance(value, dict):
            groups = cast("dict[MotionGroup, object]", value)
        elif self._multi:
            msg = f"For multiple motion groups, pass a dict[MotionGroup, {kind}]"
            raise TypeError(msg)
        else:
            groups = {self._mg_list[0]: value}
        for item in groups.values():
            if _is_chunk(item) == chunk:
                continue
            if not chunk:
                msg = (
                    "set_target takes one live target, not a chunk of future steps. Pass "
                    "it to set_chunk(chunk, dt_ms=...) instead — a chunk carries its own "
                    "horizon, so a LiveBuffer does not apply to it."
                )
            elif isinstance(item, list) and not item:
                msg = "set_chunk got an empty list; a chunk needs at least one step."
            elif isinstance(item, list):
                msg = (
                    "set_chunk expects a chunk of future steps (list[list[float]]) but "
                    "got a single target. Use set_target(...) for one target."
                )
            else:
                msg = f"set_chunk expects list[list[float]], got {type(item)}"
            raise TypeError(msg)
        return groups

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
        self._rerun = await self._lifecycle.start_monitoring()
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
            ValueError: If a target has the wrong number of values or one that is not finite.
        """
        targets = self._per_group(target, chunk=False)
        values = {mg: self._values(value) for mg, value in targets.items()}
        for mg, value in values.items():  # every group before any is sent
            self._check_live_target(mg, value)
        for mg, value in values.items():
            self._set_live_target(mg, value)
        updated = cast("dict[MotionGroup, _Target]", targets)
        self._target = {**(self._target or {}), **updated} if isinstance(target, dict) else updated

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
            ValueError: If ``dt_ms`` is not positive, or a step has the wrong number of values
                or one that is not finite (NaN, infinity).
        """
        if dt_ms <= 0:
            msg = (
                "set_chunk needs a positive dt_ms — the spacing between the "
                f"chunk's steps — but got {dt_ms!r}."
            )
            raise ValueError(msg)
        chunks = cast("dict[MotionGroup, _Chunk]", self._per_group(chunk, chunk=True))
        for mg, steps in chunks.items():  # every group before any is sent
            session = self._sessions.get(mg)
            if session is not None:
                session.check_steps(steps)
        finals = {mg: self._send_chunk(mg, steps, dt_ms) for mg, steps in chunks.items()}
        # A dict updates only the groups it names; a bare chunk replaces outright.
        kept: dict[MotionGroup, _Target] = (
            dict(self._target or {}) if isinstance(chunk, dict) else {}
        )
        self._target = {**kept, **finals}


class JointJogger(_BaseJogger):
    """Joint position jogger. Do not instantiate directly — use :func:`jog_joints`."""


class TcpJogger(_BaseJogger):
    """TCP pose jogger. Do not instantiate directly — use :func:`jog_tcp`."""

    _MODE: JoggingMode = "cartesian"
    _TARGET = "Pose"

    def _expected_dims(self, mg: MotionGroup) -> int | None:  # ruff: ignore[unused-method-argument, no-self-use]
        return _CARTESIAN_DIMS

    @override
    def _values(self, target: object) -> list[float]:
        from nova.types import Pose  # ruff: ignore[import-outside-top-level]

        if not isinstance(target, Pose):
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
            ``None`` runs :func:`~novapolicy.pipeline.defaults.default_pipeline`. Extend that list
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
        dict.fromkeys(groups, ""),
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
