"""Position-controlled jogging for one or more motion groups.

Provides ``jog_joints()`` and ``jog_tcp()`` — async context managers that
open jogging sessions. The user sets target positions in a loop; the session
streams timestamped waypoints to the NOVA jogging API.

Faults are detected automatically and raised through the ``async for`` loop:

- ``MotionError`` — joint limit or self-collision
- ``EmergencyStopError`` — e-stop, protective stop, safety violation
- ``RuntimeError`` — jogging connection lost

A triggered stop condition is not a fault: it ends the loop normally and the
triggering condition's name is available on ``jogger.stop_condition_triggered``.
"""

from __future__ import annotations

import asyncio
import contextlib
import logging
import time
from typing import TYPE_CHECKING, overload

from policy.estop import EstopMonitor, check_estop, check_sessions, triggered_stop_condition
from policy.jogging.waypoint_session import WaypointJoggingSession
from policy.types import WaypointConfig

if TYPE_CHECKING:
    from collections.abc import AsyncIterator

    from nova.cell.motion_group import MotionGroup
    from nova.types import Pose, RobotState
    from policy.rerun import PolicyRerunLogger
    from policy.types import StopCondition

logger = logging.getLogger(__name__)

_CARTESIAN_DIMS = 6  # x, y, z, rx, ry, rz — fixed by NOVA jogging API

# Safety fallback: if the robot never reports a RUNNING jogging state, anchor
# the elapsed clock anyway after this many seconds so the loop can't stall.
_ANCHOR_FALLBACK_S = 2.5


# ---------------------------------------------------------------------------
# Base jogger (shared lifecycle, error detection, state reading)
# ---------------------------------------------------------------------------


class _BaseJogger:
    """Shared logic for joint and TCP joggers."""

    def __init__(
        self,
        mg_list: list[MotionGroup],
        sessions: dict[MotionGroup, WaypointJoggingSession],
        *,
        start_joint_position: dict[MotionGroup, list[float]] | None = None,
        ease_in_s: float = 0.0,
    ) -> None:
        self._mg_list = mg_list
        self._multi = len(mg_list) > 1
        self._sessions = sessions
        self._start_joint_position = start_joint_position
        self._estop: EstopMonitor | None = None
        self._rerun: PolicyRerunLogger | None = None
        self._loop_t0: float | None = None
        self._ack0_ms: float = 0.0
        self._first_yield_t: float | None = None
        self._ease_in_s = ease_in_s
        self._ease_baseline: dict[MotionGroup, list[float]] = {}

    @property
    def elapsed(self) -> float:
        """Seconds of acknowledged motion since the jogging motion actually started.

        Holds at ``0.0`` until the robot reports it is actively executing motion
        (all sessions :attr:`~WaypointJoggingSession.is_running`), then ticks
        from zero. The robot's control loop engages a moment after the first
        waypoint; anchoring on the actual RUNNING state — rather than a fixed
        timer — means a time-parameterised target holds at its start value until
        the robot is genuinely tracking, avoiding the hard catch-up jump on the
        first move. A fallback anchors anyway if RUNNING is never reported, so
        the loop can't stall.

        Crucially this advances on **acknowledged server progress** (capped at
        the chunk horizon), not wall-clock: if the connection stalls, a target
        parameterised on ``elapsed`` stops advancing too, so it stays in step
        with the waypoint anchors and the robot never has to jump to catch up
        with a timeline that ran ahead while the link was down.
        """
        if self._loop_t0 is None:
            return 0.0
        return max(0.0, (self._acknowledged_ms() - self._ack0_ms) / 1000.0)

    def _acknowledged_ms(self) -> float:
        """Acknowledged session "now" (ms), most conservative across all arms.

        Taking the minimum means the shared jog timeline only advances as fast
        as the slowest-acknowledged motion group, so a stall on any one arm
        freezes the whole timeline.
        """
        return min(s.session_elapsed_ms for s in self._sessions.values())

    def _sessions_by_id(self) -> dict[str, WaypointJoggingSession]:
        """Sessions keyed by motion group ID (for Rerun streaming)."""
        return {mg.id: session for mg, session in self._sessions.items()}

    def _expected_dims(self, mg: MotionGroup) -> int | None:
        """Expected target dimension for a motion group. Override in subclass."""
        session = self._sessions.get(mg)
        return session.num_joints if session else None

    def _ease_steps(
        self, mg: MotionGroup, steps: list[list[float]], dt_ms: float
    ) -> list[list[float]]:
        """Blend targets toward a start baseline during the ease-in window.

        Optional, off by default (``ease_in_s == 0``). When enabled, each step is
        interpolated from the robot's position at jogging start toward the
        requested target by ``min(1, t / ease_in_s)`` — so motion (and velocity)
        ramps up smoothly from zero over the first ``ease_in_s`` seconds and is
        unchanged afterwards. Each step uses its own time within a chunk.
        """
        if self._ease_in_s <= 0 or not steps:
            return steps
        session = self._sessions.get(mg)
        if session is None:
            return steps
        base = self._ease_baseline.get(mg)
        if base is None:
            state = session.current_state
            if state is None:
                return steps  # no baseline yet; ease nothing this push
            base = (
                list(state.joints)
                if session.mode == "joint"
                else list(state.pose.position) + list(state.pose.orientation)
            )
            self._ease_baseline[mg] = base
        t0 = self.elapsed
        eased: list[list[float]] = []
        for i, step in enumerate(steps):
            e = min(1.0, (t0 + i * dt_ms / 1000.0) / self._ease_in_s)
            if e >= 1.0:
                eased.append(step)
            else:
                eased.append([base[k] + e * (step[k] - base[k]) for k in range(len(step))])
        return eased

    def _validate_and_push(self, mg: MotionGroup, values: list[float]) -> None:
        """Validate target dimensions and push to session."""
        expected = self._expected_dims(mg)
        if expected is not None and len(values) != expected:
            msg = f"Target has {len(values)} values but motion group '{mg.id}' expects {expected}"
            raise ValueError(msg)
        session = self._sessions.get(mg)
        if session is not None:
            # Single-step live target: anchor at "now", one step ahead so the
            # server has time to reach it from the robot's current position.
            session.update_chunk(
                steps=self._ease_steps(mg, [values], 0.0), dt_ms=0.0, anchor_offset_steps=1
            )
        self._log_target(mg.id, [values], 0.0)

    def _push_target(self, mg: MotionGroup, value: list, dt_ms: float) -> list[float]:
        """Push a single or chunk target to a session. Returns the final target."""
        session = self._sessions.get(mg)
        if session is None:
            return value if not isinstance(value[0], list) else value[-1]
        if value and isinstance(value[0], list):
            # Chunked: absolute anchor on the jogger's own session timeline so
            # overlapping per-tick resends land identical steps at identical
            # timestamps (one coherent trajectory, no seam jump).
            session.update_chunk(
                steps=self._ease_steps(mg, value, dt_ms),
                dt_ms=dt_ms,
                anchor_ms=session.session_elapsed_ms,
            )
            self._log_target(mg.id, value, dt_ms)
            return value[-1]
        self._validate_and_push(mg, value)
        return value

    def _log_target(self, mg_id: str, steps: list[list[float]], dt_ms: float) -> None:
        """Log jogging target to Rerun as an action chunk visualization."""
        if self._rerun is None:
            return
        from policy.types import ActionChunk  # noqa: PLC0415

        # Determine if this is joint or TCP based on session mode
        session_by_id = self._sessions_by_id()
        session = session_by_id.get(mg_id)
        if session is not None and session.mode == "cartesian":
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

    async def __aenter__(self) -> _BaseJogger:
        # PTP to start_joint_position positions before starting jogging
        if self._start_joint_position:
            await self._move_to_start_joint_position()

        for session in self._sessions.values():
            await session.start()
        self._estop = EstopMonitor(self._mg_list)
        await self._estop.start()
        await self._init_rerun()
        # Wait for all sessions to be fully initialized (server acknowledged)
        # before returning control to user code. This ensures the robot is
        # ready to execute waypoints the moment user code starts its timer.
        for session in self._sessions.values():
            await session.wait_ready()
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
        if self._rerun is not None:
            await self._rerun.stop_streaming()
            self._rerun = None
        if self._estop is not None:
            await self._estop.stop()
            self._estop = None
        for session in self._sessions.values():
            with contextlib.suppress(asyncio.CancelledError, OSError, RuntimeError):
                await session.stop()
        logger.info("%s stopped", self.__class__.__name__)
        return False

    async def _init_rerun(self) -> None:
        """Initialize Rerun logger if a viewer is active."""
        from policy.rerun import _is_rerun_active  # noqa: PLC0415

        if not _is_rerun_active():
            return

        from policy.rerun import PolicyRerunLogger  # noqa: PLC0415

        self._rerun = PolicyRerunLogger(self._mg_list)
        await self._rerun.initialize()
        if self._rerun is not None:
            self._rerun.start_streaming(self._sessions_by_id())

    async def _move_to_start_joint_position(self) -> None:
        """PTP move all motion groups to their start_joint_position positions."""
        import asyncio as _asyncio  # noqa: PLC0415

        from nova import api  # noqa: PLC0415
        from nova.actions import jnt  # noqa: PLC0415

        async def _ptp(mg: MotionGroup, joints: list[float]) -> None:
            tcp = await mg.active_tcp_name() or (await mg.tcp_names())[0]
            # Clear collision setups so the cell's safety planes don't reject
            # planning at this exact start pose (the same relaxation a manual
            # PTP-to-home would use).
            setup = await mg.get_setup(tcp)
            setup.collision_setups = api.models.CollisionSetups({})
            traj = await mg.plan([jnt(joints)], tcp, motion_group_setup=setup)
            await mg.execute(traj, tcp, actions=[jnt(joints)])

        tasks = [
            _ptp(mg, joints)
            for mg, joints in self._start_joint_position.items()  # type: ignore[union-attr]
        ]
        await _asyncio.gather(*tasks)

    @property
    def stop_condition_triggered(self) -> str | None:
        """Name of the stop condition that ended jogging, or ``None``.

        A fired stop condition ends the ``async for`` loop normally (no
        exception); read this afterwards to learn which one fired.
        """
        return triggered_stop_condition(self._sessions)

    async def __aiter__(self) -> AsyncIterator[dict[MotionGroup, RobotState] | RobotState]:
        """Yield current state at ~100Hz. Raises on faults; a stop condition ends
        the loop normally (see :attr:`stop_condition_triggered`). Use ``break`` to stop.
        """
        while True:
            check_sessions(self._sessions)
            check_estop(self._estop)
            if triggered_stop_condition(self._sessions) is not None:
                return
            s = self.state()
            if s is not None:
                if self._loop_t0 is None:
                    now = time.monotonic()
                    if self._first_yield_t is None:
                        self._first_yield_t = now
                    running = all(sess.is_running for sess in self._sessions.values())
                    # Anchor the clock once the robot is actually executing
                    # motion, or after a safety fallback so a robot that never
                    # reports RUNNING can't stall the loop.
                    if running or now - self._first_yield_t > _ANCHOR_FALLBACK_S:
                        self._loop_t0 = now
                        # Baseline the acknowledged clock at the same instant so
                        # elapsed ticks from zero on the acknowledged timeline.
                        self._ack0_ms = self._acknowledged_ms()
                yield s
            await asyncio.sleep(0.01)


# ---------------------------------------------------------------------------
# Joint jogger
# ---------------------------------------------------------------------------


class JointJogger(_BaseJogger):
    """joint position jogger.

    Do not instantiate directly — use :func:`jog_joints`.
    """

    def __init__(
        self,
        motion_groups: list[MotionGroup],
        *,
        config: WaypointConfig | None = None,
        stop_conditions: list[StopCondition] | None = None,
        start_joint_position: list[float] | dict[MotionGroup, list[float]] | None = None,
        ease_in_s: float = 0.0,
    ) -> None:
        cfg = config or WaypointConfig()
        sessions: dict[MotionGroup, WaypointJoggingSession] = {}
        for mg in motion_groups:
            sessions[mg] = WaypointJoggingSession(
                motion_group=mg,
                config=cfg,
                mode="joint",
                stop_conditions=stop_conditions,
            )
        # Normalize start_joint_position to dict[MotionGroup, list[float]]
        home_dict: dict[MotionGroup, list[float]] | None = None
        if start_joint_position is not None:
            if isinstance(start_joint_position, dict):
                home_dict = start_joint_position
            else:
                home_dict = {motion_groups[0]: start_joint_position}
        super().__init__(
            motion_groups, sessions, start_joint_position=home_dict, ease_in_s=ease_in_s
        )
        self._target: dict[MotionGroup, list[float]] | None = None

    @property
    def target(self) -> dict[MotionGroup, list[float]] | list[float] | None:
        """Current target (read-only). Use :meth:`set_target` to update."""
        if self._target is None:
            return None
        if not self._multi:
            return self._target.get(self._mg_list[0])
        return self._target

    def set_target(
        self,
        target: (
            list[float] | list[list[float]] | dict[MotionGroup, list[float] | list[list[float]]]
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
    """TCP pose jogger via server-side waypoint jogging.

    Do not instantiate directly — use :func:`jog_tcp`.
    """

    def __init__(
        self,
        motion_groups: dict[MotionGroup, str],
        *,
        config: WaypointConfig | None = None,
        stop_conditions: list[StopCondition] | None = None,
        start_joint_position: list[float] | dict[MotionGroup, list[float]] | None = None,
        ease_in_s: float = 0.0,
    ) -> None:
        cfg = config or WaypointConfig()
        sessions: dict[MotionGroup, WaypointJoggingSession] = {}
        for mg, tcp in motion_groups.items():
            sessions[mg] = WaypointJoggingSession(
                motion_group=mg,
                config=cfg,
                tcp=tcp,
                mode="cartesian",
                stop_conditions=stop_conditions,
            )
        mg_list = list(motion_groups.keys())
        # Normalize start_joint_position to dict[MotionGroup, list[float]]
        home_dict: dict[MotionGroup, list[float]] | None = None
        if start_joint_position is not None:
            if isinstance(start_joint_position, dict):
                home_dict = start_joint_position
            else:
                home_dict = {mg_list[0]: start_joint_position}
        super().__init__(mg_list, sessions, start_joint_position=home_dict, ease_in_s=ease_in_s)
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

    def set_target(
        self,
        target: Pose | list[list[float]] | dict[MotionGroup, Pose | list[list[float]]],
        *,
        dt_ms: float = 0.0,
    ) -> None:
        """Set the TCP tracking target.

        Args:
            target: TCP pose(s) to track.
                - ``Pose`` — single position target (one motion group)
                - ``list[list[float]]`` — chunk of future TCP targets [x,y,z,rx,ry,rz]
                - ``dict[MotionGroup, ...]`` — per-MG targets or chunks
            dt_ms: Time between chunk steps in milliseconds.
                Only used when target is a chunk (list of lists).
        """
        from nova.types import Pose  # noqa: PLC0415

        if isinstance(target, Pose):
            if self._multi:
                msg = "For multiple motion groups, pass a dict[MotionGroup, Pose]"
                raise TypeError(msg)
            mg = self._mg_list[0]
            self._validate_and_push(mg, list(target.position) + list(target.orientation))
            self._target = {mg: target}
        elif isinstance(target, list):
            if self._multi:
                msg = "For multiple motion groups, pass a dict[MotionGroup, ...]"
                raise TypeError(msg)
            mg = self._mg_list[0]
            self._push_target(mg, target, dt_ms)
            self._target = {mg: target[-1] if target and isinstance(target[0], list) else target}
        elif isinstance(target, dict):
            self._target = self._target or {}
            for mg, value in target.items():
                if isinstance(value, Pose):
                    self._validate_and_push(mg, list(value.position) + list(value.orientation))
                    self._target[mg] = value
                else:
                    self._push_target(mg, value, dt_ms)
                    self._target[mg] = value[-1] if value and isinstance(value[0], list) else value
        else:
            msg = f"Expected Pose, list, or dict, got {type(target)}"
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
    config: WaypointConfig | None = ...,
    stop_conditions: list[StopCondition] | None = ...,
    start_joint_position: list[float] | None = ...,
    ease_in_s: float = ...,
) -> JointJogger:
    pass


@overload
def jog_joints(
    motion_groups: list[MotionGroup],
    *,
    config: WaypointConfig | None = ...,
    stop_conditions: list[StopCondition] | None = ...,
    start_joint_position: dict[MotionGroup, list[float]] | None = ...,
    ease_in_s: float = ...,
) -> JointJogger:
    pass


def jog_joints(
    motion_groups: MotionGroup | list[MotionGroup],
    *,
    config: WaypointConfig | None = None,
    stop_conditions: list[StopCondition] | None = None,
    start_joint_position: list[float] | dict[MotionGroup, list[float]] | None = None,
    ease_in_s: float = 0.0,
) -> JointJogger:
    """Create a joint position jogger using server-side waypoint jogging.

    Args:
        motion_groups: Single motion group or list for multi-robot control.
        config: Waypoint jogging configuration.
        stop_conditions: Optional callbacks run on every jogging tick.
            Each receives a ``StopContext`` and returns ``True`` to stop the loop.
        start_joint_position: Joint positions to PTP-move to before starting jogging.
            Single list for one robot, or dict mapping each motion group
            to its start_joint_position joints for multi-robot.
        ease_in_s: If > 0, ramp motion up from a standstill over this many
            seconds at the start, so velocity begins at zero instead of jumping
            to the target's initial speed. Default 0 (disabled).

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
    if not isinstance(motion_groups, list):
        motion_groups = [motion_groups]
    return JointJogger(
        motion_groups,
        config=config,
        stop_conditions=stop_conditions,
        start_joint_position=start_joint_position,
        ease_in_s=ease_in_s,
    )


@overload
def jog_tcp(
    motion_groups: MotionGroup,
    *,
    tcp: str,
    config: WaypointConfig | None = ...,
    stop_conditions: list[StopCondition] | None = ...,
    start_joint_position: list[float] | None = ...,
    ease_in_s: float = ...,
) -> TcpJogger:
    pass


@overload
def jog_tcp(
    motion_groups: dict[MotionGroup, str],
    *,
    config: WaypointConfig | None = ...,
    stop_conditions: list[StopCondition] | None = ...,
    start_joint_position: dict[MotionGroup, list[float]] | None = ...,
) -> TcpJogger:
    pass


def jog_tcp(
    motion_groups: MotionGroup | dict[MotionGroup, str],
    *,
    tcp: str = "",
    config: WaypointConfig | None = None,
    stop_conditions: list[StopCondition] | None = None,
    start_joint_position: list[float] | dict[MotionGroup, list[float]] | None = None,
    ease_in_s: float = 0.0,
) -> TcpJogger:
    """Create a TCP pose jogger using server-side waypoint jogging.

    Args:
        motion_groups: Single motion group (with ``tcp`` kwarg) or
            ``dict[MotionGroup, str]`` mapping each group to its TCP name.
        tcp: TCP name when passing a single motion group.
        config: Waypoint jogging configuration.
        stop_conditions: Optional callbacks run on every jogging tick.
            Each receives a ``StopContext`` and returns ``True`` to stop the loop.
        start_joint_position: Joint positions to PTP-move to before starting jogging.
            Single list for one robot, or dict mapping each motion group
            to its start joints for multi-robot.
        ease_in_s: If > 0, ramp motion up from a standstill over this many
            seconds at the start, so velocity begins at zero instead of jumping
            to the target's initial speed. Default 0 (disabled).

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
            motion_groups,
            config=config,
            stop_conditions=stop_conditions,
            start_joint_position=start_joint_position,
            ease_in_s=ease_in_s,
        )
    return TcpJogger(
        {motion_groups: tcp},
        config=config,
        stop_conditions=stop_conditions,
        start_joint_position=start_joint_position,
        ease_in_s=ease_in_s,
    )
