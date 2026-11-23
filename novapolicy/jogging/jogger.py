"""Position-controlled jogging for one or more motion groups.

Provides ``jog_joints()`` and ``jog_tcp()`` — async context managers that
open jogging sessions. The user sets target positions in a loop; the session
streams timestamped waypoints to the NOVA action chunk streaming API.

Faults are detected automatically and raised through the ``async for`` loop:

- ``MotionError`` — joint limit or self-collision
- ``EmergencyStopError`` — e-stop, protective stop, safety violation
- ``RuntimeError`` — jogging connection lost

A triggered stop condition is not a fault: it ends the loop normally and the
triggering condition's name is available on ``jogger.stop_condition_triggered``.
"""

from __future__ import annotations

import asyncio
import bisect
import contextlib
import logging
import time
from typing import TYPE_CHECKING, TypeAlias, cast, overload

from novapolicy.estop import (
    EstopMonitor,
    check_estop,
    check_sessions,
    triggered_stop_condition,
)
from novapolicy.jogging.clock import ServerMs
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
    """Whether ``value`` is a chunk of future steps rather than one target.

    An empty list is not a chunk: there is nothing to infer a shape from, and
    treating it as one would send a request with no waypoints.
    """
    return isinstance(value, list) and bool(value) and isinstance(value[0], list)


# The timing constants below were tuned against a single UR10e reached over a
# wandelbox. The mechanisms they answer to are general; the numbers are not, and
# a different robot, controller firmware or link will want its own values. Treat
# them as defaults to re-tune, and prefer the reasoning over the figures.

# How far ahead of session time step zero is placed for an explicit chunk. Must
# outlast the interval between pushes (plus link latency), or the seam between
# an old and a new chunk gets executed. Deliberately constant: anything
# time-varying in the time-to-position mapping is executed as a lurch.
LEAD_MS = 100.0

# How much commanded history is retained for Rerun tracking, in server ms. Only
# needs to outlast the delivery burst that makes the cached pose stale.
_COMMAND_HISTORY_MS = 1000


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
        self._estop: EstopMonitor | None = None
        self._rerun: PolicyRerunLogger | None = None
        self._loop_t0: float | None = None
        self._ack0_ms: float = 0.0
        self._tick_ms: float | None = None
        self._timeline_ms: float = 0.0
        self._commanded: dict[str, dict[int, list[float]]] = {}

    @property
    def elapsed(self) -> float:
        """Seconds of acknowledged motion since the jogging motion actually started.

        Holds at ``0.0`` until the robot reports it is actively executing motion
        (all sessions :attr:`~WaypointJoggingSession.is_running`), then ticks
        from zero. It is anchored on the first loop iteration that has state,
        not on the robot reporting RUNNING: waiting for RUNNING deadlocks the
        live path, because the timeline has to move before any target velocity
        can be measured and the robot has to move before it reports RUNNING.
        A catch-up jump is not a concern either way: targets are placed on an
        absolute timeline and unreachable waypoints are dropped at send time.

        Crucially this advances on the **server** jogger clock, not wall-clock,
        so it stays in step with the timeline the waypoints are anchored on. The
        extrapolation between state samples is uncapped, so on a stalled link it
        keeps advancing rather than freezing; see :attr:`_timeline_ms` handling
        below for why forward jumps are kept and backward ones are not.
        """
        if self._loop_t0 is None:
            return 0.0
        # Sampled once per loop iteration (see __aiter__), so every target the
        # caller derives from this value is timestamped from the same instant.
        #
        # This MUST come from the server clock, not the monotonic clock. The
        # robot executes against the jogger session timer, and wall time drifts
        # from it; driving the trajectory clock from monotonic time measured
        # far worse in two separate experiments (10mm+ tracking error and
        # repeated stalls), even though it is the smoother of the two.
        now_ms = self._tick_ms if self._tick_ms is not None else self._acknowledged_ms()
        # Quantisation and the never-backwards ratchet are shared with the
        # executor's buffered mode; see trajectory_elapsed_ms for why each rule
        # is there.
        self._timeline_ms = trajectory_elapsed_ms(now_ms, self._ack0_ms, self._timeline_ms)
        return self._timeline_ms / 1000.0

    def _acknowledged_ms(self) -> float:
        """Acknowledged session "now" (ms), most conservative across all arms.

        Taking the minimum means the shared jog timeline only advances as fast
        as the slowest-acknowledged motion group, so the arm furthest behind
        sets the pace for all of them.
        """
        return min(s.session_elapsed_ms for s in self._sessions.values())

    def _sessions_by_id(self) -> dict[str, WaypointJoggingSession]:
        """Sessions keyed by motion group ID (for Rerun streaming)."""
        return {mg.id: session for mg, session in self._sessions.items()}

    def _anchor(self, mg: MotionGroup, lead_ms: float = LEAD_MS) -> ServerMs | None:
        """Server timestamp for content generated at this iteration's instant.

        The whole session shares **one fixed timeline**: content generated at trajectory time
        ``T`` always maps to ``ack0 + T + lead``, whenever it is sent. Re-deriving the anchor
        from server "now" at send time makes the same trajectory point land on a slightly
        different timestamp in every chunk ("now" is sampled at a jittery, aged instant);
        measured on a UR10e, that turned a jitter-free motion into a visibly vibrating one.
        The lead is constant for the same reason: anything time-varying here moves commanded
        points to new times, and the robot executes each shift as a lurch.

        ``None`` before the timeline is anchored: callers then place "now"-relative.
        """
        if mg not in self._sessions or self._loop_t0 is None:
            return None
        return ServerMs(int(self._ack0_ms + self.elapsed * 1000.0 + lead_ms))

    def _expected_dims(self, mg: MotionGroup) -> int | None:
        """Expected target dimension for a motion group. Override in subclass."""
        session = self._sessions.get(mg)
        return session.num_joints if session else None

    def _set_live_target(self, mg: MotionGroup, values: list[float]) -> None:
        """Validate one live target and send it exactly as measured.

        One waypoint, where the target actually is. Nothing is inferred about
        where it is going next — that horizon is the server's job, and until the
        API provides it a ``LiveBuffer`` in the pipeline stands in.

        Without a ``LiveBuffer`` in the pipeline this goes out as it is; with one, the session
        turns it into a horizon once the buffer can define a spacing. A lone waypoint is a *terminal* target the server decelerates to a
        standstill at, so motion is halting there. That is the honest cost of never
        guessing.
        """
        expected = self._expected_dims(mg)
        if expected is not None and len(values) != expected:
            msg = f"Target has {len(values)} values but motion group '{mg.id}' expects {expected}"
            raise ValueError(msg)
        session = self._sessions.get(mg)
        if session is not None:
            steps = [list(values)]
            anchor = self._anchor(mg, LIVE_LEAD_MS)
            # When the target was produced, on the loop's one timeline shared by every arm: a
            # LiveBuffer in the pipeline times targets by it, so the arms stay on one grid.
            # Rounded apart: the loop time is on a 10 ms grid, and truncating the sum would put
            # some targets 1 ms early, a whole grid step on the buffer's own grid.
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
            # Log what the session queued, after the pipeline's chunk modules: with a LiveBuffer
            # the target is commanded about the window later, and plotting the raw target would
            # show that delay as tracking error. (Waypoint modules such as EaseIn reshape values
            # at the send and are not reflected here.)
            queued = session.queued_chunk
            if queued is not None:
                self._record_commanded(
                    mg.id, queued.steps, queued.dt_ms, queued.first_timestamp_ms or anchor
                )
            self._log_target(mg.id, steps, 0.0)
            return
        self._log_target(mg.id, [values], 0.0)

    def _send_chunk(self, mg: MotionGroup, chunk: _Chunk, dt_ms: float) -> list[float]:
        """Send one motion group's chunk. Returns its final step.

        The chunk is placed on the absolute session timeline at
        :data:`LEAD_MS`, so the same trajectory point keeps the same timestamp
        however often the chunk is re-sent.
        """
        session = self._sessions.get(mg)
        if session is not None:
            anchor = self._anchor(mg)
            session.update_chunk(
                steps=chunk,
                dt_ms=dt_ms,
                first_timestamp_ms=(
                    anchor if anchor is not None else session.estimated_server_timestamp_ms
                ),
                # Stated rather than left to the default: a chunk is the one form
                # HoldPadding pads, and this is where that is decided.
                extend_buffer=True,
            )
            self._record_commanded(mg.id, chunk, dt_ms, anchor)
            self._log_target(mg.id, chunk, dt_ms)
        return chunk[-1]

    def _dispatch_chunk(
        self, chunk: _Chunk | dict[MotionGroup, _Chunk], dt_ms: float
    ) -> dict[MotionGroup, list[float]]:
        """Validate and send a chunk per motion group; return each one's final step.

        Shared by both joggers: a chunk is ``list[list[float]]`` in joint space and
        in TCP space alike, so only the live-target form differs between them.
        """
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

    def _record_commanded(
        self, mg_id: str, steps: list[list[float]], dt_ms: float, anchor_ms: int | None
    ) -> None:
        """Remember what was commanded for each server millisecond.

        Only kept while Rerun is logging — it exists so the tracking plot can
        line a command up with the state that was measured at the same moment.
        A later chunk overwrites an earlier one at the same timestamp, which is
        what the server does with it too.
        """
        if self._rerun is None or anchor_ms is None or not steps:
            return
        history = self._commanded.setdefault(mg_id, {})
        spacing = dt_ms if dt_ms > 0 else 0.0
        for i, step in enumerate(steps):
            history[anchor_ms + int(i * spacing)] = list(step)
            if spacing <= 0:
                break
        cutoff = anchor_ms - _COMMAND_HISTORY_MS
        for timestamp in [t for t in history if t < cutoff]:
            del history[timestamp]

    def _commanded_at(self, mg_id: str, server_ms: int | None) -> list[float] | None:
        """What was commanded for ``server_ms``, interpolated between waypoints."""
        history = self._commanded.get(mg_id)
        if not history or server_ms is None:
            return None
        timestamps = sorted(history)
        index = bisect.bisect_left(timestamps, server_ms)
        if index == 0:
            return history[timestamps[0]]
        if index >= len(timestamps):
            return history[timestamps[-1]]
        before, after = timestamps[index - 1], timestamps[index]
        start, end = history[before], history[after]
        fraction = (server_ms - before) / (after - before)
        return [start[k] + fraction * (end[k] - start[k]) for k in range(len(start))]

    def _log_target(self, mg_id: str, steps: list[list[float]], dt_ms: float) -> None:
        """Log jogging target to Rerun as an action chunk visualization."""
        if self._rerun is None:
            return
        from novapolicy.types import ActionChunk  # ruff: ignore[import-outside-top-level]

        # Only the commanded chunk is logged here. Tracking error belongs to
        # the state, not to this tick, and is logged per state packet in
        # :meth:`_log_state_tracking`.
        session = self._sessions_by_id().get(mg_id)
        if session is not None and session.mode == "cartesian":
            chunk = ActionChunk(tcp={mg_id: steps}, dt_ms=dt_ms)
        else:
            chunk = ActionChunk(joints={mg_id: steps}, dt_ms=dt_ms)
        self._rerun.log_action_chunk(chunk, step=0)

    def _log_state_tracking(
        self,
        mg_id: str,
        mode: str,
        state: RobotState,
        server_ms: int,
        generated_at: float,
    ) -> None:
        """Log commanded-vs-actual for one state packet, at its own instant.

        Driven by the state stream rather than the control loop. The two run at
        different rates and, more importantly, states arrive in bursts: sampling
        the cached pose once per control tick re-reads the same packet for tens of
        milliseconds at a time and drops the rest, which draws the tracking error
        as a repeating flat shelf and, on a bursty link, discarded roughly half of
        all states. One entry per packet plots every state at the
        moment it was generated, so the trace is continuous.

        The command is resolved for *this packet's* server timestamp, so both
        sides of the difference describe the same instant.
        """
        if self._rerun is None:
            return
        self._rerun.log_robot_state(mg_id, state, at=generated_at)
        commanded = self._commanded_at(mg_id, server_ms)
        if commanded is None:
            return
        if mode == "cartesian":
            self._rerun.log_tcp_tracking(mg_id, commanded, state, step=0, at=generated_at)
        else:
            self._rerun.log_joint_tracking(mg_id, commanded, state, step=0, at=generated_at)

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
        # Let the waypoints already accepted finish before tearing the session
        # down. They describe motion the caller asked for, and with a rolling
        # buffer all of it lies in the future by construction, so cancelling
        # straight away stops the robot part-way through the commanded path.
        #
        # Skipped when the loop ended for a reason that wants the robot stopped
        # NOW: a fault, an e-stop, or a fired stop condition. Draining those
        # would keep the robot moving for the length of the horizon after
        # something asked it to stop.
        if exc_type is None and self.stop_condition_triggered is None:
            await self._drain_sessions()
        self._stop_observing_states()
        if self._rerun is not None:
            await self._rerun.stop_streaming()
            self._rerun = None
        if self._estop is not None:
            await self._estop.stop()
            self._estop = None
        # In parallel: each stop waits for its server to settle.
        results = await asyncio.gather(
            *(session.stop() for session in self._sessions.values()), return_exceptions=True
        )
        for result in results:
            if isinstance(result, BaseException) and not isinstance(
                result, (asyncio.CancelledError, OSError, RuntimeError)
            ):
                raise result
        logger.info("%s stopped", self.__class__.__name__)
        return False

    async def _drain_sessions(self) -> None:
        """Wait out every session's outstanding waypoint schedule, in parallel."""
        with contextlib.suppress(asyncio.CancelledError, OSError, RuntimeError):
            _ = await asyncio.gather(
                *(session.drain() for session in self._sessions.values()),
                return_exceptions=True,
            )

    async def _init_rerun(self) -> None:
        """Initialize Rerun logger if a viewer is active."""
        from novapolicy.rerun import _is_rerun_active  # ruff: ignore[import-outside-top-level]

        if not _is_rerun_active():
            return

        from novapolicy.rerun import PolicyRerunLogger  # ruff: ignore[import-outside-top-level]

        self._rerun = PolicyRerunLogger(
            self._mg_list,
            use_tcp_offset_for_joint_actions=True,
        )
        await self._rerun.initialize()
        if self._rerun is not None:
            self._rerun.start_streaming(self._sessions_by_id())
            self._observe_states_for_rerun()

    def _observe_states_for_rerun(self) -> None:
        """Log tracking error from the state stream for as long as Rerun is on."""
        for mg, session in self._sessions.items():
            mg_id, mode = mg.id, session.mode

            def observe(
                state: RobotState,
                server_ms: int,
                generated_at: float,
                mg_id: str = mg_id,
                mode: str = mode,
            ) -> None:
                self._log_state_tracking(mg_id, mode, state, server_ms, generated_at)

            session.set_state_observer(observe)

    def _stop_observing_states(self) -> None:
        for session in self._sessions.values():
            session.set_state_observer(None)

    async def _move_to_start_joint_position(self) -> None:
        """PTP move all motion groups to their start_joint_position positions."""
        import asyncio as _asyncio  # ruff: ignore[import-outside-top-level]

        from nova.actions import jnt  # ruff: ignore[import-outside-top-level]

        async def _ptp(mg: MotionGroup, joints: list[float]) -> None:
            tcp = await mg.active_tcp_name() or (await mg.tcp_names())[0]
            # Clear collision setups so the cell's safety planes don't reject
            # planning at this exact start pose (the same relaxation a manual
            # PTP-to-home would use).
            setup = await mg.get_setup(tcp)
            setup.collision_setups = {}
            target = tuple(joints)
            traj = await mg.plan([jnt(target)], tcp, motion_group_setup=setup)
            await mg.execute(traj, tcp, actions=[jnt(target)])

        start_positions = self._start_joint_position
        if start_positions is None:
            return
        tasks = [_ptp(mg, joints) for mg, joints in start_positions.items()]
        await _asyncio.gather(*tasks)

    @property
    def stop_condition_triggered(self) -> str | None:
        """Name of the stop condition that ended jogging, or ``None``.

        A fired stop condition ends the ``async for`` loop normally (no
        exception); read this afterwards to learn which one fired.
        """
        return triggered_stop_condition(self._sessions)

    async def __aiter__(
        self,
    ) -> AsyncIterator[dict[MotionGroup, RobotState] | RobotState]:
        """Yield current state at ~100Hz. Raises on faults; a stop condition ends
        the loop normally (see :attr:`stop_condition_triggered`). Use ``break`` to stop.
        """
        while True:
            check_sessions(self._sessions)
            check_estop(self._estop)
            if triggered_stop_condition(self._sessions) is not None:
                return
            s = self.state()
            if s is None:
                await asyncio.sleep(0.01)
                continue
            if self._loop_t0 is None:
                # Anchor as soon as state is available. Waiting for the robot to
                # report RUNNING deadlocks the live path: the timeline stays at
                # zero, so no target velocity can be measured, so every push is
                # a single terminal waypoint, so the robot never starts moving
                # and never reports RUNNING. A catch-up jump is not a concern
                # either way: targets are placed on an absolute timeline and
                # unreachable waypoints are dropped at send time.
                self._loop_t0 = time.monotonic()
                # Baseline the acknowledged clock at the same instant so
                # elapsed ticks from zero on the acknowledged timeline.
                self._ack0_ms = self._acknowledged_ms()
            # One timeline sample per iteration: everything the caller does with
            # this state (read elapsed, build a chunk, push targets) shares it.
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

        Each target goes out as one waypoint. With a ``LiveBuffer`` in the pipeline, recent
        targets are replayed as a continuous waypoint horizon instead, so the server always has
        somewhere to be going next; spacing is measured from the trajectory times the samples
        were produced at.

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

        A chunk already describes where the robot goes next, so it carries its own horizon: a
        ``LiveBuffer`` passes it unchanged and clears its window. A chunk shorter than
        ``HoldPadding.min_horizon_ms`` is padded by repeating its final step, and leading steps
        whose moment has already passed are dropped at send time.

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

    async def __aenter__(self) -> JointJogger:
        await super().__aenter__()
        return self


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
        stop_conditions: Optional callbacks run on every jogging tick.
            Each receives a ``StopContext`` and returns ``True`` to stop the loop.
        start_joint_position: Joint positions to PTP-move to before starting jogging.
            Single list for one robot, or dict mapping each motion group
            to its start_joint_position joints for multi-robot.
        pipeline: The modules between the targets and NOVA, in order, exactly as given;
            ``None`` runs :func:`~novapolicy.jogging.modules.default_pipeline`. To add one,
            extend that list: ``[LiveBuffer(window_ms=500), *default_pipeline()]`` replays the
            last ``window_ms`` of live targets as a horizon on an even grid, so
            :meth:`set_target` streams move smoothly about the window behind (measured at a
            ~90 Hz loop: 0.015 deg rms against the target, 0.45 to 0.84 deg without); a chunk
            passes it unchanged and clears it. ``[*default_pipeline(), EaseIn(duration_s=1.0)]``
            starts the motion from a standstill. See :mod:`novapolicy.jogging.pipeline`.

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
        config: Waypoint jogging configuration.
        stop_conditions: Optional callbacks run on every jogging tick.
            Each receives a ``StopContext`` and returns ``True`` to stop the loop.
        start_joint_position: Joint positions to PTP-move to before starting jogging.
            Single list for one robot, or dict mapping each motion group
            to its start joints for multi-robot.
        pipeline: The modules between the targets and NOVA, in order, exactly as given;
            ``None`` runs :func:`~novapolicy.jogging.modules.default_pipeline`. To add one,
            extend that list: ``[LiveBuffer(window_ms=500), *default_pipeline()]`` replays the
            last ``window_ms`` of live targets as a horizon on an even grid, so
            :meth:`set_target` streams move smoothly about the window behind (measured at a
            ~90 Hz loop: 0.015 deg rms against the target, 0.45 to 0.84 deg without); a chunk
            passes it unchanged and clears it. ``[*default_pipeline(), EaseIn(duration_s=1.0)]``
            starts the motion from a standstill. See :mod:`novapolicy.jogging.pipeline`.

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
