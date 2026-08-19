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
import bisect
import contextlib
import logging
import math
import time
from typing import TYPE_CHECKING, TypeAlias, cast, overload

from novapolicy.estop import EstopMonitor, check_estop, check_sessions, triggered_stop_condition
from novapolicy.jogging.waypoint_session import WaypointJoggingSession
from novapolicy.types import WaypointConfig

if TYPE_CHECKING:
    from collections.abc import AsyncIterator

    from nova.cell.motion_group import MotionGroup
    from nova.types import Pose, RobotState
    from novapolicy.rerun import PolicyRerunLogger
    from novapolicy.types import StopCondition

logger = logging.getLogger(__name__)

_CARTESIAN_DIMS = 6  # x, y, z, rx, ry, rz — fixed by NOVA jogging API

_TargetValues: TypeAlias = list[float] | list[list[float]]
_TimedPoint: TypeAlias = tuple[float, list[float]]

# The timing constants below were tuned against a single UR10e reached over a
# wandelbox. The mechanisms they answer to are general; the numbers are not, and
# a different robot, controller firmware or link will want its own values. Treat
# them as defaults to re-tune, and prefer the reasoning over the figures.

# How far ahead of session time step zero is placed for an explicit chunk. Must
# outlast the interval between pushes (plus link latency), or the seam between
# an old and a new chunk gets executed. Deliberately constant: anything
# time-varying in the time-to-position mapping is executed as a lurch.
LEAD_MS = 100.0

# Lead used for live targets. This is how far ahead of the robot's current
# motion a replacement trajectory starts, and therefore how long the controller
# has to blend onto it. Live targets are replaced ~90 times a second, so the
# controller performs that join constantly; give it too little room and it
# occasionally cannot do so smoothly, which comes out as a brief stall and
# catch-up (the robot stays on the path, but its pace dips).
#
# Every waypoint in the buffer was measured rather than projected, so nothing
# here trades accuracy against distance: the lead can simply be as much room as
# the controller needs to make the join. On the reference rig, speed evenness
# improved monotonically from 30ms up to 100ms, where the stalls disappeared
# altogether; shorter leads stalled on most laps. Where a controller needs more
# or less room than that, this is the knob.
#
# Note this is not about running out of waypoints: lengthening the horizon
# instead does not help, because the constraint is at the *start* of each
# replacement, not the end.
LIVE_LEAD_MS = 100.0

# Quantisation of the session timeline. Anchoring every chunk at a multiple of
# this keeps successive chunks on ONE absolute grid; without it each replacement
# lands at a new phase, which on the reference rig was the difference between
# grossly uneven motion and none measurable at all.
TIMELINE_GRID_MS = 10.0

# A rolling live-target buffer needs at least two samples to define a spacing.
_MIN_BUFFER_SAMPLES = 2

# Hard cap on samples retained in the rolling buffer, on top of its time window.
# That window is measured against ``elapsed``, which holds at 0.0 until the robot
# reports it is executing motion — so while a caller pushes targets into a
# session that has not started (or whose state stream has stalled), the age of
# the oldest sample is always 0 and nothing is ever evicted. The cap is set far
# above what any real rate needs: a 100Hz caller fills a 500ms buffer with ~50
# samples.
_MAX_LIVE_SAMPLES = 256

# How much commanded history is retained for Rerun tracking, in server ms. Only
# needs to outlast the delivery burst that makes the cached pose stale.
_COMMAND_HISTORY_MS = 1000


def _resample_evenly(
    samples: list[_TimedPoint], step_s: float, *, start: float | None = None
) -> list[list[float]]:
    """Put irregularly-timed samples on an evenly spaced grid.

    Linear interpolation at each grid instant, using the time every sample was
    actually taken. The values stay the caller's own — only the instants they
    are read at change — so this resamples a recording rather than predicting
    anything.

    ``start`` pins the grid to an absolute phase; without it the grid begins at
    the oldest sample.
    """
    start = samples[0][0] if start is None else start
    end = samples[-1][0]
    if end < start:
        return []
    # Round the span UP so the grid reaches the newest sample. Truncating drops
    # the last partial interval, and that interval is the leading edge of the
    # horizon — the freshest thing the buffer knows. A grid point landing past
    # the newest sample holds its value (``fraction`` is clamped below), which
    # states the target has stopped rather than guessing where it went next.
    #
    # The epsilon absorbs float division: a span of exactly two steps comes out
    # as 1.9999999999999996 often enough to matter, and truncating that loses a
    # whole waypoint.
    span_steps = math.ceil((end - start) / step_s - 1e-9)
    count = max(_MIN_BUFFER_SAMPLES, span_steps + 1)
    grid: list[list[float]] = []
    index = 0
    for i in range(count):
        moment = start + i * step_s
        while index + 2 < len(samples) and samples[index + 1][0] <= moment:
            index += 1
        before_t, before = samples[index]
        after_t, after = samples[min(index + 1, len(samples) - 1)]
        span = after_t - before_t
        fraction = 0.0 if span <= 0 else min(1.0, max(0.0, (moment - before_t) / span))
        grid.append([before[k] + fraction * (after[k] - before[k]) for k in range(len(before))])
    return grid


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
        buffer_window_ms: float = 500.0,
    ) -> None:
        self._mg_list = mg_list
        self._multi = len(mg_list) > 1
        self._sessions = sessions
        self._start_joint_position = start_joint_position
        self._estop: EstopMonitor | None = None
        self._rerun: PolicyRerunLogger | None = None
        self._loop_t0: float | None = None
        self._ack0_ms: float = 0.0
        self._tick_ms: float | None = None
        self._timeline_ms: float = 0.0
        self._ease_in_s = ease_in_s
        self._ease_baseline: dict[MotionGroup, list[float]] = {}
        if buffer_window_ms < 0:
            msg = "buffer_window_ms must be greater than or equal to 0"
            raise ValueError(msg)
        self._buffer_window_ms = buffer_window_ms
        self._target_buffers: dict[MotionGroup, list[tuple[float, list[float]]]] = {}
        self._warned_mixed_target_forms: set[MotionGroup] = set()
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
        elapsed_ms = max(0.0, now_ms - self._ack0_ms)
        # Quantise to the timeline grid so every chunk anchors at a multiple of
        # it and successive chunks share one absolute grid, instead of each
        # landing at a new phase.
        grid_ms = TIMELINE_GRID_MS
        if grid_ms > 0:
            elapsed_ms = math.floor(elapsed_ms / grid_ms) * grid_ms
        # Never run backwards: server "now" is extrapolated between state
        # samples, so a late sample can resolve below an earlier estimate, and
        # content is a function of this value — a backward step commands the
        # robot to reverse.
        #
        # Forward jumps are deliberately NOT limited. They look like the robot
        # teleporting, but they are the clock *correcting* after our own event
        # loop was late reading the state stream. Rate-limiting them measured far
        # worse (the timeline fell a full second behind, so every waypoint landed
        # in the past and the robot starved): this clock must track the server,
        # jumps included.
        self._timeline_ms = max(self._timeline_ms, elapsed_ms)
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

    def _timeline_timestamp_ms(
        self, mg: MotionGroup, trajectory_time_s: float, lead_ms: float = LEAD_MS
    ) -> int | None:
        """Absolute server timestamp for a target generated at ``trajectory_time_s``.

        The whole session shares **one fixed timeline**: content generated at
        trajectory time ``T`` always maps to ``ack0 + T + lead``, whenever it is
        sent. Re-deriving the anchor from server "now" at send time instead makes
        the same trajectory point land on a slightly different timestamp in every
        chunk, because "now" is sampled at a jittery, aged instant — measured on
        a UR10e that re-anchoring turned an otherwise jitter-free motion into a
        visibly vibrating one.

        Returns ``None`` before the timeline is anchored, so callers fall back to
        a "now"-relative placement for the very first chunks.
        """
        session = self._sessions.get(mg)
        if session is None or self._loop_t0 is None:
            return None
        # The lead is deliberately CONSTANT. Anything time-varying in this
        # mapping (an adaptive lead, a drifting clock offset) moves previously
        # commanded points to new absolute times, and the robot executes each
        # such shift as a lurch -- an adaptive lead measured worse than a fixed
        # one on every pattern tried.
        return int(self._ack0_ms + trajectory_time_s * 1000.0 + lead_ms)

    def _anchor_for_now(self, mg: MotionGroup, lead_ms: float = LEAD_MS) -> int | None:
        """Timeline anchor for content generated at this iteration's instant."""
        return self._timeline_timestamp_ms(mg, self.elapsed, lead_ms)

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
        requested target over the first ``ease_in_s`` seconds, and is unchanged
        afterwards. Each step uses its own time within a chunk.

        The blend follows a smoothstep, not a straight ramp. A straight ramp is
        continuous in position but not in velocity: the blend rate drops from
        full to nothing the instant it completes, and the robot executes that
        step in acceleration as a stumble a few hundred ms later. Measured on
        the reference rig, the linear ramp made successive horizons disagree about
        the same future moment by an order of magnitude more than they do for the
        rest of the run, and the robot dipped to half speed shortly after.
        Smoothstep is zero-derivative at both ends, so entry and exit are both
        gradual.
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
            fraction = min(1.0, (t0 + i * dt_ms / 1000.0) / self._ease_in_s)
            if fraction >= 1.0:
                eased.append(step)
            else:
                e = fraction * fraction * (3.0 - 2.0 * fraction)
                eased.append([base[k] + e * (step[k] - base[k]) for k in range(len(step))])
        return eased

    def _validate_and_push(self, mg: MotionGroup, values: list[float]) -> None:
        """Validate target dimensions and push the target exactly as measured.

        One waypoint, where the target actually is. Nothing is inferred about
        where it is going next — that horizon is the server's job, and until the
        API provides it the rolling buffer in :meth:`_set_live_target` stands in.

        This is the path taken while that buffer is still filling (and whenever
        ``buffer_window_ms`` is 0). A lone waypoint is a *terminal* target the server
        decelerates to a standstill at, so motion is halting for the first
        ``buffer_window_ms`` of a run. That is the honest cost of never guessing.
        """
        self._validate_target_dims(mg, values)
        session = self._sessions.get(mg)
        if session is not None:
            eased = self._ease_steps(mg, [list(values)], 0.0)
            anchor = self._anchor_for_now(mg, LIVE_LEAD_MS)
            session.update_chunk(
                steps=eased,
                dt_ms=0.0,
                first_timestamp_ms=anchor,
                timestamp_offset_steps=0 if anchor is not None else 1,
                extend_buffer=False,
            )
            # Log what was actually commanded, not the raw request: during
            # ease-in they differ by design, and plotting the raw request makes
            # the tracking error look far worse than the robot is behaving.
            self._record_commanded(mg.id, eased, 0.0, anchor)
            self._log_target(mg.id, eased[:1], 0.0)
            return
        self._log_target(mg.id, [values], 0.0)

    def _push_target(
        self,
        mg: MotionGroup,
        value: _TargetValues,
        dt_ms: float,
        *,
        extend_buffer: bool = True,
    ) -> list[float]:
        """Push a single or chunk target to a session. Returns the final target."""
        is_chunk = bool(value) and isinstance(value[0], list)
        if is_chunk:
            chunk = cast("list[list[float]]", value)
            session = self._sessions.get(mg)
            if session is not None:
                eased = self._ease_steps(mg, chunk, dt_ms)
                anchor = self._anchor_for_now(mg)
                session.update_chunk(
                    steps=eased,
                    dt_ms=dt_ms,
                    first_timestamp_ms=(
                        anchor if anchor is not None else session.estimated_server_timestamp_ms
                    ),
                    extend_buffer=extend_buffer,
                )
                self._record_commanded(mg.id, eased, dt_ms, anchor)
                self._log_target(mg.id, eased, dt_ms)
            return chunk[-1]

        target = cast("list[float]", value)
        self._validate_and_push(mg, target)
        return target

    def _set_live_target(self, mg: MotionGroup, values: list[float]) -> None:
        """Send one live target through the rolling buffer, or alone if disabled.

        The server needs a horizon of *future* waypoints or it decelerates, and a
        live target only ever says where the target is **now**. That horizon
        belongs to the API; until the API provides it, this buffer stands in
        without inventing anything: targets are held and replayed ``buffer_window_ms``
        late, so every waypoint sent is a real measured target.

        Every sample is commanded at a **constant** delay after the moment it was
        produced, so a sample always maps to the same absolute time no matter how
        often the window is re-sent. Re-anchoring the window at "now" instead
        pushes the same sample further into the future on every push, so the
        playback start keeps receding and the robot never gets through it.
        """
        if self._buffer_window_ms == 0:
            self._validate_and_push(mg, values)
            return

        self._validate_target_dims(mg, values)
        session = self._sessions.get(mg)
        if session is None:
            return

        now = self.elapsed
        buffer = self._target_buffers.setdefault(mg, [])
        buffer.append((now, list(values)))
        # Keep only the trailing window; a sample is dropped once it is older
        # than buffer_window_ms, so the window length is bounded by time, not by call
        # rate.
        cutoff = now - self._buffer_window_ms / 1000.0
        while len(buffer) > _MIN_BUFFER_SAMPLES and (
            buffer[1][0] < cutoff or len(buffer) > _MAX_LIVE_SAMPLES
        ):
            del buffer[0]

        span_s = buffer[-1][0] - buffer[0][0]
        if len(buffer) < _MIN_BUFFER_SAMPLES or span_s <= 0:
            # Not enough history yet (or the timeline has not started ticking):
            # keep the robot live with a plain single target rather than waiting
            # for the window to fill, which would otherwise never move at all.
            self._validate_and_push(mg, values)
            return

        # Waypoints are laid out at a uniform spacing, so the recorded samples
        # have to be put on a uniform grid *by interpolating at their own
        # timestamps*. Handing them over as-is with an averaged dt replays each
        # one at the wrong moment -- the caller's ticks jitter over 10-13ms, and
        # a quantised timeline collapses some of them -- which time-warps the
        # trajectory and injects velocity noise into what is otherwise exact
        # recorded data — on the reference rig, replaying with an averaged dt was
        # an order of magnitude less even than honouring the timestamps.
        #
        # The grid is ONE absolute grid, not one starting wherever the oldest
        # surviving sample happens to sit: which sample that is changes with
        # every prune, so anchoring to it lands the same recorded moment on a
        # slightly different timestamp in each push -- the same re-anchoring
        # that turns an otherwise clean motion into a vibrating one.
        # The spacing the controller executes best — a per-controller property,
        # so it comes from config rather than being hardcoded — but never coarser
        # than half the window, so a short buffer is still represented by more
        # than its endpoints.
        step_dt_s = min(session.single_step_dt_ms / 1000.0, span_s / 2.0)
        step_dt_ms = step_dt_s * 1000.0
        grid_start = math.ceil(buffer[0][0] / step_dt_s) * step_dt_s
        steps = _resample_evenly(buffer, step_dt_s, start=grid_start)
        if not steps:
            self._validate_and_push(mg, values)
            return
        # The window is the whole horizon: it reaches up to "now" and no further,
        # because nothing beyond it has been measured yet. ``buffer_window_ms`` therefore
        # sets the horizon the server profiles against, which is why it has to be
        # a few hundred milliseconds — the server caps its speed at whatever it
        # can still brake to a stop within.
        anchor = self._timeline_timestamp_ms(
            mg, grid_start + self._buffer_window_ms / 1000.0, LIVE_LEAD_MS
        )
        eased = self._ease_steps(mg, steps, step_dt_ms)
        session.update_chunk(
            steps=eased,
            dt_ms=step_dt_ms,
            first_timestamp_ms=(
                anchor if anchor is not None else session.estimated_server_timestamp_ms
            ),
            extend_buffer=False,
        )
        self._record_commanded(mg.id, eased, step_dt_ms, anchor)
        self._log_target(mg.id, steps, step_dt_ms)

    def _clear_target_buffer(self, mg: MotionGroup) -> None:
        """Forget rolling samples when an explicit chunk replaces live targets.

        A chunk carries its own horizon, so the samples held for the live path are
        stale the moment one arrives. Dropping them is correct, but it is worth
        saying out loud: the buffer then has to refill before the live path can
        build a horizon again, and until it does its targets go out alone as
        terminal waypoints — the halting motion ``buffer_window_ms`` exists to
        avoid. Alternating the two forms on one motion group keeps paying that.

        Nothing is logged for a caller that only ever sends chunks, since there
        are no samples to discard.
        """
        discarded = self._target_buffers.pop(mg, None)
        if discarded and mg not in self._warned_mixed_target_forms:
            self._warned_mixed_target_forms.add(mg)
            logger.warning(
                "%s received a chunk while %d live target(s) were buffered; the "
                "buffer is cleared and live targets will be sent alone until it "
                "refills. Prefer one target form per motion group.",
                mg.id,
                len(discarded),
            )

    def _validate_target_dims(self, mg: MotionGroup, values: list[float]) -> None:
        expected = self._expected_dims(mg)
        if expected is not None and len(values) != expected:
            msg = f"Target has {len(values)} values but motion group '{mg.id}' expects {expected}"
            raise ValueError(msg)

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
        self, mg_id: str, mode: str, state: RobotState, server_ms: int, generated_at: float
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
        for session in self._sessions.values():
            with contextlib.suppress(asyncio.CancelledError, OSError, RuntimeError):
                await session.stop()
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

        from nova import api  # ruff: ignore[import-outside-top-level]
        from nova.actions import jnt  # ruff: ignore[import-outside-top-level]

        async def _ptp(mg: MotionGroup, joints: list[float]) -> None:
            tcp = await mg.active_tcp_name() or (await mg.tcp_names())[0]
            # Clear collision setups so the cell's safety planes don't reject
            # planning at this exact start pose (the same relaxation a manual
            # PTP-to-home would use).
            setup = await mg.get_setup(tcp)
            setup.collision_setups = api.models.CollisionSetups({})
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
        buffer_window_ms: float = 500.0,
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
            motion_groups,
            sessions,
            start_joint_position=home_dict,
            ease_in_s=ease_in_s,
            buffer_window_ms=buffer_window_ms,
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
            dt_ms: Time between explicit chunk steps, in milliseconds. Required
                for a chunk. Ignored for a live single target, whose spacing is
                measured from the trajectory times the samples were produced at.

        Which horizon setting applies depends on which form you pass. A live
        target goes through the ring buffer sized by ``buffer_window_ms``; a chunk
        brings its own horizon and is padded, if short, to
        :attr:`WaypointConfig.min_chunk_horizon_ms`. Pushing a chunk also clears
        the ring buffer, so alternating the two forms on one motion group leaves
        the next few live targets to go out alone until it refills.
        """
        if isinstance(target, list):
            if self._multi:
                msg = "For multiple motion groups, pass a dict[MotionGroup, ...]"
                raise TypeError(msg)
            mg = self._mg_list[0]
            if target and isinstance(target[0], list):
                final = self._push_target(mg, target, dt_ms)
                self._clear_target_buffer(mg)
            else:
                live_target = cast("list[float]", target)
                self._set_live_target(mg, live_target)
                final = live_target
            self._target = {mg: final}
        elif isinstance(target, dict):
            self._target = self._target or {}
            for mg, mg_value in target.items():
                if mg_value and isinstance(mg_value[0], list):
                    self._target[mg] = self._push_target(mg, mg_value, dt_ms)
                    self._clear_target_buffer(mg)
                else:
                    live_target = cast("list[float]", mg_value)
                    self._set_live_target(mg, live_target)
                    self._target[mg] = live_target
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
        buffer_window_ms: float = 500.0,
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
        super().__init__(
            mg_list,
            sessions,
            start_joint_position=home_dict,
            ease_in_s=ease_in_s,
            buffer_window_ms=buffer_window_ms,
        )
        self._target: dict[MotionGroup, Pose | list[float]] | None = None

    def _expected_dims(self, mg: MotionGroup) -> int | None:  # ruff: ignore[unused-method-argument, no-self-use]
        return _CARTESIAN_DIMS

    @property
    def target(
        self,
    ) -> dict[MotionGroup, Pose | list[float]] | Pose | list[float] | None:
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
            dt_ms: Time between explicit chunk steps, in milliseconds. Required
                for a chunk. Ignored for a live single pose, whose spacing is
                measured from the trajectory times the samples were produced at.

        Which horizon setting applies depends on which form you pass. A live pose
        goes through the ring buffer sized by ``buffer_window_ms``; a chunk brings
        its own horizon and is padded, if short, to
        :attr:`WaypointConfig.min_chunk_horizon_ms`. Pushing a chunk also clears
        the ring buffer, so alternating the two forms on one motion group leaves
        the next few live poses to go out alone until it refills.
        """
        from nova.types import Pose  # ruff: ignore[import-outside-top-level]

        if isinstance(target, Pose):
            if self._multi:
                msg = "For multiple motion groups, pass a dict[MotionGroup, Pose]"
                raise TypeError(msg)
            mg = self._mg_list[0]
            self._set_live_target(mg, list(target.position) + list(target.orientation))
            self._target = {mg: target}
        elif isinstance(target, list):
            if self._multi:
                msg = "For multiple motion groups, pass a dict[MotionGroup, ...]"
                raise TypeError(msg)
            mg = self._mg_list[0]
            self._target = {mg: self._push_target(mg, target, dt_ms)}
            self._clear_target_buffer(mg)
        elif isinstance(target, dict):
            self._target = self._target or {}
            for mg, value in target.items():
                if isinstance(value, Pose):
                    self._set_live_target(mg, list(value.position) + list(value.orientation))
                    self._target[mg] = value
                else:
                    self._target[mg] = self._push_target(mg, value, dt_ms)
                    self._clear_target_buffer(mg)
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
    buffer_window_ms: float = ...,
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
    buffer_window_ms: float = ...,
) -> JointJogger:
    pass


def jog_joints(
    motion_groups: MotionGroup | list[MotionGroup],
    *,
    config: WaypointConfig | None = None,
    stop_conditions: list[StopCondition] | None = None,
    start_joint_position: list[float] | dict[MotionGroup, list[float]] | None = None,
    ease_in_s: float = 0.0,
    buffer_window_ms: float = 500.0,
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
        buffer_window_ms: Length of the ring buffer of recent live targets, in
            milliseconds. **Applies to live targets only** — see below.

            Its job is to keep live jogging smooth. A live target says only where
            the target is *now*, and a lone waypoint is a *terminal* target: with
            no successor the server plans a decelerate-to-standstill profile, so a
            target replaced every tick makes the robot stop and restart
            continuously. That is the jerky motion this removes. The buffer holds
            the last ``buffer_window_ms`` of targets and replays them as a
            continuous waypoint horizon, so the server always has somewhere to be
            going next.

            Nothing in that horizon is invented: every waypoint in it is a target
            that was really measured, replayed late rather than extrapolated
            forward. The cost is latency — the robot trails the live target by a
            little over this window.

            It has to be a few hundred milliseconds. The window *is* the horizon,
            and the server caps its speed at whatever it can brake to a stop
            within, so a short window makes the robot creep and pause: on the
            UR10e this was tuned against, a 150ms window stalled a fifth of all
            samples and 450ms still stalled occasionally. Where the safe horizon
            differs, so does this default.

            ``0`` disables buffering: each target is sent alone, as measured, and
            the halting motion described above is what you get. Useful for
            stepping a robot to discrete positions, not for tracking a moving one.

            **Ignored for chunked targets.** A chunk passed to
            :meth:`~JointJogger.set_target` already carries its own horizon, so
            there is nothing to buffer and no latency to pay; pushing one also
            clears whatever the ring buffer had accumulated. The equivalent knob
            for chunks is :attr:`WaypointConfig.min_chunk_horizon_ms`, which pads
            a chunk that is too short to brake within.

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
        ease_in_s=ease_in_s,
        buffer_window_ms=buffer_window_ms,
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
    buffer_window_ms: float = ...,
) -> TcpJogger:
    pass


@overload
def jog_tcp(
    motion_groups: dict[MotionGroup, str],
    *,
    config: WaypointConfig | None = ...,
    stop_conditions: list[StopCondition] | None = ...,
    start_joint_position: dict[MotionGroup, list[float]] | None = ...,
    ease_in_s: float = ...,
    buffer_window_ms: float = ...,
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
    buffer_window_ms: float = 500.0,
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
        buffer_window_ms: Length of the ring buffer of recent live targets, in
            milliseconds. Replaying them as a continuous horizon is what keeps
            live jogging smooth instead of halting; the robot trails the live
            target by a little over this window in exchange. ``0`` sends each
            target alone. **Live targets only** — ignored for chunks, whose
            horizon comes from :attr:`WaypointConfig.min_chunk_horizon_ms`. See
            :func:`jog_joints` for the full trade.

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
            ease_in_s=ease_in_s,
            buffer_window_ms=buffer_window_ms,
        )
    return TcpJogger(
        {motion_groups: tcp},
        config=config,
        stop_conditions=stop_conditions,
        start_joint_position=start_joint_position,
        ease_in_s=ease_in_s,
        buffer_window_ms=buffer_window_ms,
    )
