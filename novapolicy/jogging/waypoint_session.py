"""Waypoint jogging session — sends timestamped position waypoints directly.

Uses NOVA's action chunk streaming endpoint (``ActionChunkRequest``, carrying
joint- or pose-flavoured waypoints) to stream action chunks. The server handles
velocity profiling, interpolation, and limits internally.
"""

from __future__ import annotations

import asyncio
import contextlib
from dataclasses import dataclass
import logging
import math
import time
from typing import TYPE_CHECKING

from websockets.exceptions import InvalidStatus

from nova import api
from nova.types import Pose, RobotState
from novapolicy._sdk import get_api_gateway, get_cell, get_controller_id
from novapolicy.io import IOWriter
from novapolicy.jogging.clock import JoggingTimeClock
from novapolicy.jogging.session import JoggingStateTracker
from novapolicy.jogging.waypoints import (
    PendingChunk,
    anchor_timestamp_ms,
    make_waypoints_request,
    step_spacing_ms,
)
from novapolicy.types import JoggingNotSupportedError, MotionError, StopContext

if TYPE_CHECKING:
    from collections.abc import AsyncGenerator, Callable, Mapping

    from nova.cell.motion_group import MotionGroup
    from novapolicy.types import JoggingMode, StopCondition, ValueType, WaypointConfig

logger = logging.getLogger(__name__)

_HTTP_NOT_FOUND = 404

# Smallest lead a waypoint may still be sent with. Waypoints closer than this to
# the robot's current session time are unreachable, and commanding them makes
# the server jump to catch up.
MIN_LEAD_MS = 20.0

# Longest a graceful stop waits for already-sent waypoints to finish executing.
# Generous next to the horizon it drains (a few hundred ms), so it only trips
# when the server has genuinely stopped reporting progress — a stalled link, or a
# session the robot never started executing.
_DRAIN_TIMEOUT_S = 3.0


@dataclass(slots=True)
class _ScheduledRequest:
    """A pending chunk resolved into the request that goes on the wire."""

    request: api.models.ActionChunkRequest | None
    """``None`` when every waypoint had already elapsed and nothing was sent."""

    skipped: int
    """Leading waypoints dropped for being unreachable."""

    step_timestamps: list[int]
    """Timestamp per *caller* step, including any trimmed or dropped ones."""


# Joint gap (deg) between a chunk's first step and the robot's current position
# above which we treat it as a genuine discontinuity worth a WARNING (smaller
# gaps are normal continuous-replacement lag and are logged at DEBUG).
_DISCONTINUITY_WARN_DEG = 10.0


class WaypointJoggingSession:  # ruff: ignore[too-many-public-methods]
    """Sends action chunks as timestamped waypoints via NOVA action chunk streaming.

    Sends raw position waypoints (joint or TCP) with timing info.
    The server computes the motion profile and handles IK (for TCP mode).
    """

    def __init__(
        self,
        motion_group: MotionGroup,
        config: WaypointConfig,
        *,
        tcp: str = "",
        mode: JoggingMode = "joint",
        stop_conditions: list[StopCondition] | None = None,
    ) -> None:
        self._motion_group = motion_group
        self._config = config
        self._tcp = tcp
        self._mode: JoggingMode = mode
        self._stop_conditions = stop_conditions or []
        self._io_values: dict[str, object] | None = None
        self._io_writer = IOWriter(motion_group)
        self._jog_tracker = JoggingStateTracker(motion_group.id)

        # Current robot state (updated by state stream)
        self._current_joints: list[float] | None = None
        self._current_tcp_pose: Pose | None = None
        self._current_tcp_name: str | None = None
        self._num_joints: int | None = None
        self._current_state_server_ms: int | None = None
        self._state_observer: Callable[[RobotState, int, float], None] | None = None

        # Stop-condition state
        self._prev_state: RobotState | None = None
        self._prev_tick_time: float | None = None
        self._stop_condition: str | None = None
        """Name of the stop condition that fired (normal stop, not a failure)."""

        # Server time synchronization: auto-computes the speed ratio between
        # server clock and client wall-clock, then scales outgoing timestamps
        # so the robot moves at real-time speed.
        self._clock = JoggingTimeClock()

        # Pending waypoints to send (set by update_chunk, consumed by jogging loop).
        # For normal chunks, store raw steps/timing and build the request at
        # yield-time so timestamps are computed as late as possible.
        self._pending_request: PendingChunk | None = None
        self._pending_request_event = asyncio.Event()
        self._queued_chunk_count = 0
        self._scheduled_chunk_count = 0
        self._scheduled_until_server_ms = 0
        self._scheduled_waypoint_timestamps: list[int] = []
        self._scheduled_step_timestamps: list[int] = []
        self._scheduled_action_timestep = -1
        self._scheduled_at_server_ms = 0

        # Task management
        self._jogging_task: asyncio.Task[None] | None = None
        self._state_task: asyncio.Task[None] | None = None
        self._running = False
        self._ready = asyncio.Event()
        self._failed = False
        self._failure_reason: str = ""
        self._failure_exception: BaseException | None = None
        self._waypoint_chunk_count = 0
        self._monitor_chunk_index = 0
        self._monitor_waypoints: list[tuple[int, list[float]]] = []
        self._monitor_next_waypoint = 0

    def set_io_values_ref(self, values: dict[str, object]) -> None:
        """Set the shared IO values dict (from IOStreamCache)."""
        self._io_values = values

    @property
    def motion_group(self) -> MotionGroup:
        return self._motion_group

    @property
    def motion_group_id(self) -> str:
        return self._motion_group.id

    @property
    def num_joints(self) -> int | None:
        """Number of joints, known after :meth:`start`. None before."""
        return self._num_joints

    @property
    def mode(self) -> JoggingMode:
        """Jogging mode: 'joint' or 'cartesian'."""
        return self._mode

    @property
    def current_state(self) -> RobotState | None:
        if self._current_joints is None or self._current_tcp_pose is None:
            return None
        return self._build_robot_state()

    def set_state_observer(self, observer: Callable[[RobotState, int, float], None] | None) -> None:
        """Call ``observer`` once per state packet, not once per control tick.

        Receives ``(state, jogger_session_timestamp_ms, generated_monotonic)``.
        Anything derived from state — a tracking error, a plotted trail — belongs
        here: sampling it from the control loop instead re-reads whatever packet
        happens to be cached, which during a delivery burst is the same one for
        tens of milliseconds, and silently discards the rest — on a bursty link
        that lost roughly half of all states.

        The observer runs inside the state-stream task and must be cheap.
        """
        self._state_observer = observer

    @property
    def current_state_server_ms(self) -> int | None:
        """Jogger-session timestamp the cached :attr:`current_state` was measured at.

        State packets are delivered in bursts on some deployments — on a wandelbox,
        stretches of silence approaching 100ms followed by a dozen packets at once
        — so the cached pose can be far older than "now". Anything comparing a
        commanded value against this pose has to line the two up on *this*
        timestamp; using wall-clock "now" instead reports a tracking error that
        ramps at the path speed and snaps back when the burst lands.
        """
        return self._current_state_server_ms

    async def wait_ready(self, timeout_s: float = 10.0) -> None:
        """Wait until the jogging session is initialized or fail if startup dies."""
        deadline = time.monotonic() + timeout_s
        while not self._ready.is_set() or not self._clock.state_clock_calibrated:
            if self._failed:
                if self._failure_exception is not None:
                    raise RuntimeError(
                        f"Waypoint jogging failed for {self.motion_group_id}: "
                        f"{self._failure_reason}"
                    ) from self._failure_exception
                raise RuntimeError(
                    f"Waypoint jogging failed for {self.motion_group_id}: {self._failure_reason}"
                )
            if time.monotonic() >= deadline:
                reason = (
                    self._failure_reason or "no ready acknowledgement or state-clock calibration"
                )
                raise RuntimeError(
                    f"Timed out waiting for waypoint jogging readiness for "
                    f"{self.motion_group_id}: {reason}"
                )
            await asyncio.sleep(0.05)

    @property
    def jogging_state(self) -> str | None:
        """Latest NOVA waypoint-jogging execution state."""
        return self._jog_tracker.last_kind

    @property
    def is_running(self) -> bool:
        """Whether the robot is actively executing jogging motion.

        Driven by the jogging state stream reporting ``kind == "RUNNING"``.
        Reflects the robot's actual execution state (the control loop engages a
        moment after the first waypoint), so it marks when motion truly begins
        — the right moment to start a time-parameterised target.
        """
        return self._jog_tracker.last_kind == "RUNNING"

    @property
    def has_failed(self) -> bool:
        return self._failed

    @property
    def stop_condition_triggered(self) -> str | None:
        """Name of the stop condition that ended the session, or ``None``.

        Set when a stop condition returns ``True``. This is a *normal* stop,
        not a failure — ``has_failed`` stays ``False``.
        """
        return self._stop_condition

    @property
    def session_elapsed_ms(self) -> int:
        """Session "now", extrapolated from the latest acknowledged server state.

        Driven by :attr:`JoggingTimeClock.acknowledged_elapsed_ms`. The
        extrapolation is deliberately uncapped — capping it deadlocks startup,
        because the jogger timer only advances once waypoints execute — so on a
        stalled link this keeps advancing and the clock warns instead. The value
        is in raw server milliseconds, which are real milliseconds.
        """
        return self._clock.acknowledged_elapsed_ms

    @property
    def queued_chunk_count(self) -> int:
        """Sequence number of the latest chunk queued by the executor."""
        return self._queued_chunk_count

    @property
    def scheduled_chunk_count(self) -> int:
        """Sequence number of the latest chunk timestamped for NOVA."""
        return self._scheduled_chunk_count

    @property
    def scheduled_until_server_ms(self) -> int:
        """When the commanded motion of the latest scheduled chunk runs out.

        The timestamp of the last waypoint the *caller* asked for — not of the
        last waypoint sent. Short chunks are padded to ``min_chunk_horizon_ms`` by
        repeating their final target (see :meth:`_extend_to_min_horizon`), and
        that padding is a braking horizon for the server rather than motion
        anyone requested. Waiting for it would sit at the final target for the
        rest of the buffer before the next chunk could be considered, which at a
        500ms buffer is most of the wait.
        """
        return self._scheduled_until_server_ms

    @property
    def scheduled_waypoint_timestamps(self) -> tuple[int, ...]:
        """NOVA timestamps of the waypoints actually sent in the latest request.

        Trimmed and padded as sent, so this describes the request on the wire.
        To ask where one of the *caller's* steps landed, use
        :meth:`scheduled_timestamp_for_step` — indices here do not line up with
        the caller's own once anything has been trimmed.
        """
        return tuple(self._scheduled_waypoint_timestamps)

    def scheduled_timestamp_for_step(self, step: int) -> int | None:
        """Absolute NOVA timestamp assigned to the caller's step ``step``.

        Indexed by the caller's own step numbering, so trimming and dropping are
        invisible here: a step whose moment had already passed keeps the
        timestamp it was given, which is what makes it read as due. Indexing
        :attr:`scheduled_waypoint_timestamps` instead shifts every lookup by the
        number of trimmed waypoints.

        ``None`` when the latest chunk had no such step.
        """
        if 0 <= step < len(self._scheduled_step_timestamps):
            return self._scheduled_step_timestamps[step]
        return None

    @property
    def scheduled_action_timestep(self) -> int:
        """Policy timestep represented by the latest scheduled request."""
        return self._scheduled_action_timestep

    @property
    def scheduled_at_server_ms(self) -> int:
        """Latest server timestamp observed when the request was scheduled."""
        return self._scheduled_at_server_ms

    @property
    def last_server_timestamp_ms(self) -> int:
        """Latest raw NOVA jogger-session timestamp from the state stream."""
        return self._clock.last_server_timestamp_ms

    @property
    def estimated_server_timestamp_ms(self) -> int:
        """Estimated current raw NOVA jogger-session timestamp."""
        return self._clock.estimated_server_timestamp_ms

    @property
    def single_step_dt_ms(self) -> float:
        """Spacing used when a live target is expanded into a short horizon."""
        return self._config.single_step_dt_ms

    @property
    def min_chunk_horizon_ms(self) -> float:
        """Minimum waypoint horizon handed to the controller."""
        return self._config.min_chunk_horizon_ms

    @property
    def failure_reason(self) -> str:
        return self._failure_reason

    @property
    def failure_exception(self) -> BaseException | None:
        return self._failure_exception

    def update_chunk(
        self,
        steps: list[list[float]],
        dt_ms: float,
        *,
        first_timestamp_ms: int | None = None,
        timestamp_offset_steps: int = 0,
        server_dt_ms: float | None = None,
        action_timestep: int = -1,
        extend_buffer: bool = True,
        **_kwargs: object,
    ) -> None:
        """Queue a new action chunk as waypoints.

        Builds an ActionChunkRequest carrying joint- or pose-flavoured waypoints
        (based on mode) with absolute server-time timestamps laid out as
        ``base + i*dt``. The request is sent on the next jogging loop iteration;
        the timestamps are computed at *yield time* (see
        :func:`make_waypoints_request`).

        Args:
            steps: Joint waypoints [rad] or TCP poses [x,y,z,rx,ry,rz] (mm/rad).
            dt_ms: Time between consecutive waypoints (ms). 0 = single-step.
            first_timestamp_ms: Exact raw NOVA jogger-session timestamp for
                step zero. When omitted, server "now" is resolved immediately
                before sending so the timestamp cannot become stale in the queue.
            timestamp_offset_steps: Shift the selected timestamp by whole
                ``dt`` steps. ``+1`` places step zero one interval ahead; a
                negative value backdates an overlapping seam; ``0`` is exact.
            server_dt_ms: Explicit waypoint spacing override in server
                milliseconds. Server milliseconds are real milliseconds, so this
                only matters when the spacing must differ from ``dt_ms``.
            action_timestep: Absolute policy timestep represented by ``steps[0]``.
                Logged with the scheduled request in Rerun.
        """
        if not steps:
            return

        effective_dt_ms = dt_ms if dt_ms > 0 else self._config.single_step_dt_ms
        buffered_steps = (
            self._extend_to_min_horizon(steps, effective_dt_ms) if extend_buffer else steps
        )

        # Store raw chunk data. Timestamps are computed in _jogging_loop
        # immediately before yielding to the server, avoiding drift from any
        # internal await/scheduling delay between policy and stream send.
        self._queued_chunk_count += 1
        self._pending_request = PendingChunk(
            steps=buffered_steps,
            dt_ms=effective_dt_ms,
            first_timestamp_ms=first_timestamp_ms,
            timestamp_offset_steps=timestamp_offset_steps,
            server_dt_ms=server_dt_ms,
            action_timestep=action_timestep,
            sequence=self._queued_chunk_count,
            caller_step_count=len(steps),
        )
        self._pending_request_event.set()

        # Compare current robot position vs chunk first step (joint mode only).
        # Skipped for backdated overlapping chunks: there the robot always
        # lags the freshest prediction's step 0 by a few degrees, which is
        # expected and not worth reporting. For exact/ahead chunks a large
        # first-step gap is a genuine discontinuity worth a WARNING.
        if (
            timestamp_offset_steps >= 0
            and self._mode == "joint"
            and self._current_joints is not None
            and len(buffered_steps) > 0
        ):
            delta = [
                abs(buffered_steps[0][j] - self._current_joints[j])
                for j in range(min(3, len(buffered_steps[0])))
            ]
            max_delta = max(delta) * 57.3
            log = logger.warning if max_delta > _DISCONTINUITY_WARN_DEG else logger.debug
            log(
                "%s: chunk first step is %.1f deg from current position "
                "(current=[%.4f,%.4f,%.4f] chunk_first=[%.4f,%.4f,%.4f])",
                self.motion_group_id,
                max_delta,
                self._current_joints[0],
                self._current_joints[1],
                self._current_joints[2],
                buffered_steps[0][0],
                buffered_steps[0][1],
                buffered_steps[0][2],
            )

        # Request is built later at yield time.

    def _extend_to_min_horizon(
        self, steps: list[list[float]], effective_dt_ms: float
    ) -> list[list[float]]:
        """Extend a short chunk by holding its final caller-provided target."""
        min_chunk_horizon_ms = max(0.0, self._config.min_chunk_horizon_ms)
        if min_chunk_horizon_ms <= 0 or effective_dt_ms <= 0:
            return steps

        min_steps = max(1, math.ceil(min_chunk_horizon_ms / effective_dt_ms))
        if len(steps) >= min_steps:
            return steps

        return [*steps, *[list(steps[-1]) for _ in range(min_steps - len(steps))]]

    async def write_ios(self, ios: Mapping[str, ValueType]) -> None:
        """Write IO values (delegated to IOWriter for deduplication)."""
        await self._io_writer.write(ios)

    async def start(self) -> None:
        """Start the state stream and jogging loop."""
        if self._running:
            msg = f"WaypointJoggingSession for {self.motion_group_id} is already running."
            raise RuntimeError(msg)

        self._running = True
        self._pending_request_event.clear()

        initial_state = await self._motion_group.get_state()
        self._current_joints = list(initial_state.joints)
        self._current_tcp_pose = initial_state.pose
        self._current_tcp_name = initial_state.tcp
        self._num_joints = len(initial_state.joints)

        self._state_task = asyncio.create_task(
            self._stream_state(), name=f"wp-state-{self.motion_group_id}"
        )
        self._jogging_task = asyncio.create_task(
            self._jogging_loop(), name=f"wp-jog-{self.motion_group_id}"
        )
        logger.info(
            "WaypointJoggingSession started for %s (%d joints)",
            self.motion_group_id,
            self._num_joints,
        )

    async def drain(self, timeout_s: float = _DRAIN_TIMEOUT_S) -> bool:
        """Wait for the waypoints already sent to finish executing.

        Every waypoint carries an absolute server timestamp, so "finished" is
        simply the server *acknowledging* a time at or past the last one the
        caller asked for (see :attr:`scheduled_until_server_ms`). Until
        then the motion is still owed to the caller: it was accepted, and
        :meth:`stop` cancelling the jogging task would throw it away mid-path.

        This matters most with a rolling buffer, where by construction everything
        sent lies in the future — the robot trails the newest target by the buffer
        duration, so cancelling immediately truncates that much of the path.

        Returns whether the schedule was reached before ``timeout_s``. Does not
        stop the session; call :meth:`stop` afterwards.
        """
        if not self._running or (self._scheduled_until_server_ms <= 0 and not self._pending()):
            return True
        deadline = time.monotonic() + timeout_s
        poll_s = max(self._config.state_rate_ms, 1) / 1000.0
        while True:
            # Re-read the schedule every pass rather than snapshotting it. The
            # jogging task is still running, so the caller's last ``set_target``
            # may only be queued when the drain starts — snapshotting would
            # finish before that chunk was ever timestamped and sent.
            target_ms = self._scheduled_until_server_ms
            # Acknowledged server progress, not the extrapolated estimate. That
            # estimate free-runs at wall-clock rate between state samples and is
            # uncapped, so on a stalled link it sails past the schedule; draining
            # against it would report the motion finished when the robot may not
            # have started it — exactly the truncation a drain exists to prevent.
            if not self._pending() and self._clock.last_server_timestamp_ms >= target_ms:
                return True
            # A fault or a fired stop condition means the rest of the path is no
            # longer wanted; surface it rather than waiting the schedule out.
            if self._failed or self._stop_condition is not None:
                return False
            if time.monotonic() >= deadline:
                logger.warning(
                    "%s still %dms short of its waypoint schedule after %.1fs; stopping anyway",
                    self.motion_group_id,
                    target_ms - self._clock.last_server_timestamp_ms,
                    timeout_s,
                )
                return False
            await asyncio.sleep(poll_s)

    def _pending(self) -> bool:
        """Whether a queued chunk is still waiting to be timestamped and sent."""
        return self._pending_request is not None

    async def stop(self) -> None:
        """Stop the session gracefully.

        Cancels immediately: anything still scheduled is dropped. Call
        :meth:`drain` first to let the accepted waypoints run out.
        """
        self._running = False

        for task in (self._jogging_task, self._state_task):
            if task is not None:
                task.cancel()
                with contextlib.suppress(asyncio.CancelledError, OSError, RuntimeError):
                    await task

        self._jogging_task = None
        self._state_task = None

        logger.info("WaypointJoggingSession stopped for %s", self.motion_group_id)

    # -------------------------------------------------------------------------
    # State stream
    # -------------------------------------------------------------------------

    async def _stream_state(self) -> None:
        """Continuously read state for guards and observation building."""
        stream = None
        try:  # ruff: ignore[too-many-statements-in-try-clause]
            stream = self._motion_group.stream_state(response_rate_msecs=self._config.state_rate_ms)
            async for state in stream:
                self._current_joints = list(state.joint_position)
                if state.tcp_pose is not None:
                    self._current_tcp_pose = Pose(state.tcp_pose)
                if state.tcp is not None:
                    self._current_tcp_name = state.tcp
                self._jog_tracker.update_from_state(state)
                state_timestamp = getattr(state, "timestamp", None)
                self._clock.observe_state_timestamp(state_timestamp)
                # Extract server jogger session timestamp for time synchronization.
                ts_ms = JoggingTimeClock.extract_from_state(state)
                if ts_ms is None:
                    continue
                self._clock.update(ts_ms, state_timestamp)
                self._current_state_server_ms = ts_ms
                self._notify_state_observer(ts_ms)
                self._measure_waypoint_tracking(ts_ms)
        except asyncio.CancelledError:
            # Expected on shutdown; stop quietly without logging as an error.
            raise
        except (OSError, RuntimeError) as e:
            logger.error("State stream error for %s: %s", self.motion_group_id, e)
            self._note_state_stream_gone(str(e), e)
        else:
            # The server closed the stream while we still wanted it.
            self._note_state_stream_gone("state stream ended", None)
        finally:
            if stream is not None:
                with contextlib.suppress(asyncio.CancelledError, OSError, RuntimeError):
                    await stream.aclose()

    def _note_state_stream_gone(self, reason: str, error: BaseException | None) -> None:
        """Fail the session when the state stream goes away before it is ready.

        Startup waits on this stream: the server/client clock skew is calibrated
        from it, so without it :meth:`wait_ready` can only run out its timeout and
        report a missing calibration — which says nothing about the stream having
        died, and buries the error that was already logged. Recording the failure
        lets ``wait_ready`` raise the real cause immediately.

        Once the session is fully startable a lost stream is left to the jogging
        loop and its own error handling, which is where an actual motion failure
        surfaces. "Startable" has to mean ready *and* calibrated, the same pair
        :meth:`wait_ready` waits on: the jogging websocket can set ``_ready``
        before the state stream has delivered a single packet, and standing down
        on readiness alone would leave that case with no way to ever calibrate
        and no failure to report — the misleading timeout this exists to prevent.
        """
        if not self._running or self._failed:
            return
        if self._ready.is_set() and self._clock.state_clock_calibrated:
            return
        self._failed = True
        self._failure_reason = f"{reason} for {self.motion_group_id}"
        self._failure_exception = error

    def _notify_state_observer(self, ts_ms: int) -> None:
        """Hand this packet to the observer, stamped when it was generated."""
        observer = self._state_observer
        if observer is None:
            return
        state = self._build_robot_state()
        if state is None:
            return
        # Before the first server timestamp there is no generation instant to
        # place this on, so fall back to now: it is the best estimate available
        # and, unlike the clock's zero-initialised field, it is a real instant.
        generated_at = self._clock.last_sample_wall
        if generated_at is None:
            generated_at = time.monotonic()
        try:
            observer(state, ts_ms, generated_at)
        except (OSError, RuntimeError, ValueError, TypeError, AttributeError) as e:
            # Observers are best-effort (visualisation); never break the stream.
            # AttributeError included deliberately: an observer reading a field
            # off a partially-populated state must not kill the state task.
            logger.debug("State observer failed for %s: %s", self.motion_group_id, e)

    # -------------------------------------------------------------------------
    # Jogging loop (waypoint mode)
    # -------------------------------------------------------------------------

    async def _consume_jogging_responses(
        self,
        response_stream: AsyncGenerator[api.models.ExecuteActionChunksResponse, None],
    ) -> None:
        """Monitor acknowledgements and failures without gating outgoing chunks."""
        async for response in response_stream:
            if not self._running:
                return
            if not self._ready.is_set():
                self._ready.set()
            if hasattr(response.root, "kind") and response.root.kind == "MOTION_ERROR":
                msg = getattr(response.root, "message", "unknown motion error")
                raise MotionError(self.motion_group_id, msg)
            self._check_stop_conditions()
            self._jog_tracker.check()

    @staticmethod
    async def _wait_for_signal_or_response(
        signal: asyncio.Event,
        response_task: asyncio.Task[None],
    ) -> bool:
        """Wait for a producer signal while still surfacing response failures."""
        signal_task = asyncio.create_task(signal.wait())
        try:
            done, _pending = await asyncio.wait(
                (signal_task, response_task),
                return_when=asyncio.FIRST_COMPLETED,
            )
            if response_task in done:
                _ = await response_task
                return False
            return True
        finally:
            if not signal_task.done():
                signal_task.cancel()
            with contextlib.suppress(asyncio.CancelledError):
                _ = await signal_task

    async def _pending_waypoint_requests(
        self,
        response_task: asyncio.Task[None],
    ) -> AsyncGenerator[api.models.ExecuteActionChunksRequest, None]:
        """Yield fresh pending chunks at the producer cadence."""
        first_chunk = True
        while self._running:
            self._check_stop_conditions()
            self._jog_tracker.check()

            if not self._ready.is_set():
                if not await self._wait_for_signal_or_response(self._ready, response_task):
                    return
                continue
            if self._pending_request is None:
                if not await self._wait_for_signal_or_response(
                    self._pending_request_event, response_task
                ):
                    return
                continue

            # Capture and clear before yielding. A target queued while the
            # websocket send is in progress remains pending for the next send.
            pending = self._pending_request
            self._pending_request = None
            self._pending_request_event.clear()

            if pending is None:
                continue

            if first_chunk:
                self._clock.start()
                first_chunk = False

            trimmed = self._trimmed_request(pending)
            if trimmed.request is None:
                self._record_dropped_chunk(pending, trimmed.step_timestamps)
                continue
            request = trimmed.request
            self._log_waypoint_timing(request)
            self._scheduled_chunk_count = pending.sequence
            self._scheduled_step_timestamps = list(trimmed.step_timestamps)
            self._scheduled_waypoint_timestamps = [
                waypoint.timestamp for waypoint in request.waypoints
            ]
            # The caller's own last step, not the padded tail — see
            # :attr:`scheduled_until_server_ms`.
            self._scheduled_until_server_ms = trimmed.step_timestamps[pending.last_caller_step]
            # Trimming moves step zero forward, so the timestep this reports has
            # to move with it — it names the policy step ``steps[0]`` came from.
            # ``-1`` means "no timestep", and stays that way.
            self._scheduled_action_timestep = (
                pending.action_timestep + trimmed.skipped
                if pending.action_timestep >= 0
                else pending.action_timestep
            )
            self._scheduled_at_server_ms = self._clock.last_server_timestamp_ms
            self._start_waypoint_tracking_measurement(request)

            yield api.models.ExecuteActionChunksRequest(request)

    def _record_dropped_chunk(self, pending: PendingChunk, step_timestamps: list[int]) -> None:
        """Account for a chunk that was dropped for being entirely in the past.

        The chunk still has to count as scheduled. Callers wait for
        :attr:`scheduled_chunk_count` to reach what they queued, so a silently
        skipped chunk leaves that wait unsatisfiable for the rest of the run —
        the executor's policy boundary never opens and its IO writes never fire.

        Its step timestamps are recorded too, unchanged: every one of them is in
        the past, which is exactly what makes the boundary read as already due.
        Nothing went on the wire, so the sent-waypoint list is cleared and
        :attr:`scheduled_until_server_ms` is left alone — the server is still
        executing the previous chunk, and that motion is still owed.
        """
        self._scheduled_chunk_count = pending.sequence
        self._scheduled_step_timestamps = list(step_timestamps)
        self._scheduled_waypoint_timestamps = []
        self._scheduled_action_timestep = pending.action_timestep
        self._scheduled_at_server_ms = self._clock.last_server_timestamp_ms

    def _trimmed_request(self, pending: PendingChunk) -> _ScheduledRequest:
        """Build the request, dropping waypoints whose moment has already passed.

        A chunk is timestamped when the caller builds it, but it is sent a little
        later. That delay eats into the lead, and once step zero lands at or
        behind the robot's current position the server has to jump to catch the
        trajectory — heavy velocity ripple that gets worse the longer a chunk
        waited, reaching several times the commanded speed for a slow caller.

        Trimming keeps the absolute time-to-position mapping exactly as the
        caller defined it and simply starts at the first waypoint the robot can
        still reach, which is what a fixed-timeline stream does by construction.

        The returned record always carries a timestamp for *every* step the
        caller passed, whether it was sent, trimmed or dropped, so callers can
        still ask where any of their own steps landed.
        """
        steps = pending.steps
        dt_ms = step_spacing_ms(pending.dt_ms, pending.server_dt_ms)
        base_ms = pending.first_timestamp_ms

        if base_ms is None or dt_ms <= 0:
            # No caller-supplied anchor to measure the lead against: "now" is
            # resolved inside the request builder, so nothing can be stale yet.
            request = make_waypoints_request(
                self._clock,
                self._mode,
                steps=steps,
                effective_dt_ms=pending.dt_ms,
                first_timestamp_ms=base_ms,
                timestamp_offset_steps=pending.timestamp_offset_steps,
                server_dt_ms=pending.server_dt_ms,
            )
            return _ScheduledRequest(
                request=request,
                skipped=0,
                step_timestamps=[waypoint.timestamp for waypoint in request.waypoints],
            )

        anchor_ms = max(0, anchor_timestamp_ms(base_ms, pending.timestamp_offset_steps, dt_ms))
        earliest_ms = self._clock.estimated_server_timestamp_ms + MIN_LEAD_MS
        skip = max(0, math.ceil((earliest_ms - anchor_ms) / dt_ms))

        if skip >= len(steps):
            # The whole chunk is in the past; the caller's next target will
            # supersede it. Sending it would only make the robot lurch.
            logger.debug(
                "%s dropping chunk %d: all %d waypoints are in the past",
                self.motion_group_id,
                pending.sequence,
                len(steps),
            )
            return _ScheduledRequest(
                request=None,
                skipped=len(steps),
                step_timestamps=[anchor_ms + int(i * dt_ms) for i in range(len(steps))],
            )

        # The offset is already folded into ``anchor_ms``, so it must not be
        # applied a second time here.
        request = make_waypoints_request(
            self._clock,
            self._mode,
            steps=steps[skip:],
            effective_dt_ms=pending.dt_ms,
            first_timestamp_ms=anchor_ms + int(skip * dt_ms),
            timestamp_offset_steps=0,
            server_dt_ms=pending.server_dt_ms,
        )
        # Dropped steps keep the timestamps they were going to be sent with, so
        # the caller's step numbering still indexes this list directly.
        return _ScheduledRequest(
            request=request,
            skipped=skip,
            step_timestamps=[
                *(anchor_ms + int(i * dt_ms) for i in range(skip)),
                *(waypoint.timestamp for waypoint in request.waypoints),
            ],
        )

    async def _jogging_loop(self) -> None:
        """Open jogging session and send waypoints when available.

        Responses are consumed independently so network round-trip latency does
        not gate outgoing waypoint updates. When no chunk is pending, the loop
        waits instead of sending hold messages that could alter the profile.
        """
        api_gateway = get_api_gateway(self._motion_group)
        cell = get_cell(self._motion_group)
        controller_id = get_controller_id(self._motion_group)
        tcp = await self._resolve_tcp()

        async def client_request_generator(
            response_stream: AsyncGenerator[api.models.ExecuteActionChunksResponse, None],
        ) -> AsyncGenerator[api.models.ExecuteActionChunksRequest, None]:
            # 1. Initialize the action chunk session.
            # The server starts its internal timer when the first waypoint
            # request arrives (not on InitializeActionChunksRequest).
            yield api.models.ExecuteActionChunksRequest(
                api.models.InitializeActionChunksRequest(
                    motion_group=self._motion_group.id, tcp=tcp
                )
            )

            response_task = asyncio.create_task(
                self._consume_jogging_responses(response_stream),
                name=f"wp-jog-responses-{self.motion_group_id}",
            )
            try:
                # Send prepared chunks independently of response latency. New
                # targets still overwrite _pending_request before it is sent,
                # so stale updates are coalesced without coupling the command
                # cadence to a websocket round trip.
                async for request in self._pending_waypoint_requests(response_task):
                    yield request
            finally:
                if not response_task.done():
                    response_task.cancel()
                with contextlib.suppress(asyncio.CancelledError, OSError, RuntimeError):
                    _ = await response_task

        try:
            await api_gateway.action_chunk_streaming_api.execute_action_chunks(
                cell=cell,
                controller=controller_id,
                client_request_generator=client_request_generator,
            )
        except asyncio.CancelledError:
            # Expected on shutdown; cancellation is not a jogging failure.
            pass
        except InvalidStatus as e:
            # An old api-gateway (< 26.6) has no executeActionChunks endpoint
            # and rejects the websocket upgrade with HTTP 404. Surface that as an
            # actionable error rather than a generic connection loss.
            if e.response.status_code == _HTTP_NOT_FOUND:
                err = JoggingNotSupportedError(self.motion_group_id)
                self._failed = True
                self._failure_reason = str(err)
                self._failure_exception = err
                self._running = False
                logger.error("%s", err)
            else:
                self._failed = True
                self._failure_reason = str(e)
                self._failure_exception = e
                self._running = False
                logger.error("Jogging connection rejected for %s: %s", self.motion_group_id, e)
        except MotionError as e:
            self._failed = True
            self._failure_reason = str(e)
            self._failure_exception = e
            self._running = False
            logger.warning("Waypoint jogging stopped for %s: %s", self.motion_group_id, e)
        except (OSError, RuntimeError) as e:
            if self._running:
                self._failed = True
                self._failure_reason = str(e)
                self._failure_exception = e
                self._running = False
                logger.error("Jogging connection lost for %s: %s", self.motion_group_id, e)

    def _start_waypoint_tracking_measurement(self, request: object) -> None:
        """Record targets so state samples can measure tracking at each deadline."""
        waypoints = getattr(request, "waypoints", None)
        if self._mode != "joint" or not waypoints:
            self._monitor_waypoints = []
            return
        self._monitor_chunk_index = self._waypoint_chunk_count
        self._monitor_waypoints = [
            (waypoint.timestamp, list(waypoint.waypoint.root.joints.root)) for waypoint in waypoints
        ]
        self._monitor_next_waypoint = 0

    def _measure_waypoint_tracking(self, server_timestamp_ms: int) -> None:
        """Log actual joint error when the NOVA clock crosses waypoint deadlines."""
        if not self._monitor_waypoints or self._current_joints is None:
            return
        while self._monitor_next_waypoint < len(self._monitor_waypoints):
            index = self._monitor_next_waypoint
            target_timestamp_ms, target_joints = self._monitor_waypoints[index]
            if server_timestamp_ms < target_timestamp_ms:
                return
            max_error_deg = (
                max(
                    abs(current - target)
                    for current, target in zip(self._current_joints, target_joints, strict=True)
                )
                * 57.3
            )
            log = logger.info if index in {0, len(self._monitor_waypoints) - 1} else logger.debug
            log(
                "%s waypoint tracking chunk=%d index=%d deadline=%dms "
                "observed=%dms lateness=%dms max_error=%.2fdeg",
                self.motion_group_id,
                self._monitor_chunk_index,
                index,
                target_timestamp_ms,
                server_timestamp_ms,
                server_timestamp_ms - target_timestamp_ms,
                max_error_deg,
            )
            self._monitor_next_waypoint += 1
        self._monitor_waypoints = []

    def _log_waypoint_timing(self, request: object) -> None:
        waypoints = getattr(request, "waypoints", None)
        if not waypoints:
            return
        self._waypoint_chunk_count += 1
        timestamps = [waypoint.timestamp for waypoint in waypoints]
        server_dt_ms = timestamps[1] - timestamps[0] if len(timestamps) > 1 else 0
        logger.debug(
            "%s waypoint chunk=%d count=%d server_sample=%dms first=%dms last=%dms dt=%dms",
            self.motion_group_id,
            self._waypoint_chunk_count,
            len(timestamps),
            self._clock.last_server_timestamp_ms,
            timestamps[0],
            timestamps[-1],
            server_dt_ms,
        )

    async def _resolve_tcp(self) -> str:
        """Get the TCP name for jogging."""
        if self._tcp:
            return self._tcp
        tcp = await self._motion_group.active_tcp_name()
        if tcp is not None:
            return tcp
        tcp_names = await self._motion_group.tcp_names()
        if tcp_names:
            return tcp_names[0]
        logger.warning("No TCP found for %s", self.motion_group_id)
        return ""

    def _check_stop_conditions(self) -> None:
        """Evaluate stop conditions with the current state.

        A condition returning ``True`` ends the session normally: it records the
        condition's name and stops the loop. This is *not* a failure — the
        executor turns the recorded name into an ``ExecutionResult`` reason.
        """
        if not self._stop_conditions:
            return

        current_state = self._build_robot_state()
        if current_state is None:
            return

        now = time.monotonic()
        dt = now - self._prev_tick_time if self._prev_tick_time is not None else 0.01

        ctx = StopContext(
            state=current_state,
            prev_state=self._prev_state,
            dt=dt,
            motion_group_id=self.motion_group_id,
            io_values=self._io_values,
        )
        for condition in self._stop_conditions:
            if condition(ctx):
                self._stop_condition = getattr(condition, "__name__", repr(condition))
                self._running = False
                logger.info(
                    "Stop condition '%s' triggered for %s",
                    self._stop_condition,
                    self.motion_group_id,
                )
                return

        self._prev_state = current_state
        self._prev_tick_time = now

    def _build_robot_state(self) -> RobotState | None:
        """Construct a RobotState from cached values."""
        if self._current_joints is None or self._current_tcp_pose is None:
            return None
        return RobotState(
            pose=self._current_tcp_pose,
            tcp=self._current_tcp_name,
            joints=tuple(self._current_joints),
        )
