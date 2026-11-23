"""Waypoint jogging session: streams timestamped joint or pose waypoints for one motion group
through NOVA's action chunk streaming (``ActionChunkRequest``), after the send pipeline."""

from __future__ import annotations

import asyncio
import bisect
import contextlib
import copy
from dataclasses import dataclass, replace
import logging
import math
import time
from typing import TYPE_CHECKING

from websockets.exceptions import InvalidStatus

from nova import api
from nova.types import Pose, RobotState
from novapolicy._sdk import get_api_gateway, get_cell, get_controller_id
from novapolicy.io import IOWriter
from novapolicy.jogging.clock import SINGLE_STEP_MARGIN_MS, JoggingTimeClock, ServerMs
from novapolicy.jogging.modules import MIN_LEAD_MS, JointClamp, default_pipeline
from novapolicy.jogging.pipeline import ModuleContext, split_pipeline
from novapolicy.jogging.session import JoggingStateTracker, StandstillDetector
from novapolicy.jogging.waypoints import (
    PendingChunk,
    anchor_timestamp_ms,
    make_timed_waypoints_request,
)
from novapolicy.joint_limits import fetch_joint_limits
from novapolicy.types import JoggingNotSupportedError, MotionError, StopContext

if TYPE_CHECKING:
    from collections.abc import AsyncGenerator, Awaitable, Callable, Mapping, Sequence

    from nova.cell.motion_group import MotionGroup
    from novapolicy.joint_limits import JointLimits
    from novapolicy.types import JoggingMode, StopCondition, ValueType, WaypointConfig

logger = logging.getLogger(__name__)

_HTTP_NOT_FOUND = 404


_PLAYBACK_SLEW = 0.1
"""Most the spacing between one-step waypoints may differ from the policy's step, as a share of
the step, while the playback clock follows the observations (10%: the commanded speed varies by
at most that much)."""

_PLAYBACK_RESYNC_MS = 100.0
"""Farther than this from "observation + lead" (a stall, a burst), the playback clock jumps back."""

_PLAYBACK_RATE_GAIN = 0.05
"""How fast the playback clock's step follows the measured time between observations (ADR 0017)."""

# Longest a graceful stop waits for already-sent waypoints to finish executing (ADR 0019).
_DRAIN_TIMEOUT_S = 3.0

# How much sent history :meth:`WaypointJoggingSession.commanded_at` keeps, in server ms: only
# needs to outlast the delivery burst that makes a state packet stale.
_COMMAND_HISTORY_MS = 1000

# How long stop() waits, after StopActionChunksRequest, for the state stream to
# report the session no longer RUNNING before it closes the stream anyway.
_STOP_TIMEOUT_S = 2.0


def _jogging_running(state: api.models.MotionGroupState) -> bool:
    """Whether the state reports a jogging session that is RUNNING."""
    details = state.execute.details if state.execute is not None else None
    return isinstance(details, api.models.JoggingDetails) and isinstance(
        details.state, api.models.JoggingRunning
    )


@dataclass(slots=True)
class _ScheduledRequest:
    """A pending chunk resolved into the request that goes on the wire."""

    request: api.models.ActionChunkRequest | None
    """``None`` when every waypoint had already elapsed and nothing was sent."""

    skipped: int
    """Leading waypoints dropped for being unreachable."""

    step_timestamps: list[int]
    """Timestamp per *caller* step, including any trimmed or dropped ones."""


@dataclass
class _PlaybackClock:
    """Due times for one-step waypoints, spaced like a streaming jitter buffer plays out (ADR 0017)."""

    last_due_ms: float | None = None  # due time of the latest one-step waypoint
    _last_target_ms: float | None = None  # its "observation + lead"
    _step_ms: float | None = None  # measured time between observations, averaged

    def due_ms(self, target_ms: float, step_ms: float, earliest_ms: float) -> float:
        """The next one-step waypoint's due time: one step after the previous one, nudged
        toward ``target_ms`` by at most ``_PLAYBACK_SLEW`` of a step. The step is the measured
        time between observations, averaged slowly, starting from the policy's step."""
        last, last_target = self.last_due_ms, self._last_target_ms
        self._last_target_ms = target_ms
        if last is None or last_target is None or step_ms <= 0:
            due = target_ms
        else:
            interval = min(2 * step_ms, max(0.5 * step_ms, target_ms - last_target))
            step_ms = self._step_ms if self._step_ms is not None else step_ms
            step_ms += _PLAYBACK_RATE_GAIN * (interval - step_ms)
            self._step_ms = step_ms
            nominal = last + step_ms
            error = target_ms - nominal
            if abs(error) > _PLAYBACK_RESYNC_MS or nominal < earliest_ms:
                # A stall or a burst: start the clock again from the observation, but no earlier
                # than ``earliest_ms`` (the controller must still be able to plan it) and half
                # a step after the previous waypoint (timestamps must keep increasing).
                due = max(target_ms, earliest_ms, last + 0.5 * step_ms)
            else:
                slew = _PLAYBACK_SLEW * step_ms
                due = nominal + max(-slew, min(slew, error))
        self.last_due_ms = due
        return due


class WaypointJoggingSession:  # ruff: ignore[too-many-public-methods]
    """Sends action chunks as timestamped joint or TCP waypoints via NOVA action chunk streaming;
    the server computes the motion profile and, for TCP waypoints, the IK."""

    def __init__(  # ruff: ignore[too-many-statements]
        self,
        motion_group: MotionGroup,
        config: WaypointConfig,
        *,
        tcp: str = "",
        mode: JoggingMode = "joint",
        stop_conditions: list[StopCondition] | None = None,
        pipeline: Sequence[object] | None = None,
    ) -> None:
        self._motion_group = motion_group
        # Each session runs its own copy, so two arms never share a module's state.
        self._chunk_modules, self._filters, self._waypoint_modules = split_pipeline(
            copy.deepcopy(list(default_pipeline() if pipeline is None else pipeline))
        )
        self._built: list[tuple[int, list[float]]] = []
        """Timed values of the request last built, recorded once it is sent."""
        self._commanded: dict[int, list[float]] = {}
        """Sent waypoint values by timestamp, as the server holds them."""
        self._joint_limits: JointLimits | None = None
        """Read at :meth:`start` when a ``JointClamp`` is in the pipeline."""
        self._config = config
        self._tcp = tcp
        self._mode: JoggingMode = mode
        self._stop_conditions = stop_conditions or []
        self._io_values: dict[str, object] | None = None
        self._io_writer = IOWriter(motion_group)
        self._jog_tracker = JoggingStateTracker(motion_group.id)
        self._standstill = StandstillDetector()

        # Current robot state (updated by state stream)
        self._current_joints: list[float] | None = None
        self._current_tcp_pose: Pose | None = None
        self._current_tcp_name: str | None = None
        self._num_joints: int | None = None
        self._current_state_server_ms: int | None = None
        self._state_observers: list[Callable[[RobotState, int, float], None]] = []

        # Stop-condition state
        self._prev_state: RobotState | None = None
        self._prev_tick_time: float | None = None
        self._stop_condition: str | None = None
        """Name of the stop condition that fired (normal stop, not a failure)."""

        self._clock = JoggingTimeClock()  # session-clock estimate (ADR 0012)
        # One-step waypoints already sent and not yet due, resent with every new one.
        self._lead_window: list[tuple[int, list[float]]] = []
        self._playback = _PlaybackClock()  # due times of one-step waypoints

        # The newest queued chunk, stamped and sent by the jogging loop.
        self._pending_request: PendingChunk | None = None
        self._pending_request_event = asyncio.Event()
        self._stop_requested = False
        self._stop_acknowledged = False
        self._stop_settled = asyncio.Event()
        self._queued_chunk_count = 0
        self._scheduled_chunk_count = 0
        self._due_tasks: set[asyncio.Task[None]] = set()
        self._due_lock = asyncio.Lock()
        self._scheduled_until_server_ms = 0
        self._scheduled_waypoint_timestamps: list[int] = []
        self._scheduled_step_timestamps: list[int] = []
        self._scheduled_start: tuple[int, ServerMs, float] | None = None
        """Policy timestep at the caller's first step, its time and the step spacing."""
        self._last_sent: tuple[int, list[float]] | None = None
        """Timestamp and target of the last waypoint sent, for :meth:`_queue_final_target`."""
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

    def add_state_observer(self, observer: Callable[[RobotState, int, float], None]) -> None:
        """Call ``observer(state, jogger_session_timestamp_ms, generated_monotonic)`` once per
        state packet. Observers run inside the state-stream task and must be cheap."""
        self._state_observers.append(observer)

    def remove_state_observer(self, observer: Callable[[RobotState, int, float], None]) -> None:
        """Stop calling ``observer``; one that was never added is ignored."""
        with contextlib.suppress(ValueError):
            self._state_observers.remove(observer)

    def commanded_at(self, server_ms: int) -> list[float] | None:
        """The waypoint value sent for ``server_ms``, interpolated between sent waypoints and
        clamped to the first and last; ``None`` before anything was sent."""
        history = self._commanded
        if not history:
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
        return [a + fraction * (b - a) for a, b in zip(start, end, strict=False)]

    @property
    def current_state_server_ms(self) -> ServerMs | None:
        """Jogger-session timestamp the cached :attr:`current_state` was measured at; compare
        commanded values against the state on this timestamp, not on "now"."""
        ts = self._current_state_server_ms
        return None if ts is None else ServerMs(ts)

    async def wait_ready(self, timeout_s: float = 10.0) -> None:
        """Wait until the server acknowledged the session and the clock is calibrated; raise
        ``RuntimeError`` if startup fails or times out. Returns early on a fired stop condition."""
        deadline = time.monotonic() + timeout_s
        while not self._ready.is_set() or not self._clock.state_clock_calibrated:
            if self._stop_condition is not None:
                return
            if self._failed:
                raise RuntimeError(
                    f"Waypoint jogging failed for {self.motion_group_id}: {self._failure_reason}"
                ) from self._failure_exception
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
        """Whether the jogging state stream reports ``RUNNING``; it engages a moment after the
        first waypoint."""
        return self._jog_tracker.last_kind == "RUNNING"

    @property
    def is_at_standstill(self) -> bool:
        """Whether the joints have stopped moving, measured from the state stream; NOVA reports
        no "waypoints ran out" state (executor.md, "Detecting standstill")."""
        return self._standstill.is_settled

    @property
    def standstill_ms(self) -> float:
        """How long the joints have been still [server ms]; ``0.0`` if moving."""
        return self._standstill.standstill_ms

    @property
    def has_failed(self) -> bool:
        return self._failed

    @property
    def stop_condition_triggered(self) -> str | None:
        """Name of the stop condition that ended the session, or ``None``; a normal stop, so
        :attr:`has_failed` stays ``False``."""
        return self._stop_condition

    @property
    def session_elapsed_ms(self) -> int:
        """Session "now" [server ms], extrapolated from the latest acknowledged state without a
        cap (ADR 0012)."""
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
    def scheduled_until_server_ms(self) -> ServerMs:
        """Timestamp of the last step the caller asked for in the latest scheduled chunk; hold
        padding after it is not counted."""
        return ServerMs(self._scheduled_until_server_ms)

    @property
    def scheduled_waypoint_timestamps(self) -> tuple[int, ...]:
        """NOVA timestamps of the waypoints sent in the latest request, trimmed and padded; for
        a caller's step use :meth:`scheduled_timestamp_for_step`."""
        return tuple(self._scheduled_waypoint_timestamps)

    def scheduled_timestamp_for_step(self, step: int) -> ServerMs | None:
        """NOVA timestamp of the caller's ``step`` in the latest chunk, sent or dropped; ``None``
        when it had no such step."""
        if 0 <= step < len(self._scheduled_step_timestamps):
            return ServerMs(self._scheduled_step_timestamps[step])
        return None

    @property
    def scheduled_action_timestep(self) -> int:
        """Policy timestep represented by the latest scheduled request."""
        return self._scheduled_action_timestep

    @property
    def executing_action_timestep(self) -> int:
        """The policy timestep due now, from the latest scheduled chunk that carried timesteps;
        ``-1`` before one. A timestep queue is told this, so it replaces only the steps after."""
        if self._scheduled_start is None:
            return -1
        timestep, start_ms, dt_ms = self._scheduled_start
        elapsed_ms = max(0.0, self._clock.last_server_timestamp_ms - start_ms)
        return timestep + int(elapsed_ms // dt_ms) if dt_ms > 0 else timestep

    @property
    def scheduled_at_server_ms(self) -> ServerMs:
        """Latest server timestamp observed when the request was scheduled."""
        return ServerMs(self._scheduled_at_server_ms)

    @property
    def last_server_timestamp_ms(self) -> ServerMs:
        """Latest raw NOVA jogger-session timestamp from the state stream."""
        return self._clock.last_server_timestamp_ms

    @property
    def estimated_server_timestamp_ms(self) -> ServerMs:
        """Estimated current raw NOVA jogger-session timestamp."""
        return self._clock.estimated_server_timestamp_ms

    @property
    def single_step_dt_ms(self) -> float:
        """Spacing used when a live target is expanded into a short horizon."""
        return self._config.single_step_dt_ms

    @property
    def queued_chunk(self) -> PendingChunk | None:
        """The chunk waiting to be sent, after the pipeline's chunk modules; ``None`` once sent."""
        return self._pending_request

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
        first_timestamp_ms: ServerMs | None = None,
        timestamp_offset_steps: int = 0,
        action_timestep: int = -1,
        extend_buffer: bool = True,
        lead: bool = False,
        observed_at_ms: ServerMs | None = None,
        when_due: tuple[int, Callable[[], Awaitable[None]]] | None = None,
        retained_prefix_steps: int = 0,
    ) -> None:
        """Queue a chunk; the jogging loop stamps it on the session clock and sends it.

        Args:
            steps: Joint waypoints [rad] or TCP poses ``[x, y, z, rx, ry, rz]`` [mm, rad].
            dt_ms: Time between consecutive waypoints [ms]; 0 = single step.
            first_timestamp_ms: Session-clock time of step zero; ``None`` resolves "now" at the
                send.
            timestamp_offset_steps: Shift of step zero in whole ``dt`` steps (``+1`` one ahead).
            action_timestep: Absolute policy timestep of ``steps[0]``; ``-1`` for none.
            extend_buffer: Whether ``HoldPadding`` may pad the chunk.
            lead: Date step ``k`` ``max_expected_latency_ms + k * dt_ms`` after
                ``observed_at_ms``, or after the send when ``None`` (contracts T1); the two
                arguments above are then not used.
            observed_at_ms: Session-clock time of the state the chunk was computed from; for a
                live target, when it was produced (a ``LiveBuffer`` times targets by it).
            when_due: ``(step, action)``: ``action`` runs when the caller's ``step`` is due, at
                once if already past, never if the chunk is replaced before it is sent or the
                session stopped first (contracts T7). Actions run one after another; one that
                raises fails the session.
            retained_prefix_steps: Leading steps a queue already sent, which ``Smoothing``
                keeps exact.
        """
        if not steps:
            return

        chunk = PendingChunk(
            steps=[list(step) for step in steps],
            dt_ms=dt_ms if dt_ms > 0 else self._config.single_step_dt_ms,
            first_timestamp_ms=first_timestamp_ms,
            timestamp_offset_steps=timestamp_offset_steps,
            action_timestep=action_timestep,
            caller_step_count=len(steps),
            lead=lead,
            observed_at_ms=observed_at_ms,
            when_due=when_due,
            extend_buffer=extend_buffer,
            retained_prefix_steps=retained_prefix_steps,
        )
        # Chunk modules see every chunk, before the session keeps only the newest one.
        for module in self._chunk_modules:
            chunk = module.on_chunk(chunk, self._module_context())

        self._queued_chunk_count += 1
        self._pending_request = replace(chunk, sequence=self._queued_chunk_count)
        self._pending_request_event.set()

    async def write_ios(self, ios: Mapping[str, ValueType]) -> None:
        """Write IO values (delegated to IOWriter for deduplication)."""
        await self._io_writer.write(ios)

    async def start(self) -> None:
        """Start the state stream and jogging loop."""
        if self._running:
            msg = f"WaypointJoggingSession for {self.motion_group_id} is already running."
            raise RuntimeError(msg)

        self._running = True
        self._lead_window = []
        self._playback = _PlaybackClock()
        self._pending_request_event.clear()
        # A session can be started again after stop(); the last stop is over.
        self._stop_requested = False
        self._stop_acknowledged = False
        self._stop_settled.clear()

        initial_state = await self._motion_group.get_state()
        self._current_joints = list(initial_state.joints)
        self._current_tcp_pose = initial_state.pose
        self._current_tcp_name = initial_state.tcp
        self._num_joints = len(initial_state.joints)
        clamp = next((m for m in self._chunk_modules if isinstance(m, JointClamp)), None)
        if clamp is not None and self._mode == "joint" and self._joint_limits is None:
            limits = await fetch_joint_limits([self._motion_group], margin=clamp.margin_rad)
            self._joint_limits = limits.get(self.motion_group_id)

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
        """Wait until the server acknowledged :attr:`scheduled_until_server_ms`; with
        ``delay_until_braking_ms`` set, the last target is sent once more to stop on (ADR 0019).

        Returns whether that happened within ``timeout_s``. Does not stop the session; call
        :meth:`stop` afterwards.
        """
        if not self._running or (self._scheduled_until_server_ms <= 0 and not self._pending()):
            return True
        deadline = time.monotonic() + timeout_s
        poll_s = max(self._config.state_rate_ms, 1) / 1000.0
        final_queued = not self._config.delay_until_braking_ms
        while True:
            if not final_queued and not self._pending():
                final_queued = True
                self._queue_final_target()
            # Re-read every pass: the caller's last chunk may still be queued. Compared with
            # acknowledged progress, not the estimate (ADR 0019).
            target_ms = self._scheduled_until_server_ms
            if not self._pending() and self._clock.last_server_timestamp_ms >= target_ms:
                return True
            # After a fault or a fired stop condition the rest of the path is not wanted.
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

    def _queue_final_target(self) -> None:
        """Queue the last waypoint sent again, to stop on it rather than coast past it."""
        if self._last_sent is None or self._failed or self._stop_condition is not None:
            return
        timestamp_ms, target = self._last_sent
        self._queued_chunk_count += 1
        self._pending_request = PendingChunk(
            steps=[target],
            dt_ms=self._config.single_step_dt_ms,
            first_timestamp_ms=ServerMs(timestamp_ms),
            sequence=self._queued_chunk_count,
            final=True,
        )
        self._pending_request_event.set()

    async def stop(self) -> None:
        """Stop the session at once, dropping anything still scheduled; call :meth:`drain` first
        to let the accepted waypoints run out."""
        self._running = False
        for task in self._due_tasks:
            task.cancel()  # not due yet: the motion it belongs to is stopped too
        await self._request_server_stop()

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
                # Only a state after the stop was acknowledged counts (ADR 0019).
                if self._stop_acknowledged and not _jogging_running(state):
                    self._stop_settled.set()
                state_timestamp = getattr(state, "timestamp", None)
                self._clock.observe_state_timestamp(state_timestamp)
                ts_ms = JoggingTimeClock.extract_from_state(state)
                if ts_ms is None:
                    continue
                self._clock.update(ts_ms, state_timestamp)
                self._current_state_server_ms = ts_ms
                # Standstill is timed on the session clock, so only samples carrying it count.
                self._standstill.update(ts_ms, self._current_joints)
                self._notify_state_observer(ts_ms)
        except asyncio.CancelledError:
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
        """Fail the session when the state stream ends before it is ready and calibrated, so
        :meth:`wait_ready` raises the cause; later, a lost stream is left to the jogging loop."""
        if not self._running or self._failed:
            return
        if self._ready.is_set() and self._clock.state_clock_calibrated:
            return
        self._fail(f"{reason} for {self.motion_group_id}", error)

    def _fail(self, reason: str, error: BaseException | None) -> None:
        """Record why the session failed."""
        self._failed = True
        self._failure_reason = reason
        self._failure_exception = error

    def _notify_state_observer(self, ts_ms: int) -> None:
        """Hand this packet to every observer, stamped when it was generated."""
        if not self._state_observers:
            return
        state = self._build_robot_state()
        if state is None:
            return
        # No generation instant before the first server timestamp: use now (ADR 0012).
        generated_at = self._clock.last_sample_wall
        if generated_at is None:
            generated_at = time.monotonic()
        for observer in list(self._state_observers):
            try:
                observer(state, ts_ms, generated_at)
            except (OSError, RuntimeError, ValueError, TypeError, AttributeError) as e:
                # One failing observer (AttributeError: it read a partial state) neither breaks
                # the stream nor skips the others.
                logger.debug("State observer failed for %s: %s", self.motion_group_id, e)

    # -------------------------------------------------------------------------
    # Jogging loop (waypoint mode)
    # -------------------------------------------------------------------------

    async def _request_server_stop(self) -> None:
        """End the session on the server with ``StopActionChunksRequest`` and wait up to
        ``_STOP_TIMEOUT_S`` for it to leave ``RUNNING`` (ADR 0019)."""
        task = self._jogging_task
        if task is None or task.done() or not self._ready.is_set():
            return
        self._stop_requested = True
        self._pending_request_event.set()  # wakes the request generator
        try:
            await asyncio.wait_for(self._stop_settled.wait(), _STOP_TIMEOUT_S)
        except TimeoutError:
            logger.warning(
                "%s: the server did not report the jogging session stopped within %.1f s",
                self.motion_group_id,
                _STOP_TIMEOUT_S,
            )

    async def _consume_jogging_responses(
        self,
        response_stream: AsyncGenerator[api.models.ExecuteActionChunksResponse, None],
    ) -> None:
        """Monitor acknowledgements and failures without gating outgoing chunks."""
        async for response in response_stream:
            if isinstance(response, api.models.StopActionChunksResponse):
                self._stop_acknowledged = True
                continue
            if not self._running:
                if self._stop_requested:
                    continue  # the stream stays open until the stop settles
                return
            if not self._ready.is_set():
                self._ready.set()
            if hasattr(response, "kind") and response.kind == "MOTION_ERROR":
                msg = getattr(response, "message", "unknown motion error")
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
            self._run_when_due(pending, trimmed.step_timestamps)
            if trimmed.request is None:
                self._record_dropped_chunk(pending, trimmed.step_timestamps)
                continue
            request = trimmed.request
            self._scheduled_chunk_count = pending.sequence
            self._scheduled_step_timestamps = list(trimmed.step_timestamps)
            self._scheduled_waypoint_timestamps = [
                waypoint.timestamp for waypoint in request.waypoints
            ]
            # The caller's own last step, not the padded tail — see
            # :attr:`scheduled_until_server_ms`.
            self._scheduled_until_server_ms = trimmed.step_timestamps[pending.last_caller_step]
            self._record_timesteps(pending, trimmed.step_timestamps, trimmed.skipped)
            self._scheduled_at_server_ms = self._clock.last_server_timestamp_ms
            self._record_sent()

            yield request

        if self._stop_requested:
            yield api.models.StopActionChunksRequest()
            # Keep the stream open until the state leaves RUNNING (ADR 0019).
            settled = asyncio.create_task(self._stop_settled.wait())
            _ = await asyncio.wait((settled, response_task), return_when=asyncio.FIRST_COMPLETED)
            settled.cancel()

    def _record_sent(self) -> None:
        """Remember the request being sent, after the waypoint modules: the drain stops on its
        last waypoint, and :meth:`commanded_at` answers from it. Like the server, a request
        replaces what was sent from its first timestamp on."""
        first = self._built[0][0]
        self._commanded = {
            t: v for t, v in self._commanded.items() if first - _COMMAND_HISTORY_MS <= t < first
        }
        self._commanded.update((t, list(v)) for t, v in self._built)
        self._last_sent = (self._built[-1][0], list(self._built[-1][1]))

    def _run_when_due(self, pending: PendingChunk, step_timestamps: list[int]) -> None:
        """Start the chunk's ``when_due`` action for the moment its step is due."""
        if pending.when_due is None or not step_timestamps:
            return
        step, action = pending.when_due
        due_ms = step_timestamps[min(step, len(step_timestamps) - 1)]
        task = asyncio.create_task(self._at(due_ms, action))
        self._due_tasks.add(task)
        task.add_done_callback(self._due_tasks.discard)

    async def _at(self, due_ms: int, action: Callable[[], Awaitable[None]]) -> None:
        delay_s = (due_ms - self._clock.estimated_server_timestamp_ms) / 1000.0
        if delay_s > 0:
            await asyncio.sleep(delay_s)
        if self._failed or self._stop_condition is not None or not self._running:
            return  # the motion it belongs to was stopped
        try:
            async with self._due_lock:  # one chunk's actions after the other's
                await action()
        except Exception as error:  # ruff: ignore[blind-except] (any hook error fails the run)
            self._fail(f"action at a chunk step failed for {self.motion_group_id}", error)

    def _record_dropped_chunk(self, pending: PendingChunk, step_timestamps: list[int]) -> None:
        """Count a chunk dropped as entirely past as scheduled, with its step timestamps; nothing
        was sent, so :attr:`scheduled_until_server_ms` keeps the previous chunk's motion."""
        self._scheduled_chunk_count = pending.sequence
        self._scheduled_step_timestamps = list(step_timestamps)
        self._scheduled_waypoint_timestamps = []
        self._record_timesteps(pending, step_timestamps, 0)
        self._scheduled_at_server_ms = self._clock.last_server_timestamp_ms

    def _record_timesteps(self, pending: PendingChunk, stamps: list[int], skipped: int) -> None:
        """Note which policy timestep the sent request starts with (``-1`` for a bridge's
        steps or a chunk without timesteps) and when the policy's first step is due."""
        timestep, start = pending.action_timestep, pending.start_step
        sent_first = timestep + skipped - start if timestep >= 0 and skipped >= start else -1
        self._scheduled_action_timestep = sent_first
        if timestep >= 0 and start < len(stamps):
            self._scheduled_start = (timestep, ServerMs(stamps[start]), pending.dt_ms)

    def _lead_request(self, pending: PendingChunk) -> _ScheduledRequest:
        """A chunk placed by the lead rule, with the earlier waypoints not yet due.

        Step ``k`` is due ``max_expected_latency_ms + k * dt`` after ``observed_at_ms`` (else
        "now"), one-step chunks spaced by the playback clock (contracts T1, T4, T5). The filters
        drop late steps; ``step_timestamps`` reports every step, sent or not.
        """
        now_ms = self._clock.estimated_server_timestamp_ms
        observed_ms = pending.observed_at_ms
        lead_ms = self._config.max_expected_latency_ms
        first_ms = (now_ms if observed_ms is None else observed_ms) + lead_ms
        if len(pending.steps) == 1:
            # A lagging clock may not eat the controller's planning margin.
            first_ms = self._playback.due_ms(
                first_ms, pending.dt_ms, now_ms + SINGLE_STEP_MARGIN_MS
            )
        else:
            self._playback = _PlaybackClock()
        dt_ms = pending.dt_ms
        stamps = [int(first_ms + k * dt_ms) for k in range(len(pending.steps))]
        kept = self._filtered(stamps, pending)
        if not kept:
            return _ScheduledRequest(request=None, skipped=len(stamps), step_timestamps=stamps)
        sent = [(stamps[i], list(pending.steps[i])) for i in kept]
        earliest_ms = now_ms + MIN_LEAD_MS
        self._lead_window = [
            (t, step) for t, step in self._lead_window if earliest_ms <= t < sent[0][0]
        ] + sent
        request = self._request(self._lead_window, pending)
        return _ScheduledRequest(request=request, skipped=kept[0], step_timestamps=stamps)

    def _trimmed_request(self, pending: PendingChunk) -> _ScheduledRequest:
        """Stamp the chunk (the pipeline's ``Timestamps()``), run the filters, build the request.

        Steps are timed from ``first_timestamp_ms`` shifted by ``timestamp_offset_steps``, else
        from "now" at the send; a final target no earlier than it can be reached (ADR 0019).
        The record has a timestamp for every caller step, sent or filtered out; ``skipped``
        counts the steps before the first one sent.
        """
        if pending.lead:
            return self._lead_request(pending)
        self._lead_window = []
        self._playback = _PlaybackClock()
        stamps = self._timestamps(pending)
        kept = self._filtered(stamps, pending)
        if not kept:
            logger.debug(
                "%s dropping chunk %d: all %d waypoints are in the past",
                self.motion_group_id,
                pending.sequence,
                len(stamps),
            )
            return _ScheduledRequest(request=None, skipped=len(stamps), step_timestamps=stamps)
        request = self._request([(stamps[i], pending.steps[i]) for i in kept], pending)
        return _ScheduledRequest(request=request, skipped=kept[0], step_timestamps=stamps)

    def _timestamps(self, pending: PendingChunk) -> list[int]:
        """Each step's time on the session clock, for a chunk not placed by the lead rule."""
        dt_ms = pending.dt_ms
        base_ms = pending.first_timestamp_ms
        if base_ms is None:
            base_ms = self._clock.estimated_server_timestamp_ms
        anchor_ms = max(0, anchor_timestamp_ms(base_ms, pending.timestamp_offset_steps, dt_ms))
        if pending.final and pending.first_timestamp_ms is not None and dt_ms > 0:
            # Reachable even when the robot is already past its time (ADR 0019).
            earliest_ms = self._clock.estimated_server_timestamp_ms + MIN_LEAD_MS
            anchor_ms = max(anchor_ms, math.ceil(earliest_ms))
        return [anchor_ms + int(i * dt_ms) for i in range(len(pending.steps))]

    def _filtered(self, stamps: list[int], pending: PendingChunk) -> list[int]:
        """The indices of the steps the pipeline's filters keep, in order."""
        kept = list(range(len(stamps)))
        if not self._filters or pending.final:
            return kept  # the final target is where the robot has to stop
        context = self._module_context()
        for module in self._filters:
            waypoints = [(ServerMs(stamps[i]), pending.steps[i]) for i in kept]
            result = module.keep(waypoints, pending, context)
            by_time = {stamps[i]: i for i in kept}
            narrowed = [by_time.get(t) for t, _ in result]
            if None in narrowed or narrowed != sorted(set(narrowed)):  # type: ignore[type-var]
                msg = (
                    f"{type(module).__name__} must keep some of the waypoints it got, in order, "
                    "with their timestamps"
                )
                raise ValueError(msg)
            kept = [i for i in narrowed if i is not None]
        return kept

    def _request(
        self, timed: list[tuple[int, list[float]]], pending: PendingChunk
    ) -> api.models.ActionChunkRequest:
        """The request for these timed waypoints, after the waypoint modules set their values."""
        values = [list(v) for _, v in timed]
        if self._waypoint_modules:
            times = [ServerMs(t) for t, _ in timed]
            context = self._module_context()
            for module in self._waypoint_modules:
                values = module.on_waypoints(list(zip(times, values, strict=True)), context)
                if len(values) != len(timed):
                    msg = (
                        f"{type(module).__name__} returned {len(values)} values for "
                        f"{len(timed)} waypoints"
                    )
                    raise ValueError(msg)
        self._built = [(t, v) for (t, _), v in zip(timed, values, strict=True)]
        return make_timed_waypoints_request(
            self._mode,
            self._built,
            delay_until_braking_ms=0 if pending.final else self._config.delay_until_braking_ms,
        )

    def _module_context(self) -> ModuleContext:
        return ModuleContext(
            now_ms=self._clock.estimated_server_timestamp_ms,
            position=self._position(),
            step_ms=self._config.single_step_dt_ms,
            motion_group_id=self.motion_group_id,
            mode=self._mode,
            joint_limits=self._joint_limits,
            scheduled_start=None if self._scheduled_start is None else self._scheduled_start[:2],
        )

    def _position(self) -> list[float] | None:
        """The latest measured position in the waypoints' form."""
        if self._mode == "joint":
            return None if self._current_joints is None else list(self._current_joints)
        pose = self._current_tcp_pose
        return None if pose is None else [*pose.position, *pose.orientation]

    async def _jogging_loop(self) -> None:
        """Open the action chunk stream and send each pending chunk; responses are consumed in
        their own task, and nothing is sent while no chunk is pending (ADR 0019)."""
        api_gateway = get_api_gateway(self._motion_group)
        cell = get_cell(self._motion_group)
        controller_id = get_controller_id(self._motion_group)
        tcp = await self._resolve_tcp()

        async def client_request_generator(
            response_stream: AsyncGenerator[api.models.ExecuteActionChunksResponse, None],
        ) -> AsyncGenerator[api.models.ExecuteActionChunksRequest, None]:
            # The session clock starts with the first waypoint request, not with this one.
            yield api.models.InitializeActionChunksRequest(
                motion_group=self._motion_group.id, tcp=tcp
            )

            response_task = asyncio.create_task(
                self._consume_jogging_responses(response_stream),
                name=f"wp-jog-responses-{self.motion_group_id}",
            )
            try:
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
            pass  # shutdown, not a failure
        except InvalidStatus as e:
            # 404: no executeActionChunks endpoint (jogging.md, "Error detection").
            if e.response.status_code == _HTTP_NOT_FOUND:
                err = JoggingNotSupportedError(self.motion_group_id)
                self._fail(str(err), err)
                logger.error("%s", err)
            else:
                self._fail(str(e), e)
                logger.error("Jogging connection rejected for %s: %s", self.motion_group_id, e)
            self._running = False
        except MotionError as e:
            self._fail(str(e), e)
            self._running = False
            logger.warning("Waypoint jogging stopped for %s: %s", self.motion_group_id, e)
        except (OSError, RuntimeError) as e:
            if self._running:
                self._fail(str(e), e)
                self._running = False
                logger.error("Jogging connection lost for %s: %s", self.motion_group_id, e)

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
        """Evaluate stop conditions with the current state; the first returning ``True`` is
        recorded and ends the session normally, not as a failure."""
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
