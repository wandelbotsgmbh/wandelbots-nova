"""Waypoint jogging session — sends timestamped position waypoints directly.

Uses NOVA's JointWaypointsRequest and PoseWaypointsRequest to stream action
chunks. The server handles velocity profiling, interpolation, and limits
internally.
"""

from __future__ import annotations

import asyncio
import contextlib
import logging
import time
from typing import TYPE_CHECKING

from websockets.exceptions import InvalidStatus

from nova import api
from nova.types import Pose, RobotState
from novapolicy._sdk import get_api_gateway, get_cell, get_controller_id
from novapolicy.io import IOWriter
from novapolicy.jogging.clock import JoggingTimeClock
from novapolicy.jogging.session import JoggingStateTracker
from novapolicy.jogging.waypoints import PendingChunk, make_waypoints_request
from novapolicy.types import JoggingNotSupportedError, MotionError, StopContext

if TYPE_CHECKING:
    from collections.abc import AsyncGenerator, Mapping

    from nova.cell.motion_group import MotionGroup
    from novapolicy.debug import WaypointTrajectoryTrace
    from novapolicy.types import JoggingMode, StopCondition, ValueType, WaypointConfig

logger = logging.getLogger(__name__)

_HTTP_NOT_FOUND = 404

# Joint gap (deg) between a chunk's first step and the robot's current position
# above which we treat it as a genuine discontinuity worth a WARNING (smaller
# gaps are normal continuous-replacement lag and are logged at DEBUG).
_DISCONTINUITY_WARN_DEG = 10.0


class WaypointJoggingSession:
    """Sends action chunks as timestamped waypoints via the NOVA Jogging API.

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
        trajectory_trace: WaypointTrajectoryTrace | None = None,
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
        self._queued_chunk_count = 0
        self._scheduled_chunk_count = 0
        self._scheduled_until_server_ms = 0
        self._scheduled_waypoint_timestamps: list[int] = []

        self._trajectory_trace = trajectory_trace

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

    async def wait_ready(self, timeout_s: float = 10.0) -> None:
        """Wait until the jogging session is initialized or fail if startup dies."""
        deadline = time.monotonic() + timeout_s
        while not self._ready.is_set():
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
                reason = self._failure_reason or "no ready acknowledgement from NOVA"
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
        """Session "now" anchored on acknowledged server progress (capped).

        Driven by :attr:`JoggingTimeClock.acknowledged_elapsed_ms`, so on a weak
        connection it freezes instead of running ahead of the robot — callers
        that anchor chunks here won't produce a catch-up jump on recovery.
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
        """Final NOVA session timestamp of the latest scheduled chunk."""
        return self._scheduled_until_server_ms

    @property
    def scheduled_waypoint_timestamps(self) -> tuple[int, ...]:
        """NOVA timestamps of the latest scheduled waypoint request."""
        return tuple(self._scheduled_waypoint_timestamps)

    @property
    def last_server_timestamp_ms(self) -> int:
        """Latest raw NOVA jogger-session timestamp from the state stream."""
        return self._clock.last_server_timestamp_ms

    @property
    def estimated_server_timestamp_ms(self) -> int:
        """Estimated current raw NOVA jogger-session timestamp."""
        return self._clock.estimated_server_timestamp_ms

    @property
    def speed_ratio(self) -> float:
        """Auto-computed ratio: server_time / client_time.

        Converges to the real ratio (~1.09 on UR10e) after a few hundred ms.
        Returns 1.0 before server time is available.
        """
        return self._clock.speed_ratio

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
        **_kwargs: object,
    ) -> None:
        """Queue a new action chunk as waypoints.

        Builds a JointWaypointsRequest or PoseWaypointsRequest (based on mode)
        with absolute server-time timestamps laid out as ``base + i*dt``. The
        request is sent on the next jogging loop iteration; the timestamps are
        computed at *yield time* (see :func:`make_waypoints_request`).

        Args:
            steps: Joint waypoints [rad] or TCP poses [x,y,z,rx,ry,rz] (mm/rad).
            dt_ms: Time between consecutive waypoints (ms). 0 = single-step.
            first_timestamp_ms: Exact raw NOVA jogger-session timestamp for
                step zero. When omitted, server "now" is resolved immediately
                before sending so the timestamp cannot become stale in the queue.
            timestamp_offset_steps: Shift the selected timestamp by whole
                ``dt`` steps. ``+1`` places step zero one interval ahead; a
                negative value backdates an overlapping seam; ``0`` is exact.
            server_dt_ms: Exact raw controller-time waypoint spacing. Policy
                queues use this to avoid client-wall clock-rate scaling.
            action_timestep: Absolute policy timestep represented by ``steps[0]``.
                Used only by the optional trajectory diagnostic trace.
        """
        if not steps:
            return

        # Single-step target: use a default step time
        effective_dt_ms = dt_ms if dt_ms > 0 else 100.0

        # Cap how far the session clock may run ahead of acknowledged server
        # time at this chunk's horizon, so a stalled link drifts at most one
        # lookahead window before "now" freezes (see acknowledged_elapsed_ms).
        self._clock.max_lookahead_ms = len(steps) * effective_dt_ms

        # Store raw chunk data. Timestamps are computed in _jogging_loop
        # immediately before yielding to the server, avoiding drift from any
        # internal await/scheduling delay between policy and stream send.
        self._queued_chunk_count += 1
        self._pending_request = PendingChunk(
            steps=steps,
            dt_ms=effective_dt_ms,
            first_timestamp_ms=first_timestamp_ms,
            timestamp_offset_steps=timestamp_offset_steps,
            server_dt_ms=server_dt_ms,
            action_timestep=action_timestep,
            sequence=self._queued_chunk_count,
        )

        # Compare current robot position vs chunk first step (joint mode only).
        # Skipped for backdated overlapping chunks: there the robot always
        # lags the freshest prediction's step 0 by a few degrees, which is
        # expected and not worth reporting. For exact/ahead chunks a large
        # first-step gap is a genuine discontinuity worth a WARNING.
        if (
            timestamp_offset_steps >= 0
            and self._mode == "joint"
            and self._current_joints is not None
            and len(steps) > 0
        ):
            delta = [
                abs(steps[0][j] - self._current_joints[j]) for j in range(min(3, len(steps[0])))
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
                steps[0][0],
                steps[0][1],
                steps[0][2],
            )

        # Request is built later at yield time.

    async def write_ios(self, ios: Mapping[str, ValueType]) -> None:
        """Write IO values (delegated to IOWriter for deduplication)."""
        await self._io_writer.write(ios)

    async def start(self) -> None:
        """Start the state stream and jogging loop."""
        if self._running:
            msg = f"WaypointJoggingSession for {self.motion_group_id} is already running."
            raise RuntimeError(msg)

        self._running = True

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

    async def stop(self) -> None:
        """Stop the session gracefully."""
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
        try:
            stream = self._motion_group.stream_state(response_rate_msecs=self._config.state_rate_ms)
            async for state in stream:
                self._current_joints = list(state.joint_position)
                if state.tcp_pose is not None:
                    self._current_tcp_pose = Pose(state.tcp_pose)
                if state.tcp is not None:
                    self._current_tcp_name = state.tcp
                # Extract server jogger session timestamp for time synchronization
                ts_ms = JoggingTimeClock.extract_from_state(state)
                if ts_ms is not None:
                    self._clock.update(ts_ms)
                    if self._trajectory_trace is not None:
                        self._trajectory_trace.record_state(
                            server_timestamp_ms=ts_ms,
                            joints=list(self._current_joints),
                            tcp=self._current_tcp_pose,
                        )
                    self._measure_waypoint_tracking(ts_ms)
                self._jog_tracker.update_from_state(state)
        except asyncio.CancelledError:
            # Expected on shutdown; stop quietly without logging as an error.
            pass
        except (OSError, RuntimeError) as e:
            logger.error("State stream error for %s: %s", self.motion_group_id, e)
        finally:
            if stream is not None:
                with contextlib.suppress(asyncio.CancelledError, OSError, RuntimeError):
                    await stream.aclose()

    # -------------------------------------------------------------------------
    # Jogging loop (waypoint mode)
    # -------------------------------------------------------------------------

    async def _jogging_loop(self) -> None:  # noqa: C901, PLR0915
        """Open jogging session and send waypoints when available.

        Uses the bidirectional stream's ping-pong pattern, but between chunks
        we sleep instead of sending hold/zero-velocity messages that could
        confuse the server's motion planner.
        """
        api_gateway = get_api_gateway(self._motion_group)
        cell = get_cell(self._motion_group)
        controller_id = get_controller_id(self._motion_group)
        tcp = await self._resolve_tcp()

        async def client_request_generator(
            response_stream: AsyncGenerator[api.models.ExecuteWaypointJoggingResponse, None],
        ) -> AsyncGenerator[api.models.ExecuteWaypointJoggingRequest, None]:
            # 1. Initialize the jogging session.
            # The server starts its internal timer when the first waypoint
            # request arrives (not on InitializeJoggingRequest).
            yield api.models.ExecuteWaypointJoggingRequest(
                api.models.InitializeJoggingRequest(motion_group=self._motion_group.id, tcp=tcp)
            )

            # 2. Main loop: for each server response, either send a new
            #    chunk (if ready) or sleep until one is ready.
            first_chunk = True
            async for response in response_stream:
                if not self._running:
                    return

                # Signal that the session is ready on first server response.
                # This means InitializeJoggingRequest was acknowledged and
                # the server is ready to accept waypoints.
                if not self._ready.is_set():
                    self._ready.set()

                if hasattr(response.root, "kind") and response.root.kind == "MOTION_ERROR":
                    msg = getattr(response.root, "message", "unknown motion error")
                    raise MotionError(self.motion_group_id, msg)

                self._check_stop_conditions()
                self._jog_tracker.check()

                # Wait until a new chunk is available (sleep, don't send junk)
                while self._pending_request is None and self._running:
                    await asyncio.sleep(0.001)

                if not self._running:
                    return

                # Send the chunk. Capture and clear BEFORE yielding: while this
                # async generator is suspended at yield, the executor may write
                # the next _pending_request. Clearing after yield would delete
                # that fresh chunk and create gaps that let the server exhaust
                # its current action chunk.
                pending = self._pending_request
                self._pending_request = None
                if pending is None:
                    continue

                # Start the clock on the first chunk — this is when the
                # server's internal timer begins (first waypoint message).
                if first_chunk:
                    self._clock.start()
                    first_chunk = False

                # Build the request now so timestamps are aligned to the server
                # session timer at the last possible moment.
                request = make_waypoints_request(
                    self._clock,
                    self._mode,
                    steps=pending.steps,
                    effective_dt_ms=pending.dt_ms,
                    first_timestamp_ms=pending.first_timestamp_ms,
                    timestamp_offset_steps=pending.timestamp_offset_steps,
                    server_dt_ms=pending.server_dt_ms,
                )
                self._log_waypoint_timing(request)
                self._scheduled_chunk_count = pending.sequence
                self._scheduled_waypoint_timestamps = [
                    waypoint.timestamp for waypoint in request.waypoints
                ]
                self._scheduled_until_server_ms = self._scheduled_waypoint_timestamps[-1]
                if self._trajectory_trace is not None:
                    self._trajectory_trace.record_request(
                        sequence=pending.sequence,
                        action_timestep=pending.action_timestep,
                        policy_dt_ms=pending.dt_ms,
                        first_timestamp_ms=pending.first_timestamp_ms,
                        timestamp_offset_steps=pending.timestamp_offset_steps,
                        server_dt_ms=pending.server_dt_ms,
                        server_sample_ms=self._clock.last_server_timestamp_ms,
                        timestamps_ms=list(self._scheduled_waypoint_timestamps),
                        steps=pending.steps,
                    )
                self._start_waypoint_tracking_measurement(request)

                yield api.models.ExecuteWaypointJoggingRequest(request)

        try:
            await api_gateway.jogging_api.execute_waypoint_jogging(
                cell=cell,
                controller=controller_id,
                client_request_generator=client_request_generator,
            )
        except asyncio.CancelledError:
            # Expected on shutdown; cancellation is not a jogging failure.
            pass
        except InvalidStatus as e:
            # An old api-gateway (< 26.5) has no executeWaypointJogging endpoint
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
            (waypoint.timestamp, list(waypoint.joints.root)) for waypoint in waypoints
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
