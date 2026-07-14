"""Waypoint jogging session — sends timestamped position waypoints directly.

Uses NOVA's action-chunk-streaming API (``ActionChunkRequest`` with unified
joint/pose waypoints) to stream action chunks. The server handles velocity
profiling, interpolation, and limits internally.
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
from novapolicy.jogging.waypoints import NOW, PendingChunk, make_waypoints_request
from novapolicy.types import JoggingNotSupportedError, MotionError, StopContext

if TYPE_CHECKING:
    from collections.abc import AsyncGenerator

    from nova.cell.motion_group import MotionGroup
    from novapolicy.types import JoggingMode, StopCondition, ValueType, WaypointConfig

logger = logging.getLogger(__name__)

_HTTP_NOT_FOUND = 404

# Joint gap (deg) between a chunk's first step and the robot's current position
# above which we treat it as a genuine discontinuity worth a WARNING (smaller
# gaps are normal RTC tracking lag and are logged at DEBUG).
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

    async def wait_ready(self) -> None:
        """Wait until the jogging session is fully initialized and ready for waypoints."""
        await self._ready.wait()

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
        anchor_ms: int = NOW,
        anchor_offset_steps: int = 0,
        **_kwargs: object,
    ) -> None:
        """Queue a new action chunk as waypoints.

        Builds an ActionChunkRequest (joint or pose waypoints, based on mode)
        with absolute server-time timestamps laid out as ``base + i*dt``. The
        request is sent on the next jogging loop iteration; the timestamps are
        computed at *yield time* (see :func:`make_waypoints_request`).

        Args:
            steps: Joint waypoints [rad] or TCP poses [x,y,z,rx,ry,rz] (mm/rad).
            dt_ms: Time between consecutive waypoints (ms). 0 = single-step.
            anchor_ms: Where step 0 sits on the server timeline. ``NOW`` (default)
                resolves "now" at yield time so the anchor cannot go stale while
                the chunk waits in the queue; ``>= 0`` is an explicit absolute
                anchor (replay / scheduled segments), used verbatim.
            anchor_offset_steps: Shift the anchor by whole ``dt`` steps. ``+1``
                places step 0 one dt ahead (live single targets); a negative
                value backdates the anchor so an already-passed step lands at
                "now" (RTC seam stitching); ``0`` anchors exactly.
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
        self._pending_request = PendingChunk(
            steps=steps,
            dt_ms=effective_dt_ms,
            anchor_ms=anchor_ms,
            anchor_offset_steps=anchor_offset_steps,
        )

        # Compare current robot position vs chunk first step (joint mode only).
        # Skipped for backdated (overlapping RTC) chunks: there the robot always
        # lags the freshest prediction's step 0 by a few degrees, which is
        # expected and not worth reporting. For exact/ahead chunks a large
        # first-step gap is a genuine discontinuity worth a WARNING.
        if (
            anchor_offset_steps >= 0
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

    async def write_ios(self, ios: dict[str, ValueType]) -> None:
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
            response_stream: AsyncGenerator[api.models.ExecuteActionChunksResponse, None],
        ) -> AsyncGenerator[api.models.ExecuteActionChunksRequest, None]:
            # 1. Initialize the action-chunk-streaming session.
            # The server starts its internal timer when the first waypoint
            # request arrives (not on InitializeActionChunksRequest).
            yield api.models.ExecuteActionChunksRequest(
                api.models.InitializeActionChunksRequest(
                    motion_group=self._motion_group.id, tcp=tcp
                )
            )

            # 2. Main loop: for each server response, either send a new
            #    chunk (if ready) or sleep until one is ready.
            first_chunk = True
            async for response in response_stream:
                if not self._running:
                    return

                # Signal that the session is ready on first server response.
                # This means InitializeActionChunksRequest was acknowledged and
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
                    anchor_ms=pending.anchor_ms,
                    anchor_offset_steps=pending.anchor_offset_steps,
                )

                yield api.models.ExecuteActionChunksRequest(request)

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
