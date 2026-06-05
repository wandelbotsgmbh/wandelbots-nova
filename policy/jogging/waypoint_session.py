"""Waypoint jogging session — sends timestamped position waypoints directly.

Uses NOVA's JointWaypointsRequest and PoseWaypointsRequest to stream action
chunks. The server handles velocity profiling, interpolation, and limits
internally.
"""

from __future__ import annotations

import asyncio
import contextlib
from dataclasses import dataclass, field
import logging
import time
from typing import TYPE_CHECKING

from nova import api
from nova.types import Pose, RobotState
from policy._sdk import get_api_gateway, get_cell, get_controller_id
from policy.io import IOWriter
from policy.jogging.session import JoggingStateTracker
from policy.types import GuardState, GuardStopError, MotionError

if TYPE_CHECKING:
    from collections.abc import AsyncGenerator

    from nova.cell.motion_group import MotionGroup
    from policy.types import JoggingMode, MotionConfig, SafetyGuard, ValueType

logger = logging.getLogger(__name__)


@dataclass
class JoggingTimeClock:
    """Tracks the server's jogger session clock and computes the speed ratio.

    The server exposes ``jogger_session_timestamp_ms`` in the state stream
    (field on ``JoggingDetails``). It starts at 0 after ``InitializeJoggingRequest``
    and increments while waypoints are being executed.

    This class observes that timestamp, compares it to the client's wall-clock
    elapsed time, and derives the speed ratio (server_time / client_time).
    The ratio is used to scale outgoing waypoint timestamps so that the
    robot moves at real-time speed regardless of the server's internal rate.
    """

    speed_ratio: float = 1.0
    synced: bool = False
    _client_start_time: float = field(default=0.0, repr=False)

    def start(self) -> None:
        """Mark the client-side session start time."""
        self._client_start_time = time.monotonic()

    @property
    def client_elapsed_ms(self) -> int:
        """Client wall-clock elapsed since session start."""
        if self._client_start_time == 0.0:
            return 0
        return int((time.monotonic() - self._client_start_time) * 1000)

    def update(self, timestamp_ms: int) -> None:
        """Feed a new ``jogger_session_timestamp_ms`` reading from the state stream."""
        if timestamp_ms <= 0:
            return
        if not self.synced:
            self.synced = True
            logger.info(
                "Server time sync established (jogger_session_timestamp_ms=%d)", timestamp_ms
            )
        # Use raw ratio (server_time / client_time) directly.
        # Clamp >= 1.0 since the server is never slower than wall-clock.
        client_ms = self.client_elapsed_ms
        if client_ms > 0:
            self.speed_ratio = max(1.0, timestamp_ms / client_ms)

    def scale_timestamp(self, trajectory_time_ms: int) -> int:
        """Convert a trajectory-time timestamp to server-time."""
        return int(trajectory_time_ms * self.speed_ratio)

    def scale_dt(self, dt_ms: float) -> float:
        """Convert a trajectory-time dt to server-time."""
        return dt_ms * self.speed_ratio

    @staticmethod
    def extract_from_state(state: object) -> int | None:
        """Extract jogger_session_timestamp_ms from a MotionGroupState, or None."""
        execute = getattr(state, "execute", None)
        if execute is None:
            return None
        details = getattr(execute, "details", None)
        if details is None:
            return None
        ts = getattr(details, "jogger_session_timestamp_ms", None)
        if isinstance(ts, int):
            return ts
        return None


def is_waypoint_jogging_available() -> bool:
    """Check if the NOVA SDK has the JointWaypointsRequest model.

    Returns False on older SDK versions that don't support waypoint jogging.
    """
    return hasattr(api.models, "JointWaypointsRequest")


def is_pose_waypoint_jogging_available() -> bool:
    """Check if the NOVA SDK has the PoseWaypointsRequest model.

    Returns False on older SDK versions that don't support Cartesian waypoint jogging.
    """
    return hasattr(api.models, "PoseWaypointsRequest")


class WaypointJoggingSession:
    """Sends action chunks as timestamped waypoints via the NOVA Jogging API.

    Sends raw position waypoints (joint or TCP) with timing info.
    The server computes the motion profile and handles IK (for TCP mode).
    """

    def __init__(
        self,
        motion_group: MotionGroup,
        config: MotionConfig,
        *,
        tcp: str = "",
        mode: JoggingMode = "joint",
        safety_guards: list[SafetyGuard] | None = None,
    ) -> None:
        if not is_waypoint_jogging_available():
            msg = (
                "JointWaypointsRequest not available in this NOVA SDK version. "
                "Use MotionConfig (velocity profile) instead, or upgrade NOVA."
            )
            raise RuntimeError(msg)

        if mode == "cartesian" and not is_pose_waypoint_jogging_available():
            msg = (
                "PoseWaypointsRequest not available in this NOVA SDK version. "
                "Cartesian waypoint jogging requires NOVA >= 26.5. "
                "Use joint mode or upgrade NOVA."
            )
            raise RuntimeError(msg)

        self._motion_group = motion_group
        self._config = config
        self._tcp = tcp
        self._mode: JoggingMode = mode
        self._safety_guards = safety_guards or []
        self._io_values: dict[str, object] | None = None
        self._io_writer = IOWriter(motion_group)
        self._jog_tracker = JoggingStateTracker(motion_group.id)

        # Current robot state (updated by state stream)
        self._current_joints: list[float] | None = None
        self._current_joint_torques: list[float] | None = None
        self._current_joint_currents: list[float] | None = None
        self._current_tcp_pose: Pose | None = None
        self._current_tcp_name: str | None = None
        self._num_joints: int | None = None

        # Safety guard state
        self._prev_state: RobotState | None = None
        self._prev_tick_time: float | None = None

        # Server time synchronization: auto-computes the speed ratio between
        # server clock and client wall-clock, then scales outgoing timestamps
        # so the robot moves at real-time speed.
        self._clock = JoggingTimeClock()

        # Pending waypoints to send (set by update_chunk, consumed by jogging loop).
        # For normal chunks, store raw steps/timing and build the request at
        # yield-time so timestamps are computed as late as possible.
        self._pending_request: object | None = None

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
    def current_state(self) -> RobotState | None:
        if self._current_joints is None or self._current_tcp_pose is None:
            return None
        return self._build_robot_state()

    @property
    def is_running(self) -> bool:
        return self._running

    @property
    def is_ready(self) -> bool:
        """True when the server has acknowledged the session and is ready for waypoints."""
        return self._ready.is_set()

    async def wait_ready(self) -> None:
        """Wait until the jogging session is fully initialized and ready for waypoints."""
        await self._ready.wait()

    @property
    def has_failed(self) -> bool:
        return self._failed

    @property
    def session_elapsed_ms(self) -> int:
        """Elapsed milliseconds on the client-side jogger session clock."""
        return self._clock.client_elapsed_ms

    @property
    def speed_ratio(self) -> float:
        """Auto-computed ratio: server_time / client_time.

        Converges to the real ratio (~1.09 on UR10e) after a few hundred ms.
        Returns 1.0 before server time is available.
        """
        return self._clock.speed_ratio

    @property
    def chunk_done(self) -> bool:
        """Always True — waypoint mode uses fixed-rate policy calls."""
        return True

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
        start_time_ms: int = -1,
        **_kwargs: object,
    ) -> None:
        """Queue a new action chunk as waypoints.

        Builds a JointWaypointsRequest or PoseWaypointsRequest (based on mode)
        with timestamps computed from dt_ms.
        The request is sent on the next jogging loop iteration.

        Args:
            steps: Joint waypoints [rad] or TCP poses [x,y,z,rx,ry,rz] (mm/rad).
            dt_ms: Time between consecutive waypoints (ms). 0 = single-step.
            start_time_ms: Absolute timestamp (ms from session start) for the
                first waypoint. When >=0, timestamps are trajectory-absolute:
                [start_time_ms + dt, start_time_ms + 2*dt, ...].
                When -1 (default), uses current session time + dt (legacy).

        Using trajectory-absolute timestamps (start_time_ms >= 0) is recommended
        for overlapping chunks: timestamps will be "in the past" by the time
        the server receives them, which lets the server interpolate smoothly
        from the current time forward without replanning jitter.
        """
        if not steps:
            return

        # Single-step target: use a default step time
        effective_dt_ms = dt_ms if dt_ms > 0 else 100.0

        # Store raw chunk data. Timestamps are computed in _jogging_loop
        # immediately before yielding to the server, avoiding drift from any
        # internal await/scheduling delay between policy and stream send.
        self._pending_request = ("chunk", steps, effective_dt_ms, start_time_ms)

        # Debug: log current robot position vs chunk first step (joint mode only)
        if self._mode == "joint" and self._current_joints is not None and len(steps) > 0:
            delta = [
                abs(steps[0][j] - self._current_joints[j]) for j in range(min(3, len(steps[0])))
            ]
            max_delta = max(delta) * 57.3
            if max_delta > 1.0:  # only log if > 1 degree
                logger.warning(
                    "%s: chunk first step is %.1f deg from current position! "
                    "current=[%.4f,%.4f,%.4f] chunk_first=[%.4f,%.4f,%.4f]",
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

    def _make_waypoints_request(
        self,
        *,
        steps: list[list[float]],
        effective_dt_ms: float,
        start_time_ms: int,
    ) -> object:
        """Build JointWaypointsRequest or PoseWaypointsRequest at stream-yield time.

        Scales the policy's real-time timestamps to server-time using the
        auto-computed speed ratio from ``JoggingTimeClock``.

        When ``start_time_ms >= 0`` (trajectory-absolute mode), timestamps are
        placed at [start_time_ms * ratio, ...] on the server timeline.

        When ``start_time_ms == -1`` (legacy mode), timestamps start from
        the current client elapsed time scaled to server time.

        In cartesian mode (``self._mode == 'cartesian'``), steps are
        [x, y, z, rx, ry, rz] and are sent as PoseWaypointsRequest.
        In joint mode, steps are joint radians sent as JointWaypointsRequest.
        """
        client_now_ms = self._clock.client_elapsed_ms

        # Scale timestamps by auto-computed speed ratio so the server
        # receives timestamps aligned with its faster internal clock.
        # The policy sends in "real time"; we convert to "server time".
        scaled_dt_ms = self._clock.scale_dt(effective_dt_ms)

        if start_time_ms >= 0:
            base_ms = self._clock.scale_timestamp(start_time_ms)
            timestamps = [base_ms + int(i * scaled_dt_ms) for i in range(len(steps))]
        else:
            server_now_ms = self._clock.scale_timestamp(client_now_ms)
            timestamps = [server_now_ms + int((i + 1) * scaled_dt_ms) for i in range(len(steps))]

        if self._mode == "cartesian":
            return self._build_pose_request(timestamps, steps)
        return self._build_joint_request(timestamps, steps)

    @staticmethod
    def _build_joint_request(
        timestamps: list[int],
        steps: list[list[float]],
    ) -> object:
        """Build a JointWaypointsRequest from timestamps and joint steps.

        The request uses the array-of-structs layout: a single ``waypoints``
        list where each ``JointWaypoint`` bundles its timestamp with its joints.
        """
        return api.models.JointWaypointsRequest(
            waypoints=[
                api.models.JointWaypoint(timestamp=ts, joints=api.models.Joints(root=step))
                for ts, step in zip(timestamps, steps, strict=True)
            ],
        )

    @staticmethod
    def _build_pose_request(
        timestamps: list[int],
        steps: list[list[float]],
    ) -> object:
        """Build a PoseWaypointsRequest from timestamps and TCP pose steps.

        Each step is [x, y, z, rx, ry, rz] where position is in mm and
        orientation is a rotation vector in radians.
        """
        from wandelbots_api_client.v2_pydantic.models.models import (  # noqa: PLC0415
            Pose as ApiPose,
            RotationVector,
            Vector3d,
        )

        waypoints = []
        for ts, step in zip(timestamps, steps, strict=True):
            # step = [x, y, z, rx, ry, rz]
            pos = Vector3d(root=list(step[:3]))
            orient = RotationVector(root=list(step[3:6]))
            waypoints.append(
                api.models.PoseWaypoint(
                    timestamp=ts, pose=ApiPose(position=pos, orientation=orient)
                )
            )

        return api.models.PoseWaypointsRequest(waypoints=waypoints)

    async def start(self) -> None:
        """Start the state stream and jogging loop."""
        if self._running:
            msg = f"WaypointJoggingSession for {self.motion_group_id} is already running."
            raise RuntimeError(msg)

        self._running = True

        initial_state = await self._motion_group.get_state()
        self._current_joints = list(initial_state.joints)
        self._current_joint_torques = (
            list(initial_state.joint_torques)
            if getattr(initial_state, "joint_torques", None) is not None
            else None
        )
        self._current_joint_currents = (
            list(initial_state.joint_currents)
            if getattr(initial_state, "joint_currents", None) is not None
            else None
        )
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
                self._current_joint_torques = (
                    list(state.joint_torque.root)
                    if getattr(state, "joint_torque", None) is not None
                    else None
                )
                self._current_joint_currents = (
                    list(state.joint_current.root)
                    if getattr(state, "joint_current", None) is not None
                    else None
                )
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

    async def _jogging_loop(self) -> None:  # noqa: C901
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
            response_stream: AsyncGenerator[api.models.ExecuteJoggingResponse, None],
        ) -> AsyncGenerator[api.models.ExecuteJoggingRequest, None]:
            # 1. Initialize the jogging session.
            # The server starts its internal timer when the first waypoint
            # request arrives (not on InitializeJoggingRequest).
            yield api.models.ExecuteJoggingRequest(
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

                self._run_guards()
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
                request = self._pending_request
                self._pending_request = None

                # If this is a raw action chunk, build the JointWaypointsRequest
                # now so timestamps are aligned to the server session timer at
                # the last possible moment.
                if isinstance(request, tuple) and request and request[0] == "chunk":
                    # Start the clock on the first chunk — this is when the
                    # server's internal timer begins (first waypoint message).
                    if first_chunk:
                        self._clock.start()
                        first_chunk = False
                    _, steps, effective_dt_ms, start_time_ms = request
                    request = self._make_waypoints_request(
                        steps=steps,
                        effective_dt_ms=effective_dt_ms,
                        start_time_ms=start_time_ms,
                    )

                yield api.models.ExecuteJoggingRequest(request)

        try:
            await api_gateway.jogging_api.execute_jogging(
                cell=cell,
                controller=controller_id,
                client_request_generator=client_request_generator,
            )
        except asyncio.CancelledError:
            pass
        except (GuardStopError, MotionError) as e:
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

    def _run_guards(self) -> None:
        """Run safety guards with current state."""
        if not self._safety_guards:
            return

        current_state = self._build_robot_state()
        if current_state is None:
            return

        now = time.monotonic()
        dt = now - self._prev_tick_time if self._prev_tick_time is not None else 0.01

        ctx = GuardState(
            state=current_state,
            prev_state=self._prev_state,
            dt=dt,
            motion_group_id=self.motion_group_id,
            io_values=self._io_values,
        )
        for guard in self._safety_guards:
            if not guard(ctx):
                guard_name = getattr(guard, "__name__", repr(guard))
                self._running = False
                raise GuardStopError(self.motion_group_id, guard_name)

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
            joint_torques=(
                tuple(self._current_joint_torques)
                if self._current_joint_torques is not None
                else None
            ),
            joint_currents=(
                tuple(self._current_joint_currents)
                if self._current_joint_currents is not None
                else None
            ),
        )
