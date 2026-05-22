"""Waypoint jogging session — sends timestamped position waypoints directly.

Uses NOVA's native JointWaypointsRequest to stream action chunks.
The server handles velocity profiling, interpolation, and limits internally.

This is the preferred motion mode when the NOVA instance supports it.
Falls back to the client-side velocity profile (JoggingSession) on older
instances that don't have the waypoint jogging API.

.. note:: This API is experimental and not yet published in stable NOVA releases.
   Import this module conditionally and handle ImportError/AttributeError.

.. todo:: Remove the ``KIND_UNKNOWN`` monkey-patch in
   ``.venv/.../wandelbots_api_client/v2_pydantic/models/models.py`` once the
   stable ``wandelbots_api_client`` release includes the updated
   ``JoggingDetails.state`` discriminator (tracking: service-manager MR !2345).
"""

from __future__ import annotations

import asyncio
import contextlib
import json
import logging
from pathlib import Path
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


def is_waypoint_jogging_available() -> bool:
    """Check if the NOVA SDK has the JointWaypointsRequest model.

    Returns False on older SDK versions that don't support waypoint jogging.
    """
    return hasattr(api.models, "JointWaypointsRequest")


class WaypointJoggingSession:
    """Sends action chunks as timestamped waypoints via the NOVA Jogging API.

    Instead of computing velocities client-side, this sends raw position
    waypoints with timing info. The server computes the motion profile.

    Has the same interface as JoggingSession so the executor can use either.
    """

    def __init__(
        self,
        motion_group: MotionGroup,
        config: MotionConfig,
        *,
        tcp: str = "",
        mode: JoggingMode = "joint",
        safety_guards: list[SafetyGuard] | None = None,
        server_speed_ratio: float = 1.0,
    ) -> None:
        if not is_waypoint_jogging_available():
            msg = (
                "JointWaypointsRequest not available in this NOVA SDK version. "
                "Use MotionConfig (velocity profile) instead, or upgrade NOVA."
            )
            raise RuntimeError(msg)

        self._motion_group = motion_group
        self._config = config
        self._tcp = tcp
        self._mode: JoggingMode = mode
        self._safety_guards = safety_guards or []
        self._server_speed_ratio = server_speed_ratio
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

        # Chunk state
        self._session_start_time: float = 0.0
        self._external_start_time: float = 0.0
        self._action_time_offset_ms: int | None = None
        self._chunk_started: bool = False  # True after first chunk is sent
        self._standstill: bool = True  # Updated by state stream

        # Position stability tracking for chunk_done
        self._prev_position: list[float] | None = None
        self._stable_count: int = 0
        self._stable_threshold = 0.001  # rad — below this = not moving
        self._stable_required = 5  # consecutive stable readings
        self._last_target: list[float] | None = None

        # Debug log file for jogger commands
        self._log_file: Path | None = None
        self._log_fh: object | None = None
        self._cmd_seq: int = 0  # command sequence counter

        # Pending waypoints to send (set by update_chunk, consumed by jogging loop).
        # For normal chunks, store raw steps/timing and build the request at
        # yield-time so timestamps are computed as late as possible.
        self._pending_request: object | None = None

        # Task management
        self._jogging_task: asyncio.Task[None] | None = None
        self._state_task: asyncio.Task[None] | None = None
        self._running = False
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
    def has_failed(self) -> bool:
        return self._failed

    @property
    def session_elapsed_ms(self) -> int:
        """Elapsed milliseconds on the client-side jogger session clock."""
        if self._session_start_time == 0.0:
            return 0
        return int((time.monotonic() - self._session_start_time) * 1000)

    @property
    def chunk_done(self) -> bool:
        """Always True — the executor uses fixed-rate timing for waypoint mode."""
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

        Builds a JointWaypointsRequest with timestamps computed from dt_ms.
        The request is sent on the next jogging loop iteration.

        Args:
            steps: Joint waypoints to send.
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

        self._chunk_started = True
        self._standstill = False  # assume robot will move
        self._stable_count = 0  # reset stability counter

        # Single-step target: use a default step time
        effective_dt_ms = dt_ms if dt_ms > 0 else 100.0

        # Store raw chunk data. Timestamps are computed in _jogging_loop
        # immediately before yielding to the server, avoiding drift from any
        # internal await/scheduling delay between policy and stream send.
        self._pending_request = ("chunk", steps, effective_dt_ms, start_time_ms)

        # Debug: log current robot position vs chunk first step
        if self._current_joints is not None and len(steps) > 0:
            delta = [abs(steps[0][j] - self._current_joints[j]) for j in range(min(3, len(steps[0])))]
            max_delta = max(delta) * 57.3
            if max_delta > 1.0:  # only log if > 1 degree
                logger.warning(
                    "%s: chunk first step is %.1f deg from current position! "
                    "current=[%.4f,%.4f,%.4f] chunk_first=[%.4f,%.4f,%.4f]",
                    self.motion_group_id, max_delta,
                    self._current_joints[0], self._current_joints[1], self._current_joints[2],
                    steps[0][0], steps[0][1], steps[0][2],
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
        """Build JointWaypointsRequest at stream-yield time.

        The server starts its timer when the jogging session opens. When
        ``start_time_ms >= 0``, it is already a timestamp on that server-session
        timeline (milliseconds since jogger start), so no policy-local clock
        translation is applied here.

        If ``server_speed_ratio != 1.0``, both the base timestamp and dt are
        stretched to compensate for the server consuming waypoints faster
        than wall clock. The policy sends in "trajectory time"; this method
        converts to "server time".
        """
        now_ms = int((time.monotonic() - self._session_start_time) * 1000)

        # Scale both base and dt by the server speed ratio
        ratio = self._server_speed_ratio
        scaled_dt_ms = effective_dt_ms * ratio

        if start_time_ms >= 0:
            base_ms = int(start_time_ms * ratio)
            timestamps = [base_ms + int(i * scaled_dt_ms) for i in range(len(steps))]
        else:
            base_ms = now_ms
            timestamps = [base_ms + int((i + 1) * scaled_dt_ms) for i in range(len(steps))]

        self._log_chunk(now_ms, timestamps, steps)
        return api.models.JointWaypointsRequest(
            timestamps=timestamps,
            joint_waypoints=[api.models.Joints(root=step) for step in steps],
        )

    def send_raw_waypoints(
        self,
        timestamps: list[int],
        waypoints: list[list[float]],
    ) -> None:
        """Send a pre-built JointWaypointsRequest with explicit timestamps.

        Unlike update_chunk() which computes timestamps from the current clock,
        this method uses the provided timestamps directly. Useful for replay
        tests where you want exact control over timing.
        """
        self._chunk_started = True
        self._pending_request = api.models.JointWaypointsRequest(
            timestamps=timestamps,
            joint_waypoints=[api.models.Joints(root=wp) for wp in waypoints],
        )
        self._log_chunk(
            int((time.monotonic() - self._session_start_time) * 1000),
            timestamps,
            waypoints,
        )

    def enable_logging(self, path: Path | str) -> None:
        """Enable file-based logging of all jogger commands.

        Creates a JSONL file recording every request sent to the jogger API,
        including timestamps, waypoints, hold requests, and state updates.
        Call before start().
        """
        self._log_file = Path(path)
        self._log_file.parent.mkdir(parents=True, exist_ok=True)

    async def start(self) -> None:
        """Start the state stream and jogging loop."""
        if self._running:
            msg = (
                f"WaypointJoggingSession for {self.motion_group_id} is already running."
            )
            raise RuntimeError(msg)

        self._running = True
        self._session_start_time = time.monotonic()

        # Open log file if enabled
        if self._log_file is not None:
            self._log_fh = self._log_file.open("w")
            self._write_log("session_start", {
                "motion_group": self.motion_group_id,
                "session_start_time": self._session_start_time,
            })

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

        # Close log file
        if self._log_fh is not None:
            self._write_log("session_stop", {
                "duration_s": time.monotonic() - self._session_start_time,
                "total_cmds": self._cmd_seq,
            })
            self._log_fh.close()  # type: ignore[union-attr]
            self._log_fh = None
            logger.info(
                "Jogger command log saved to %s (%d entries)",
                self._log_file, self._cmd_seq,
            )

        logger.info("WaypointJoggingSession stopped for %s", self.motion_group_id)

    # -------------------------------------------------------------------------
    # State stream
    # -------------------------------------------------------------------------

    async def _stream_state(self) -> None:
        """Continuously read state for guards and observation building."""
        stream = None
        try:
            stream = self._motion_group.stream_state(
                response_rate_msecs=self._config.state_rate_ms
            )
            async for state in stream:
                self._current_joints = list(state.joint_position)
                self._log_state(state)
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
                # Track standstill for chunk_done
                if hasattr(state, "standstill"):
                    self._standstill = bool(state.standstill)
                # Track position stability
                self._update_stability(list(state.joint_position))
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
            # The server starts its internal timer when the jogger is opened.
            # Start our timer at the same boundary: immediately before sending
            # InitializeJoggingRequest. Do not reset it on the first response;
            # that would make our client timer late relative to the server.
            self._session_start_time = time.monotonic()
            self._external_start_time = time.time()
            self._action_time_offset_ms = None
            yield api.models.ExecuteJoggingRequest(
                api.models.InitializeJoggingRequest(
                    motion_group=self._motion_group.id, tcp=tcp
                )
            )

            # 2. Main loop: for each server response, either send a new
            #    chunk (if ready) or sleep until one is ready.
            async for response in response_stream:
                if not self._running:
                    return
                self._log_server_response(response)
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
                    _, steps, effective_dt_ms, start_time_ms = request
                    request = self._make_waypoints_request(
                        steps=steps,
                        effective_dt_ms=effective_dt_ms,
                        start_time_ms=start_time_ms,
                    )

                self._log_jogger_cmd("waypoints", request)
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

    def _update_stability(self, joints: list[float]) -> None:
        """Track whether position has stabilized (not moving)."""
        if self._prev_position is None:
            self._prev_position = joints
            return
        max_delta = max(
            abs(joints[j] - self._prev_position[j]) for j in range(len(joints))
        )
        self._prev_position = joints
        if max_delta < self._stable_threshold:
            self._stable_count += 1
        else:
            self._stable_count = 0

    def _position_stable(self) -> bool:
        """True if position hasn't changed for enough consecutive readings."""
        return self._stable_count >= self._stable_required

    @staticmethod
    def _get_last_target_from_request(req: object) -> list[float] | None:
        """Extract the last waypoint position from a JointWaypointsRequest."""
        if hasattr(req, "joint_waypoints") and req.joint_waypoints:
            last_wp = req.joint_waypoints[-1]
            return list(last_wp.root) if hasattr(last_wp, "root") else None
        return None

    def _make_hold_request(self, chunk_request: object) -> object:
        """Create a hold-at-last-target waypoint request.

        Uses the last waypoint from the chunk with a timestamp far enough
        in the future that it won't interfere with chunk execution.
        The server interprets this as: after finishing the chunk, hold here.
        """
        target = self._get_last_target_from_request(chunk_request)
        if target is None:
            return api.models.JointVelocityRequest(velocity=[0.0] * (self._num_joints or 6))
        # Timestamp far in the future (10s from now)
        hold_ts = int((time.monotonic() - self._session_start_time) * 1000) + 10000
        return api.models.JointWaypointsRequest(
            timestamps=[hold_ts],
            joint_waypoints=[api.models.Joints(root=target)],
        )

    def _log_state(self, state: object) -> None:
        """Log state stream update to file."""
        if self._log_fh is None:
            return
        t_ms = (time.monotonic() - self._session_start_time) * 1000
        joint_position = getattr(state, "joint_position", None)
        joints = list(joint_position) if hasattr(joint_position, "__iter__") else []

        jog_state = self._jog_tracker._extract_jogging_state(state)
        jog_payload = None
        if jog_state is not None:
            jog_payload = {
                "type": type(jog_state).__name__,
                "kind": getattr(jog_state, "kind", None),
                "description": getattr(jog_state, "description", None),
                "joint_indices": getattr(jog_state, "joint_indices", None),
            }

        self._write_log("state", {
            "t_ms": round(t_ms, 2),
            "joints": [round(j, 6) for j in joints],
            "standstill": getattr(state, "standstill", None),
            "jogging_state": jog_payload,
        })

    def _log_chunk(self, now_ms: int, timestamps: list[int], steps: list[list[float]]) -> None:
        """Log update_chunk call to file."""
        if self._log_fh is None:
            return
        self._write_log("update_chunk", {
            "now_ms": now_ms,
            "external_ms": round((time.time() - self._external_start_time) * 1000, 2),
            "n_steps": len(steps),
            "timestamps_first": timestamps[0] if timestamps else None,
            "timestamps_last": timestamps[-1] if timestamps else None,
            "timestamps": (
                [*timestamps[:5], "...", *timestamps[-2:]]
                if len(timestamps) > 7  # noqa: PLR2004
                else timestamps
            ),
            "first_step": [round(v, 6) for v in steps[0]] if steps else [],
            "last_step": [round(v, 6) for v in steps[-1]] if steps else [],
            "current_joints": (
                [round(j, 6) for j in self._current_joints]
                if self._current_joints else None
            ),
            "action_time_offset_ms": self._action_time_offset_ms,
        })

    def _log_server_response(self, response: object) -> None:
        """Log compact information from each server response."""
        if self._log_fh is None:
            return
        t_ms = (time.monotonic() - self._session_start_time) * 1000
        root = getattr(response, "root", response)
        payload: dict = {
            "t_ms": round(t_ms, 2),
            "external_ms": round((time.time() - self._external_start_time) * 1000, 2),
            "response_type": type(root).__name__,
            "kind": getattr(root, "kind", None),
        }
        for name in ("timestamp", "time", "time_ms", "elapsed_time", "elapsed_time_ms", "sequence"):
            value = getattr(root, name, None)
            if value is not None:
                payload[name] = value
        self._write_log("server_response", payload)

    def _log_jogger_cmd(self, cmd_type: str, request: object) -> None:
        """Log an actual command yielded to the jogger API."""
        if self._log_fh is None:
            return
        t_ms = (time.monotonic() - self._session_start_time) * 1000
        payload: dict = {
            "t_ms": round(t_ms, 2),
            "external_ms": round((time.time() - self._external_start_time) * 1000, 2),
            "type": cmd_type,
        }

        if hasattr(request, "timestamps") and hasattr(request, "joint_waypoints"):
            payload["n_waypoints"] = len(request.joint_waypoints)
            payload["timestamps"] = request.timestamps
            # Log first and last waypoint for compactness
            wps = request.joint_waypoints
            if wps:
                payload["wp_first"] = [round(v, 6) for v in list(wps[0].root)]
                payload["wp_last"] = [round(v, 6) for v in list(wps[-1].root)]
        elif hasattr(request, "velocity"):
            payload["velocity"] = [round(v, 6) for v in request.velocity]

        self._write_log("jogger_cmd", payload)

    def _write_log(self, event: str, data: dict) -> None:
        """Write a single log entry as JSON line."""
        if self._log_fh is None:
            return
        self._cmd_seq += 1
        entry = {"seq": self._cmd_seq, "event": event, **data}
        self._log_fh.write(json.dumps(entry) + "\n")  # type: ignore[union-attr]

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
