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
        self._chunk_end_time: float = 0.0
        self._chunk_started: bool = False  # True after first chunk is sent
        self._standstill: bool = True  # Updated by state stream

        # Position stability tracking for chunk_done
        self._prev_position: list[float] | None = None
        self._stable_count: int = 0
        self._stable_threshold = 0.001  # rad — below this = not moving
        self._stable_required = 5  # consecutive stable readings
        self._last_target: list[float] | None = None

        # Debug log file
        self._debug_file: object | None = None

        # Pending waypoints to send (set by update_chunk, consumed by jogging loop)
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
    def chunk_done(self) -> bool:
        """True when enough time has passed for the chunk to complete."""
        if not self._chunk_started:
            return True
        if self._chunk_end_time == 0.0:
            return True
        if time.monotonic() >= self._chunk_end_time:
            self._chunk_end_time = 0.0
            return True
        return False

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
        **_kwargs: object,
    ) -> None:
        """Queue a new action chunk as waypoints.

        Builds a JointWaypointsRequest with timestamps computed from dt_ms.
        The request is sent on the next jogging loop iteration.

        For single-step targets (dt_ms=0), uses a default step time of 100ms
        so the server still receives a valid waypoint.
        """
        if not steps:
            return

        self._chunk_started = True
        self._standstill = False  # assume robot will move
        self._stable_count = 0  # reset stability counter

        # Single-step target: use a default step time
        effective_dt_ms = dt_ms if dt_ms > 0 else 100.0

        # Build waypoints with timestamps from now
        now_ms = int((time.monotonic() - self._session_start_time) * 1000)
        timestamps = [now_ms + int((i + 1) * effective_dt_ms) for i in range(len(steps))]

        # Log to debug file
        self._log_chunk(now_ms, timestamps, steps)

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

        self._pending_request = api.models.JointWaypointsRequest(
            timestamps=timestamps,
            joint_waypoints=[api.models.Joints(root=step) for step in steps],
        )
        # chunk_done: waypoint duration + 0.5s for server braking
        chunk_duration_s = len(steps) * effective_dt_ms / 1000.0
        self._chunk_end_time = time.monotonic() + chunk_duration_s + 0.5

    async def write_ios(self, ios: dict[str, ValueType]) -> None:
        """Write IO values (delegated to IOWriter for deduplication)."""
        await self._io_writer.write(ios)

    async def start(self) -> None:
        """Start the state stream and jogging loop."""
        if self._running:
            msg = (
                f"WaypointJoggingSession for {self.motion_group_id} is already running."
            )
            raise RuntimeError(msg)

        self._running = True
        self._session_start_time = time.monotonic()

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

        # Open debug log
        import pathlib  # noqa: PLC0415
        log_path = pathlib.Path(f"/tmp/waypoint_debug_{self.motion_group_id.replace('@', '_')}.jsonl")  # noqa: S108
        self._debug_file = log_path.open("w")
        self._debug_file.write(f'{{"event":"start","t_ms":0,"joints":{list(self._current_joints)}}}\n')
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

        if self._debug_file is not None:
            self._debug_file.close()
            self._debug_file = None

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
            stream = self._motion_group.stream_state(
                response_rate_msecs=self._config.state_rate_ms
            )
            async for state in stream:
                self._current_joints = list(state.joint_position)
                self._log_state(state.joint_position)
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
        """Open jogging session and send waypoints only when available.

        Uses execute_jogging but breaks the ping-pong pattern: instead of
        yielding for every server response, we only yield when a new chunk
        is ready. Between chunks the server executes autonomously.
        """
        api_gateway = get_api_gateway(self._motion_group)
        cell = get_cell(self._motion_group)
        controller_id = get_controller_id(self._motion_group)
        tcp = await self._resolve_tcp()

        async def client_request_generator(
            response_stream: AsyncGenerator[api.models.ExecuteJoggingResponse, None],
        ) -> AsyncGenerator[api.models.ExecuteJoggingRequest, None]:
            yield api.models.ExecuteJoggingRequest(
                api.models.InitializeJoggingRequest(
                    motion_group=self._motion_group.id, tcp=tcp
                )
            )

            # Protocol: send waypoints ONCE per chunk with increasing
            # timestamps. For subsequent server responses, yield a hold
            # waypoint (last target at a far-future timestamp) so the
            # server knows we want to stay there after the chunk finishes.
            # Never re-send the same chunk — timestamps only go forward.
            hold_request: object | None = None
            async for response in response_stream:
                if not self._running:
                    return
                if hasattr(response.root, "kind") and response.root.kind == "MOTION_ERROR":
                    msg = getattr(response.root, "message", "unknown motion error")
                    raise MotionError(self.motion_group_id, msg)

                self._run_guards()
                self._jog_tracker.check()

                if self._pending_request is not None:
                    # New chunk: send waypoints (timestamps already correct)
                    hold_request = self._make_hold_request(self._pending_request)
                    yield api.models.ExecuteJoggingRequest(self._pending_request)
                    self._pending_request = None
                elif hold_request is not None:
                    # Keep session alive: hold at last target
                    yield api.models.ExecuteJoggingRequest(hold_request)
                else:
                    # Before first chunk: zero velocity
                    yield api.models.ExecuteJoggingRequest(
                        api.models.JointVelocityRequest(
                            velocity=[0.0] * (self._num_joints or 6)
                        )
                    )

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
        if self._stable_count >= self._stable_required:
            self._chunk_end_time = 0.0
            return True
        return False

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

    def _log_state(self, joint_position: object) -> None:
        """Log state stream update to debug file."""
        if self._debug_file is None:
            return
        t_ms = int((time.monotonic() - self._session_start_time) * 1000)
        joints = list(joint_position) if hasattr(joint_position, '__iter__') else []
        import json as _json  # noqa: PLC0415
        self._debug_file.write(
            _json.dumps({"event": "state", "t_ms": t_ms, "joints": [round(j, 5) for j in joints]}) + "\n"
        )

    def _log_chunk(self, now_ms: int, timestamps: list[int], steps: list[list[float]]) -> None:
        """Log sent chunk to debug file."""
        if self._debug_file is None:
            return
        import json as _json  # noqa: PLC0415
        self._debug_file.write(
            _json.dumps({
                "event": "chunk",
                "t_ms": now_ms,
                "timestamps": timestamps,
                "n_steps": len(steps),
                "first": [round(v, 5) for v in steps[0]] if steps else [],
                "last": [round(v, 5) for v in steps[-1]] if steps else [],
                "current": [round(v, 5) for v in self._current_joints] if self._current_joints else [],
            }) + "\n"
        )
        self._debug_file.flush()

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
