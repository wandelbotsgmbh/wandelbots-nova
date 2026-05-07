"""PID jogging session — manages velocity-streaming for one motion group.

Uses PID control to convert joint position targets into velocity commands
streamed via the NOVA Jogging API.
"""

from __future__ import annotations

import asyncio
import contextlib
import logging
import time
from typing import TYPE_CHECKING

from nova import api
from nova.types import Pose, RobotState
from policy.types import GuardState, GuardStopError, MotionError
from policy.velocity_controller import VelocityController

if TYPE_CHECKING:
    from collections.abc import AsyncGenerator

    from nova.cell.motion_group import MotionGroup
    from policy.types import PolicyRunnerConfig, SafetyGuard, ValueType

logger = logging.getLogger(__name__)


class PidJoggingSession:
    """Manages a PID-controlled jogging session for a single motion group.

    Connects to the NOVA Jogging API websocket and continuously streams
    velocity commands computed by the PID VelocityController.
    """

    def __init__(
        self,
        motion_group: MotionGroup,
        config: PolicyRunnerConfig,
        *,
        tcp: str = "",
        safety_guards: list[SafetyGuard] | None = None,
        io_values: dict[str, object] | None = None,
    ) -> None:
        self._motion_group = motion_group
        self._config = config
        self._tcp = tcp
        self._safety_guards = safety_guards or []
        self._io_values = io_values
        self._pid = VelocityController(
            velocity_limit=config.velocity_limit,
            tolerance=config.tolerance,
            p_gain=config.p_gain,
            i_gain=config.i_gain,
            d_gain=config.d_gain,
            ff_gain=config.ff_gain,
            integral_limit=config.integral_limit,
        )

        # Step sequence state
        self._steps: list[list[float]] = []
        self._step_index: int = 0
        self._dt_ms: float = 0.0
        self._step_start_time: float = 0.0

        # Current joint state (updated by state stream)
        self._current_joints: list[float] | None = None
        self._current_tcp_pose: Pose | None = None
        self._current_tcp_name: str | None = None
        self._num_joints: int | None = None

        # Safety guard state
        self._prev_state: RobotState | None = None
        self._prev_tick_time: float | None = None

        # Limit/standstill detection from state stream
        self._joint_limit_reached: list[bool] | None = None
        self._standstill: bool = False
        self._jogging_paused_reason: str | None = None  # NOVA's pause reason
        self._jogging_paused_detail: str = ""  # description or joint indices
        self._paused_tick_count: int = 0
        self._pause_raise_after: int = 10  # ticks (~100ms) — confirm it's not transient

        # Task management
        self._jogging_task: asyncio.Task[None] | None = None
        self._state_task: asyncio.Task[None] | None = None
        self._running = False
        self._failed = False

        # IO write deduplication — only write when value changes
        self._last_io_written: dict[str, ValueType] = {}
        self._io_write_lock = asyncio.Lock()
        self._failure_reason: str = ""

    @property
    def motion_group_id(self) -> str:
        """The motion group ID this session controls."""
        return self._motion_group.id

    @property
    def current_state(self) -> RobotState | None:
        """Current robot state, or None if not yet streaming."""
        if self._current_joints is None or self._current_tcp_pose is None:
            return None
        return RobotState(
            pose=self._current_tcp_pose,
            tcp=self._current_tcp_name,
            joints=tuple(self._current_joints),
        )

    @property
    def is_running(self) -> bool:
        """Whether the jogging session is active."""
        return self._running

    @property
    def has_failed(self) -> bool:
        """Whether the jogging session failed (e.g. e-stop, connection lost)."""
        return self._failed

    @property
    def failure_reason(self) -> str:
        return self._failure_reason

    def update_chunk(self, steps: list[list[float]], dt_ms: float) -> None:
        """Replace the current step sequence with a new chunk.

        Args:
            steps: List of joint position targets.
            dt_ms: Time spacing between steps in milliseconds.
        """
        self._steps = steps
        self._step_index = 0
        self._dt_ms = dt_ms
        self._step_start_time = time.monotonic()

    async def write_ios(self, ios: dict[str, ValueType]) -> None:
        """Write IO values via the motion group's API client.

        Only writes values that have changed since the last write (deduplication).
        Writes are serialized per session to avoid 429 rate limit errors.
        """
        api_client = self._motion_group._api_client
        cell = self._motion_group._cell
        controller_id = self._motion_group._controller_id

        for key, value in ios.items():
            # Skip if value hasn't changed
            if self._last_io_written.get(key) == value:
                continue
            async with self._io_write_lock:
                # Re-check after acquiring lock (another coroutine may have written)
                if self._last_io_written.get(key) == value:
                    continue
                try:
                    if isinstance(value, bool):
                        io_value = api.models.IOBooleanValue(io=key, value=value)
                    elif isinstance(value, (int, float)):
                        io_value = api.models.IOFloatValue(io=key, value=float(value))
                    else:
                        io_value = api.models.IOStringValue(io=key, value=str(value))

                    await api_client.controller_ios_api.set_output_values(
                        cell=cell, controller=controller_id, io_value=[io_value]
                    )
                    self._last_io_written[key] = value
                except Exception as e:  # noqa: BLE001
                    logger.warning("Failed to write IO %s=%s: %s", key, value, e)

    async def start(self) -> None:
        """Start the state stream and jogging loop.

        Raises:
            RuntimeError: If this session is already running.
        """
        if self._running:
            msg = (
                f"PidJoggingSession for {self.motion_group_id} is already running. "
                "Cannot start two sessions on the same motion group."
            )
            raise RuntimeError(msg)

        self._running = True

        # Fetch initial state to determine joint count
        initial_state = await self._motion_group.get_state()
        self._current_joints = list(initial_state.joints)
        self._current_tcp_pose = initial_state.pose
        self._current_tcp_name = initial_state.tcp
        self._num_joints = len(initial_state.joints)

        # Start state stream task
        self._state_task = asyncio.create_task(
            self._stream_state(), name=f"policy-state-{self.motion_group_id}"
        )

        # Start jogging task
        self._jogging_task = asyncio.create_task(
            self._jogging_loop(), name=f"policy-jog-{self.motion_group_id}"
        )

        logger.info(
            "PidJoggingSession started for %s (%d joints)", self.motion_group_id, self._num_joints
        )

    async def stop(self) -> None:
        """Stop the jogging session gracefully."""
        self._running = False
        self._steps = []
        self._pid.reset()

        for task, _name in [(self._jogging_task, "jogging"), (self._state_task, "state")]:
            if task is not None:
                task.cancel()
                with contextlib.suppress(asyncio.CancelledError, OSError, Exception):
                    await task

        self._jogging_task = None
        self._state_task = None
        logger.info("PidJoggingSession stopped for %s", self.motion_group_id)

    # -------------------------------------------------------------------------
    # Background tasks
    # -------------------------------------------------------------------------

    async def _stream_state(self) -> None:
        """Continuously read joint positions from the motion group state stream."""
        stream = None
        try:
            stream = self._motion_group.stream_state(
                response_rate_msecs=self._config.state_rate_ms
            )
            async for state in stream:
                self._current_joints = list(state.joint_position)
                if state.tcp_pose is not None:
                    self._current_tcp_pose = Pose(state.tcp_pose)
                if state.tcp is not None:
                    self._current_tcp_name = state.tcp
                # Track limit/standstill flags from NOVA
                if hasattr(state, "joint_limit_reached") and state.joint_limit_reached is not None:
                    self._joint_limit_reached = state.joint_limit_reached.limit_reached
                if hasattr(state, "standstill"):
                    self._standstill = state.standstill
                # Track jogging pause reason from execute.details
                self._read_jogging_state(state)
        except asyncio.CancelledError:
            pass
        except (OSError, RuntimeError) as e:
            logger.error("State stream error for %s: %s", self.motion_group_id, e)
        finally:
            if stream is not None:
                with contextlib.suppress(Exception):
                    await stream.aclose()

    def _read_jogging_state(self, state: object) -> None:
        """Extract jogging pause reason from MotionGroupState.execute.details."""
        execute = getattr(state, "execute", None)
        if execute is None:
            self._jogging_paused_reason = None
            self._jogging_paused_detail = ""
            return

        details = getattr(execute, "details", None)
        if details is None:
            self._jogging_paused_reason = None
            self._jogging_paused_detail = ""
            return

        jog_state = getattr(details, "state", None)
        if jog_state is None:
            self._jogging_paused_reason = None
            self._jogging_paused_detail = ""
            return

        kind = getattr(jog_state, "kind", "RUNNING")
        if kind == "RUNNING":
            self._jogging_paused_reason = None
            self._jogging_paused_detail = ""
        else:
            self._jogging_paused_reason = kind
            # Extract extra info if available
            if hasattr(jog_state, "joint_indices"):
                self._jogging_paused_detail = f"joints: {jog_state.joint_indices}"
            elif hasattr(jog_state, "description"):
                self._jogging_paused_detail = jog_state.description
            else:
                self._jogging_paused_detail = ""

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

    async def _jogging_loop(self) -> None:
        """Run the jogging API websocket, streaming velocity commands."""
        api_gateway = self._motion_group._api_client
        cell = self._motion_group._cell
        controller_id = self._motion_group._controller_id
        tcp = await self._resolve_tcp()

        async def client_request_generator(
            response_stream: AsyncGenerator[api.models.ExecuteJoggingResponse, None],
        ) -> AsyncGenerator[api.models.ExecuteJoggingRequest, None]:
            # Send initialization
            init_req = api.models.InitializeJoggingRequest(
                motion_group=self._motion_group.id, tcp=tcp
            )
            yield api.models.ExecuteJoggingRequest(init_req)

            # Stream velocity commands
            async for response in response_stream:
                if not self._running:
                    return
                # Check for motion errors (joint limits, self-collision)
                if hasattr(response.root, "kind") and response.root.kind == "MOTION_ERROR":
                    msg = getattr(response.root, "message", "unknown motion error")
                    raise MotionError(self.motion_group_id, msg)
                velocity = self._compute_velocity_with_safety()
                self._check_standstill()
                yield api.models.ExecuteJoggingRequest(
                    api.models.JointVelocityRequest(velocity=velocity)
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
            self._running = False
            logger.warning(
                "Jogging stopped for %s: %s",
                self.motion_group_id,
                e,
            )
        except (OSError, RuntimeError) as e:
            if self._running:
                self._failed = True
                self._failure_reason = str(e)
                self._running = False
                logger.error(
                    "Jogging connection lost for %s: %s (e-stop or network failure)",
                    self.motion_group_id,
                    e,
                )

    # -------------------------------------------------------------------------
    # Velocity computation + safety
    # -------------------------------------------------------------------------

    def _get_active_target(self) -> list[float] | None:
        """Get the currently active target based on step index and timing."""
        if not self._steps:
            return None

        # Advance step index based on dt_ms
        if self._dt_ms > 0.0 and self._step_index < len(self._steps) - 1:
            elapsed = (time.monotonic() - self._step_start_time) * 1000.0
            new_index = min(int(elapsed / self._dt_ms), len(self._steps) - 1)
            self._step_index = new_index

        return self._steps[self._step_index]

    def _check_standstill(self) -> None:
        """Detect if NOVA paused jogging due to limits, collision, or singularity.

        NOVA reports the exact reason via execute.details.state in the state stream:
        - PAUSED_NEAR_JOINT_LIMIT (includes joint_indices)
        - PAUSED_NEAR_COLLISION (includes description)
        - PAUSED_NEAR_SINGULARITY (includes description)

        We wait a few ticks to confirm it's not transient before raising.
        """
        if self._jogging_paused_reason is None:
            self._paused_tick_count = 0
            return

        # Only care about motion-blocking pauses
        blocking_pauses = {
            "PAUSED_NEAR_JOINT_LIMIT",
            "PAUSED_NEAR_COLLISION",
            "PAUSED_NEAR_SINGULARITY",
        }
        if self._jogging_paused_reason not in blocking_pauses:
            self._paused_tick_count = 0
            return

        self._paused_tick_count += 1
        if self._paused_tick_count >= self._pause_raise_after:
            reason = self._jogging_paused_reason.replace("PAUSED_NEAR_", "").lower()
            detail = f" ({self._jogging_paused_detail})" if self._jogging_paused_detail else ""
            raise MotionError(
                self.motion_group_id,
                f"Jogging paused: {reason}{detail}",
            )

    def _compute_velocity_with_safety(self) -> list[float]:
        """Compute velocity command, running safety guards."""
        if self._num_joints is None:
            return []

        n = self._num_joints
        current = self._current_joints
        target = self._get_active_target()

        if current is None or target is None:
            return [0.0] * n

        # Build current robot state (used by guards and standstill detection)
        current_robot_state = None
        if self._current_tcp_pose is not None:
            current_robot_state = RobotState(
                pose=self._current_tcp_pose, tcp=self._current_tcp_name, joints=tuple(current)
            )

        # Run safety guards
        if self._safety_guards and current_robot_state is not None:
            now = time.monotonic()
            dt = now - self._prev_tick_time if self._prev_tick_time is not None else 0.01
            ctx = GuardState(
                state=current_robot_state,
                prev_state=self._prev_state,
                dt=dt,
                motion_group_id=self.motion_group_id,
                io_values=self._io_values,
            )
            for guard in self._safety_guards:
                if not guard(ctx):
                    guard_name = getattr(guard, "__name__", repr(guard))
                    logger.warning(
                        "Safety guard '%s' triggered for %s", guard_name, self.motion_group_id
                    )
                    self._running = False
                    raise GuardStopError(self.motion_group_id, guard_name)
            self._prev_tick_time = now

        # Always track prev_state for standstill detection
        if current_robot_state is not None:
            self._prev_state = current_robot_state

        return self._pid.compute(current, target)
