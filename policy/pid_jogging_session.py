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
from policy._sdk import get_api_gateway, get_cell
from policy.io import IOWriter
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
        mode: str = "joint",
        safety_guards: list[SafetyGuard] | None = None,
        io_values: dict[str, object] | None = None,
    ) -> None:
        self._motion_group = motion_group
        self._config = config
        self._tcp = tcp
        self._mode = mode  # "joint" or "cartesian"
        self._safety_guards = safety_guards or []
        self._io_values = io_values
        self._pid = VelocityController(
            velocity_limit=self._resolve_velocity_limit_for_mode(),
            tolerance=config.tolerance,
            p_gain=config.p_gain,
            i_gain=config.i_gain,
            d_gain=config.d_gain,
            ff_gain=config.ff_gain,
            integral_limit=config.integral_limit,
        )
        self._io_writer = IOWriter(motion_group)
        self._jog_tracker = JoggingStateTracker(motion_group.id)

        # Step sequence state
        self._steps: list[list[float]] = []
        self._step_index: int = 0
        self._dt_ms: float = 0.0
        self._step_start_time: float = 0.0

        # Current joint state (updated by state stream)
        self._current_joints: list[float] | None = None
        self._current_joint_torques: list[float] | None = None
        self._current_joint_currents: list[float] | None = None
        self._current_tcp_pose: Pose | None = None
        self._current_tcp_name: str | None = None
        self._num_joints: int | None = None

        # Safety guard state
        self._prev_state: RobotState | None = None
        self._prev_tick_time: float | None = None

        # Task management
        self._jogging_task: asyncio.Task[None] | None = None
        self._state_task: asyncio.Task[None] | None = None
        self._running = False
        self._failed = False
        self._failure_reason: str = ""

    def _resolve_velocity_limit_for_mode(self) -> float | list[float]:
        """Resolve velocity limit considering the jogging mode."""
        if self._mode == "cartesian":
            return [
                self._config.tcp_velocity_limit,
                self._config.tcp_velocity_limit,
                self._config.tcp_velocity_limit,
                self._config.tcp_orientation_velocity_limit,
                self._config.tcp_orientation_velocity_limit,
                self._config.tcp_orientation_velocity_limit,
            ]
        return self._config.velocity_limit

    @property
    def motion_group_id(self) -> str:
        return self._motion_group.id

    @property
    def current_state(self) -> RobotState | None:
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

    @property
    def is_running(self) -> bool:
        return self._running

    @property
    def has_failed(self) -> bool:
        return self._failed

    @property
    def failure_reason(self) -> str:
        return self._failure_reason

    def update_chunk(self, steps: list[list[float]], dt_ms: float) -> None:
        """Replace the current step sequence with a new chunk."""
        self._steps = steps
        self._step_index = 0
        self._dt_ms = dt_ms
        self._step_start_time = time.monotonic()

    async def write_ios(self, ios: dict[str, ValueType]) -> None:
        """Write IO values (delegated to IOWriter for deduplication)."""
        await self._io_writer.write(ios)

    async def start(self) -> None:
        """Start the state stream and jogging loop."""
        if self._running:
            msg = (
                f"PidJoggingSession for {self.motion_group_id} is already running. "
                "Cannot start two sessions on the same motion group."
            )
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
            self._stream_state(), name=f"policy-state-{self.motion_group_id}"
        )
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

        for task in (self._jogging_task, self._state_task):
            if task is not None:
                task.cancel()
                with contextlib.suppress(asyncio.CancelledError, OSError, Exception):
                    await task

        self._jogging_task = None
        self._state_task = None
        logger.info("PidJoggingSession stopped for %s", self.motion_group_id)

    # -------------------------------------------------------------------------
    # State stream
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
                self._jog_tracker.update_from_state(state)
        except asyncio.CancelledError:
            pass
        except (OSError, RuntimeError) as e:
            logger.error("State stream error for %s: %s", self.motion_group_id, e)
        finally:
            if stream is not None:
                with contextlib.suppress(Exception):
                    await stream.aclose()

    # -------------------------------------------------------------------------
    # Jogging loop
    # -------------------------------------------------------------------------

    async def _jogging_loop(self) -> None:
        """Run the jogging API websocket, streaming velocity commands."""
        api_gateway = get_api_gateway(self._motion_group)
        cell = get_cell(self._motion_group)
        controller_id = self._motion_group.id.split("@")[1] if "@" in self._motion_group.id else self._motion_group.id
        tcp = await self._resolve_tcp()

        async def client_request_generator(
            response_stream: AsyncGenerator[api.models.ExecuteJoggingResponse, None],
        ) -> AsyncGenerator[api.models.ExecuteJoggingRequest, None]:
            init_req = api.models.InitializeJoggingRequest(
                motion_group=self._motion_group.id, tcp=tcp
            )
            yield api.models.ExecuteJoggingRequest(init_req)

            async for response in response_stream:
                if not self._running:
                    return
                if hasattr(response.root, "kind") and response.root.kind == "MOTION_ERROR":
                    msg = getattr(response.root, "message", "unknown motion error")
                    raise MotionError(self.motion_group_id, msg)
                velocity = self._compute_velocity_with_safety()
                self._jog_tracker.check()
                yield api.models.ExecuteJoggingRequest(
                    self._make_velocity_request(velocity)
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
            logger.warning("Jogging stopped for %s: %s", self.motion_group_id, e)
        except (OSError, RuntimeError) as e:
            if self._running:
                self._failed = True
                self._failure_reason = str(e)
                self._running = False
                logger.error(
                    "Jogging connection lost for %s: %s", self.motion_group_id, e,
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

    # -------------------------------------------------------------------------
    # Velocity computation + safety guards
    # -------------------------------------------------------------------------

    def _get_active_target(self) -> list[float] | None:
        """Get the currently active target based on step index and timing."""
        if not self._steps:
            return None

        if self._dt_ms > 0.0 and self._step_index < len(self._steps) - 1:
            elapsed = (time.monotonic() - self._step_start_time) * 1000.0
            new_index = min(int(elapsed / self._dt_ms), len(self._steps) - 1)
            self._step_index = new_index

        return self._steps[self._step_index]

    def _make_velocity_request(
        self, velocity: list[float],
    ) -> api.models.JointVelocityRequest | api.models.TcpVelocityRequest:
        """Wrap velocity list into the right request type for the current mode."""
        if self._mode == "cartesian":
            return api.models.TcpVelocityRequest(
                translation=api.models.Vector3d(velocity[:3]),
                rotation=api.models.Vector3d(velocity[3:6]),
            )
        return api.models.JointVelocityRequest(velocity=velocity)

    def _get_current_for_pid(self) -> list[float] | None:
        """Get current position vector for PID: joints or TCP pose."""
        if self._mode == "cartesian":
            if self._current_tcp_pose is None:
                return None
            return list(self._current_tcp_pose.position) + list(self._current_tcp_pose.orientation)
        return self._current_joints

    def _get_zero_velocity(self) -> list[float]:
        """Return zero velocity of the right dimension."""
        if self._mode == "cartesian":
            return [0.0] * 6
        return [0.0] * (self._num_joints or 6)

    def _compute_velocity_with_safety(self) -> list[float]:
        """Compute velocity command, running safety guards first."""
        current = self._get_current_for_pid()
        target = self._get_active_target()

        if current is None or target is None:
            return self._get_zero_velocity()

        current_robot_state = None
        if self._current_tcp_pose is not None and self._current_joints is not None:
            current_robot_state = RobotState(
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

        if current_robot_state is not None:
            self._prev_state = current_robot_state

        return self._pid.compute(current, target)


# ---------------------------------------------------------------------------
# Jogging state tracker (internal to session)
# ---------------------------------------------------------------------------

_BLOCKING_PAUSES = frozenset({
    "PAUSED_NEAR_JOINT_LIMIT",
    "PAUSED_NEAR_COLLISION",
    "PAUSED_NEAR_SINGULARITY",
})


class JoggingStateTracker:
    """Tracks NOVA jogging pause state and raises on confirmed standstill.

    Fed from the state stream on every tick. Waits ``confirm_ticks`` ticks
    of continuous pause before raising ``MotionError``.
    """

    def __init__(self, motion_group_id: str, *, confirm_ticks: int = 10) -> None:
        self.motion_group_id = motion_group_id
        self._confirm_ticks = confirm_ticks
        self._paused_reason: str | None = None
        self._paused_detail: str = ""
        self._paused_count: int = 0

    def update_from_state(self, state: object) -> None:
        """Extract jogging pause reason from MotionGroupState.execute.details."""
        execute = getattr(state, "execute", None)
        details = getattr(execute, "details", None) if execute else None
        jog_state = getattr(details, "state", None) if details else None

        if jog_state is None:
            self._paused_reason = None
            self._paused_detail = ""
            return

        kind = getattr(jog_state, "kind", "RUNNING")
        if kind == "RUNNING":
            self._paused_reason = None
            self._paused_detail = ""
        else:
            self._paused_reason = kind
            if hasattr(jog_state, "joint_indices"):
                self._paused_detail = f"joints: {jog_state.joint_indices}"
            elif hasattr(jog_state, "description"):
                self._paused_detail = jog_state.description
            else:
                self._paused_detail = ""

    def check(self) -> None:
        """Raise ``MotionError`` after confirmed blocking pause."""
        if self._paused_reason is None or self._paused_reason not in _BLOCKING_PAUSES:
            self._paused_count = 0
            return

        self._paused_count += 1
        if self._paused_count >= self._confirm_ticks:
            reason = self._paused_reason.replace("PAUSED_NEAR_", "").lower()
            detail = f" ({self._paused_detail})" if self._paused_detail else ""
            raise MotionError(
                self.motion_group_id,
                f"Jogging paused: {reason}{detail}",
            )
