"""Jogging session — manages velocity-streaming for one motion group.

Uses a trapezoidal velocity profile to convert joint position targets into
velocity commands streamed via the NOVA Jogging API.

The profile computes velocities upfront from position differences between
chunk steps, applies a trapezoidal ramp envelope, and advances based on
the robot's actual position at runtime. This guarantees:
- Zero overshoot (velocity is zero at the last step)
- Correct timing under latency (profile advances when robot moves)
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
from policy.jogging.velocity_profile import VelocityProfile
from policy.types import GuardState, GuardStopError, MotionError

if TYPE_CHECKING:
    from collections.abc import AsyncGenerator

    from nova.cell.motion_group import MotionGroup
    from policy.types import JoggingMode, MotionConfig, SafetyGuard, ValueType

logger = logging.getLogger(__name__)

_MIN_DT = 0.001


class JoggingSession:
    """Manages a jogging session for a single motion group.

    Connects to the NOVA Jogging API websocket and continuously streams
    velocity commands computed from a trapezoidal velocity profile.
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
        self._motion_group = motion_group
        self._config = config
        self._tcp = tcp
        self._mode: JoggingMode = mode
        self._safety_guards = safety_guards or []
        self._io_values: dict[str, object] | None = None
        self._io_writer = IOWriter(motion_group)
        self._jog_tracker = JoggingStateTracker(motion_group.id)
        self._profile = VelocityProfile(
            n_joints=6,  # updated on start when DOF is known
            vel_limit=config.velocity_limit,
            ramp_steps=config.ramp_steps,
            p_gain=config.p_gain,
        )

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
        """True when the velocity profile has been fully traversed."""
        return self._profile.done

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
        """Update with a new action chunk."""
        self._profile.set_chunk(steps, dt_ms)

    async def write_ios(self, ios: dict[str, ValueType]) -> None:
        """Write IO values (delegated to IOWriter for deduplication)."""
        await self._io_writer.write(ios)

    async def start(self) -> None:
        """Start the state stream and jogging loop."""
        if self._running:
            msg = (
                f"JoggingSession for {self.motion_group_id} is already running. "
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

        # Reinitialize profile with correct DOF
        self._profile = VelocityProfile(
            n_joints=self._num_joints,
            vel_limit=self._config.velocity_limit,
            ramp_steps=self._config.ramp_steps,
            p_gain=self._config.p_gain,
        )

        self._state_task = asyncio.create_task(
            self._stream_state(), name=f"jog-state-{self.motion_group_id}"
        )
        self._jogging_task = asyncio.create_task(
            self._jogging_loop(), name=f"jog-loop-{self.motion_group_id}"
        )
        logger.info(
            "JoggingSession started for %s (%d joints)", self.motion_group_id, self._num_joints
        )

    async def stop(self) -> None:
        """Stop the jogging session gracefully."""
        self._running = False

        for task in (self._jogging_task, self._state_task):
            if task is not None:
                task.cancel()
                with contextlib.suppress(asyncio.CancelledError, OSError, RuntimeError):
                    await task

        self._jogging_task = None
        self._state_task = None
        logger.info("JoggingSession stopped for %s", self.motion_group_id)

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
                with contextlib.suppress(asyncio.CancelledError, OSError, RuntimeError):
                    await stream.aclose()

    # -------------------------------------------------------------------------
    # Jogging loop
    # -------------------------------------------------------------------------

    async def _jogging_loop(self) -> None:
        """Run the jogging API websocket, streaming velocity commands."""
        api_gateway = get_api_gateway(self._motion_group)
        cell = get_cell(self._motion_group)
        controller_id = get_controller_id(self._motion_group)
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
                velocity = self._compute_velocity()
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
            self._failure_exception = e
            self._running = False
            logger.warning("Jogging stopped for %s: %s", self.motion_group_id, e)
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

    # -------------------------------------------------------------------------
    # Velocity computation
    # -------------------------------------------------------------------------

    def _get_current_position(self) -> list[float] | None:
        """Get current position vector: joints or TCP pose depending on mode."""
        if self._mode == "cartesian":
            if self._current_tcp_pose is None:
                return None
            return list(self._current_tcp_pose.position) + list(self._current_tcp_pose.orientation)
        return list(self._current_joints) if self._current_joints is not None else None

    def _get_zero_velocity(self) -> list[float]:
        """Return zero velocity of the right dimension."""
        if self._mode == "cartesian":
            return [0.0] * 6
        return [0.0] * (self._num_joints or 6)

    def _compute_velocity(self) -> list[float]:
        """Compute velocity from the trapezoidal profile + safety guards."""
        current = self._get_current_position()
        if current is None:
            return self._get_zero_velocity()

        # Run safety guards
        current_robot_state = self._build_robot_state()
        if self._safety_guards and current_robot_state is not None:
            self._check_safety_guards(current_robot_state)
        if current_robot_state is not None:
            self._prev_state = current_robot_state

        return self._profile.compute(current, time.monotonic())

    def _check_safety_guards(self, current_robot_state: RobotState) -> None:
        """Run all safety guards. Raises GuardStopError on failure."""
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

    def _make_velocity_request(
        self, velocity: list[float],
    ) -> api.models.JointVelocityRequest | api.models.TcpVelocityRequest:
        """Wrap velocity into the right request type for the current mode."""
        if self._mode == "cartesian":
            return api.models.TcpVelocityRequest(
                translation=api.models.Vector3d(velocity[:3]),
                rotation=api.models.Vector3d(velocity[3:6]),
            )
        return api.models.JointVelocityRequest(velocity=velocity)

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


# ---------------------------------------------------------------------------
# Jogging state tracker
# ---------------------------------------------------------------------------

_BLOCKING_PAUSES = frozenset({
    "PAUSED_NEAR_JOINT_LIMIT",
    "PAUSED_NEAR_COLLISION",
    "PAUSED_NEAR_SINGULARITY",
})


class JoggingStateTracker:
    """Tracks NOVA jogging pause state and raises on confirmed standstill."""

    def __init__(self, motion_group_id: str, *, confirm_ticks: int = 10) -> None:
        self.motion_group_id = motion_group_id
        self._confirm_ticks = confirm_ticks
        self._paused_reason: str | None = None
        self._paused_detail: str = ""
        self._paused_count: int = 0

    def update_from_state(self, state: object) -> None:
        """Extract jogging pause reason from MotionGroupState.execute.details."""
        jog_state = self._extract_jogging_state(state)

        if jog_state is None:
            self._paused_reason = None
            self._paused_detail = ""
            return

        kind: str = getattr(jog_state, "kind", "RUNNING")
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

    @staticmethod
    def _extract_jogging_state(state: object) -> object | None:
        """Navigate MotionGroupState.execute.details.state safely."""
        execute = getattr(state, "execute", None)
        if execute is None:
            return None
        details = getattr(execute, "details", None)
        if details is None:
            return None
        return getattr(details, "state", None)

    def check(self) -> None:
        """Raise MotionError after confirmed blocking pause."""
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
