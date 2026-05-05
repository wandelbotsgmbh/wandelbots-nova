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
from policy.types import GuardState, GuardStopError
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
        safety_guards: list[SafetyGuard] | None = None,
    ) -> None:
        self._motion_group = motion_group
        self._config = config
        self._safety_guards = safety_guards or []
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

        # Task management
        self._jogging_task: asyncio.Task[None] | None = None
        self._state_task: asyncio.Task[None] | None = None
        self._running = False
        self._failed = False
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

        Args:
            ios: Mapping of io_key → value.
        """
        api_client = self._motion_group._api_client
        cell = self._motion_group._cell
        controller_id = self._motion_group._controller_id

        for key, value in ios.items():
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
            except (OSError, RuntimeError, ValueError) as e:
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
        try:
            async for state in self._motion_group.stream_state(
                response_rate_msecs=self._config.state_rate_ms
            ):
                self._current_joints = list(state.joint_position)
                if state.tcp_pose is not None:
                    self._current_tcp_pose = Pose(state.tcp_pose)
                if state.tcp is not None:
                    self._current_tcp_name = state.tcp
        except asyncio.CancelledError:
            pass
        except (OSError, RuntimeError) as e:
            logger.error("State stream error for %s: %s", self.motion_group_id, e)

    async def _jogging_loop(self) -> None:
        """Run the jogging API websocket, streaming velocity commands."""
        api_gateway = self._motion_group._api_client
        cell = self._motion_group._cell
        controller_id = self._motion_group._controller_id

        # Determine TCP from motion group's active TCP
        tcp = await self._motion_group.active_tcp_name()
        if tcp is None:
            tcp_names = await self._motion_group.tcp_names()
            if tcp_names:
                tcp = tcp_names[0]
            else:
                logger.warning(
                    "No TCP found for %s, jogging may not work correctly", self.motion_group_id
                )
                tcp = ""

        async def client_request_generator(
            response_stream: AsyncGenerator[api.models.ExecuteJoggingResponse, None],
        ) -> AsyncGenerator[api.models.ExecuteJoggingRequest, None]:
            # Send initialization
            init_req = api.models.InitializeJoggingRequest(
                motion_group=self._motion_group.id, tcp=tcp
            )
            yield api.models.ExecuteJoggingRequest(init_req)

            # Stream velocity commands
            async for _ in response_stream:
                if not self._running:
                    return
                velocity = self._compute_velocity_with_safety()
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
        except GuardStopError as e:
            self._failed = True
            self._failure_reason = str(e)
            self._running = False
            logger.warning(
                "Safety guard stopped jogging for %s: %s",
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

    def _compute_velocity_with_safety(self) -> list[float]:
        """Compute velocity command, running safety guards."""
        if self._num_joints is None:
            return []

        n = self._num_joints
        current = self._current_joints
        target = self._get_active_target()

        if current is None or target is None:
            return [0.0] * n

        # Run safety guards
        if self._safety_guards and self._current_tcp_pose is not None:
            now = time.monotonic()
            dt = now - self._prev_tick_time if self._prev_tick_time is not None else 0.01
            current_robot_state = RobotState(
                pose=self._current_tcp_pose, tcp=self._current_tcp_name, joints=tuple(current)
            )
            ctx = GuardState(
                state=current_robot_state,
                prev_state=self._prev_state,
                dt=dt,
                motion_group_id=self.motion_group_id,
            )
            for guard in self._safety_guards:
                if not guard(ctx):
                    guard_name = getattr(guard, "__name__", repr(guard))
                    logger.warning(
                        "Safety guard '%s' triggered for %s", guard_name, self.motion_group_id
                    )
                    self._running = False
                    raise GuardStopError(self.motion_group_id, guard_name)

            self._prev_state = current_robot_state
            self._prev_tick_time = now

        return self._pid.compute(current, target)
