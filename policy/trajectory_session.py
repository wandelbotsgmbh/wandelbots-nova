"""Trajectory-based motion session using joint_ptp movements.

Drop-in alternative to ``PidJoggingSession`` that uses Nova's trajectory
planner instead of PID velocity control.  Advantages:

- Built-in collision avoidance from the motion planner
- Smooth trapezoidal/S-curve velocity profiles
- No PID tuning required

Each action chunk is planned as a multi-waypoint trajectory using all steps.
When a new chunk arrives, the current execution is cancelled and replaced.
"""

from __future__ import annotations

import asyncio
import contextlib
import logging
from typing import TYPE_CHECKING, Any

from nova.actions import joint_ptp
from nova.types import MotionSettings, Pose, RobotState
from policy.io import IOWriter

if TYPE_CHECKING:
    from nova.cell.motion_group import MotionGroup
    from policy.types import ValueType

logger = logging.getLogger(__name__)


class TrajectorySession:
    """Motion session that executes action chunks via planned trajectories.

    Same interface as ``PidJoggingSession`` so the executor can use either.
    """

    def __init__(
        self,
        motion_group: MotionGroup,
        *,
        tcp: str | None = None,
        velocity_limit: float = 500.0,
        safety_guards: list[Any] | None = None,
        mode: str = "joint",
    ) -> None:
        self._motion_group = motion_group
        self._tcp_name = tcp
        self._velocity_limit = velocity_limit
        self._safety_guards = safety_guards or []
        self._mode = mode

        self._running = False
        self._failed = False
        self._failure_reason = ""
        self._failure_exception: BaseException | None = None

        self._current_state: RobotState | None = None
        self._state_task: asyncio.Task[None] | None = None
        self._execute_task: asyncio.Task[None] | None = None
        self._current_execution: asyncio.Task[None] | None = None
        self._io_values: dict[str, object] | None = None
        self._io_writer: IOWriter | None = None

        # Pending chunk — set by update_chunk, consumed by _execute_loop
        self._pending_steps: list[list[float]] | None = None
        self._target_event = asyncio.Event()

    def set_io_values_ref(self, values: dict[str, object]) -> None:
        self._io_values = values

    @property
    def motion_group(self) -> MotionGroup:
        return self._motion_group

    @property
    def motion_group_id(self) -> str:
        return self._motion_group.id

    @property
    def current_state(self) -> RobotState | None:
        return self._current_state

    @property
    def is_running(self) -> bool:
        return self._running

    @property
    def has_failed(self) -> bool:
        return self._failed

    @property
    def failure_reason(self) -> str:
        return self._failure_reason

    @property
    def failure_exception(self) -> BaseException | None:
        return self._failure_exception

    def update_chunk(self, steps: list[list[float]], dt_ms: float) -> None:
        """Accept a new action chunk — all steps become trajectory waypoints.

        If a trajectory is currently executing, it will be cancelled so
        the new chunk starts immediately.
        """
        del dt_ms
        if not steps:
            return
        self._pending_steps = steps
        self._target_event.set()

    async def write_ios(self, ios: dict[str, ValueType]) -> None:
        if self._io_writer is not None:
            await self._io_writer.write(ios)

    async def start(self) -> None:
        """Start state streaming and the execution loop."""
        mg = self._motion_group

        self._io_writer = IOWriter(mg)

        # Resolve TCP
        if self._tcp_name is None:
            tcps = await mg.tcp_names()
            self._tcp_name = tcps[0] if tcps else "Flange"

        # Get initial state
        self._current_state = await mg.get_state()

        self._running = True
        self._state_task = asyncio.create_task(
            self._stream_state(), name=f"traj-state-{mg.id}"
        )
        self._execute_task = asyncio.create_task(
            self._execute_loop(), name=f"traj-exec-{mg.id}"
        )
        logger.info(
            "TrajectorySession started for %s (tcp=%s, velocity=%.0f mm/s)",
            mg.id, self._tcp_name, self._velocity_limit,
        )

    async def stop(self) -> None:
        """Stop the session."""
        self._running = False
        self._target_event.set()

        if self._current_execution is not None:
            self._current_execution.cancel()
            with contextlib.suppress(asyncio.CancelledError, OSError, RuntimeError):
                await self._current_execution

        for task in (self._execute_task, self._state_task):
            if task is not None:
                task.cancel()
                with contextlib.suppress(asyncio.CancelledError, OSError, RuntimeError):
                    await task

        self._execute_task = None
        self._state_task = None
        self._current_execution = None
        logger.info("TrajectorySession stopped for %s", self._motion_group.id)

    # ------------------------------------------------------------------
    # State streaming
    # ------------------------------------------------------------------

    async def _stream_state(self) -> None:
        """Continuously update current_state from the motion group state stream."""
        mg = self._motion_group
        stream = None

        try:
            stream = mg.stream_state(response_rate_msecs=10)
            async for state in stream:
                if not self._running:
                    break
                pose = Pose(state.tcp_pose) if state.tcp_pose is not None else None
                torques = (
                    tuple(state.joint_torque.root)
                    if getattr(state, "joint_torque", None) is not None
                    else None
                )
                currents = (
                    tuple(state.joint_current.root)
                    if getattr(state, "joint_current", None) is not None
                    else None
                )
                self._current_state = RobotState(
                    pose=pose,
                    tcp=self._tcp_name,
                    joints=tuple(state.joint_position),
                    joint_torques=torques,
                    joint_currents=currents,
                )
        except asyncio.CancelledError:
            pass
        except (OSError, RuntimeError) as e:
            if self._running:
                self._failed = True
                self._failure_reason = f"State stream error: {e}"
                self._failure_exception = e
        finally:
            if stream is not None:
                await stream.aclose()

    # ------------------------------------------------------------------
    # Trajectory execution loop
    # ------------------------------------------------------------------

    async def _execute_loop(self) -> None:
        """Wait for chunks and execute multi-waypoint trajectories."""
        mg = self._motion_group

        try:
            while self._running:
                self._target_event.clear()
                await self._target_event.wait()

                if not self._running:
                    break

                steps = self._pending_steps
                if not steps:
                    continue
                self._pending_steps = None

                try:
                    await self._plan_and_execute(mg, steps)
                except asyncio.CancelledError:
                    # Execution was cancelled by a new chunk — this is normal
                    logger.debug("Trajectory cancelled for %s (new chunk)", mg.id)
                    continue
                except (OSError, RuntimeError) as e:
                    logger.warning("Trajectory failed for %s: %s", mg.id, e)
                    self._failed = True
                    self._failure_reason = str(e)
                    self._failure_exception = e
                    break

        except asyncio.CancelledError:
            pass

    async def _plan_and_execute(self, mg: MotionGroup, steps: list[list[float]]) -> None:
        """Plan a multi-waypoint trajectory from the chunk and execute it."""
        settings = MotionSettings(tcp_velocity_limit=self._velocity_limit)
        tcp = self._tcp_name or "Flange"

        actions = [joint_ptp(tuple(step), settings=settings) for step in steps]

        t0 = asyncio.get_event_loop().time()
        trajectory = await mg.plan(actions, tcp)
        t1 = asyncio.get_event_loop().time()

        # Run execute in its own task so update_chunk() can cancel just the move
        exec_task = asyncio.create_task(
            mg.execute(trajectory, tcp, actions=actions),
            name=f"traj-move-{mg.id}",
        )
        self._current_execution = exec_task
        try:
            await asyncio.shield(exec_task)
        except asyncio.CancelledError:
            # New chunk arrived — cancel the execution task and re-raise
            exec_task.cancel()
            with contextlib.suppress(asyncio.CancelledError):
                await exec_task
            raise
        finally:
            self._current_execution = None
        t2 = asyncio.get_event_loop().time()

        logger.info(
            "Trajectory %s: %d waypoints, plan=%.0fms exec=%.0fms",
            mg.id, len(steps), (t1 - t0) * 1000, (t2 - t1) * 1000,
        )
