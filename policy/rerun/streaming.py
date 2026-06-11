"""Continuous state streaming — logs robot position between policy steps."""

from __future__ import annotations

import asyncio
import contextlib
import logging
import time
from typing import TYPE_CHECKING, Any

from policy.rerun.constants import _MIN_LINE_STEPS, _TCP_TRAIL_COLOR, _TRAIL_WIDTH_UI

if TYPE_CHECKING:
    from nova.types import RobotState

logger = logging.getLogger(__name__)


class StateStreamer:
    """Polls session states at ~100Hz and logs to Rerun.

    Provides dense robot position data between policy calls, so the Rerun
    viewer shows smooth robot mesh movement and a complete TCP trail.
    """

    def __init__(
        self,
        *,
        start_time: float,
        dh_robots: dict[str, Any],
        visualizers: dict[str, Any],
        tcp_trail: dict[str, list[list[float]]],
        max_trail_points: int,
    ) -> None:
        self._start_time = start_time
        self._dh_robots = dh_robots
        self._visualizers = visualizers
        self._tcp_trail = tcp_trail
        self._max_trail_points = max_trail_points
        self._sessions: dict[str, Any] | None = None
        self._task: asyncio.Task[None] | None = None
        self._running = False
        self._tick_counter = 0

    def start(self, sessions: dict[str, Any]) -> None:
        """Start the background streaming task."""
        self._sessions = sessions
        self._running = True
        self._task = asyncio.create_task(self._loop(), name="rerun-state-stream")

    async def stop(self) -> None:
        """Stop the background streaming task."""
        self._running = False
        if self._task is not None:
            self._task.cancel()
            with contextlib.suppress(asyncio.CancelledError):
                await self._task
            self._task = None
        self._sessions = None

    async def _loop(self) -> None:
        """Poll session states at ~100Hz and log to Rerun."""
        import rerun as rr  # noqa: PLC0415

        try:
            while self._running:
                if self._sessions is None:
                    break

                elapsed = time.monotonic() - self._start_time
                rr.set_time("policy_time", duration=elapsed)
                self._tick_counter += 1
                rr.set_time("state_tick", sequence=self._tick_counter)

                for mg_id, session in self._sessions.items():
                    state = session.current_state
                    if state is not None:
                        self._log_state(mg_id, state)

                await asyncio.sleep(0.01)  # ~100Hz
        except asyncio.CancelledError:
            # Expected on shutdown; stop quietly without logging as an error.
            pass
        except (OSError, RuntimeError) as e:
            logger.debug("State stream logging stopped: %s", e)

    def _log_state(self, mg_id: str, state: RobotState) -> None:
        """Log a single state sample for one motion group."""
        import rerun as rr  # noqa: PLC0415

        joints = list(state.joints)

        # Update 3D robot mesh position
        visualizer = self._visualizers.get(mg_id)
        if visualizer is not None:
            visualizer.log_robot_geometry(joint_position=joints)

        # TCP trail: prefer the actual TCP pose reported by the robot (honours
        # the active/jogged TCP offset). Fall back to DH flange FK if no pose.
        tcp_pos: list[float] | None = None
        pose = getattr(state, "pose", None)
        if pose is not None:
            tcp_pos = list(pose.position)
        else:
            dh_robot = self._dh_robots.get(mg_id)
            if dh_robot is not None:
                tcp_pos = dh_robot.calculate_joint_positions(joints)[-1]
        if tcp_pos is not None:
            trail = self._tcp_trail[mg_id]
            trail.append(tcp_pos)
            if len(trail) > self._max_trail_points:
                trail.pop(0)
            if len(trail) >= _MIN_LINE_STEPS:
                rr.log(
                    f"policy/{mg_id}/tcp_trail",
                    rr.LineStrips3D(
                        [trail],
                        colors=[_TCP_TRAIL_COLOR],
                        radii=rr.components.Radius.ui_points(_TRAIL_WIDTH_UI),
                    ),
                )
            rr.log(
                f"policy/{mg_id}/tcp",
                rr.Points3D(
                    [tcp_pos],
                    colors=[_TCP_TRAIL_COLOR],
                    radii=rr.components.Radius.ui_points(4.0),
                ),
            )

        # Log joint scalars on continuous timeline
        for i, j in enumerate(joints):
            rr.log(f"policy/{mg_id}/joints/j{i}", rr.Scalars(j))
