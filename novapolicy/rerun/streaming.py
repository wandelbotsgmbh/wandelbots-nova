"""Continuous state streaming — logs robot position between policy steps."""

from __future__ import annotations

import asyncio
import contextlib
import logging
import time
from typing import TYPE_CHECKING, Any

from novapolicy.rerun.constants import MIN_LINE_STEPS, TCP_TRAIL_COLOR, TRAIL_WIDTH_UI

if TYPE_CHECKING:
    from collections.abc import Callable

    from nova.types import RobotState
    from rerun import RecordingStream

logger = logging.getLogger(__name__)

_STATE_STREAM_PERIOD_S = 1.0 / 30.0
_IMAGE_STREAM_PERIOD_S = 1.0 / 15.0


class StateStreamer:
    """Stream robot state and the latest camera frames to Rerun.

    Robot visualization is capped at 30 Hz so it cannot starve camera updates
    or overwhelm the live Rerun viewer. Camera frames are logged at 15 Hz
    independently of the policy inference cadence.
    """

    def __init__(
        self,
        *,
        start_time: float,
        dh_robots: dict[str, Any],
        visualizers: dict[str, Any],
        tcp_trail: dict[str, list[list[float]]],
        max_trail_points: int,
        recording: RecordingStream | None = None,
        image_reader: Callable[[], dict[str, Any]] | None = None,
    ) -> None:
        self._start_time = start_time
        self._dh_robots = dh_robots
        self._visualizers = visualizers
        self._tcp_trail = tcp_trail
        self._max_trail_points = max_trail_points
        self._recording = recording
        self._image_reader = image_reader
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
        """Log live state and camera frames without blocking policy execution."""
        import rerun as rr  # noqa: PLC0415

        next_image_time = 0.0
        try:
            while self._running:
                if self._sessions is None:
                    break

                elapsed = time.monotonic() - self._start_time
                rr.set_time("policy_time", duration=elapsed, recording=self._recording)
                self._tick_counter += 1
                rr.set_time(
                    "state_tick",
                    sequence=self._tick_counter,
                    recording=self._recording,
                )

                for mg_id, session in self._sessions.items():
                    state = session.current_state
                    if state is not None:
                        self._log_state(mg_id, state)

                if self._image_reader is not None and elapsed >= next_image_time:
                    try:
                        self._log_images(self._image_reader())
                    except (OSError, RuntimeError, ValueError, TypeError) as e:
                        logger.debug("Camera frame logging skipped: %s", e)
                    next_image_time = elapsed + _IMAGE_STREAM_PERIOD_S

                await asyncio.sleep(_STATE_STREAM_PERIOD_S)
        except asyncio.CancelledError:
            # Expected on shutdown; stop quietly without logging as an error.
            pass
        except (OSError, RuntimeError) as e:
            logger.debug("State stream logging stopped: %s", e)

    def _log_images(self, images: dict[str, Any]) -> None:
        if not images:
            return
        from novapolicy.rerun.images import log_images  # noqa: PLC0415

        log_images(
            images,
            start_time=self._start_time,
            recording=self._recording,
        )

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
            if len(trail) >= MIN_LINE_STEPS:
                rr.log(
                    f"policy/{mg_id}/tcp_trail",
                    rr.LineStrips3D(
                        [trail],
                        colors=[TCP_TRAIL_COLOR],
                        radii=rr.components.Radius.ui_points(TRAIL_WIDTH_UI),
                    ),
                    recording=self._recording,
                )
            rr.log(
                f"policy/{mg_id}/tcp",
                rr.Points3D(
                    [tcp_pos],
                    colors=[TCP_TRAIL_COLOR],
                    radii=rr.components.Radius.ui_points(4.0),
                ),
                recording=self._recording,
            )

        # Log joint scalars on continuous timeline
        for i, j in enumerate(joints):
            rr.log(
                f"policy/{mg_id}/joints/j{i}",
                rr.Scalars(j),
                recording=self._recording,
            )
