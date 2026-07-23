"""Continuous state streaming — logs robot position between policy steps."""

from __future__ import annotations

import asyncio
import contextlib
import logging
import math
import time
from typing import TYPE_CHECKING, Any

from novapolicy.rerun.constants import MIN_LINE_STEPS, TCP_TRAIL_COLOR, TRAIL_WIDTH_UI

if TYPE_CHECKING:
    from collections.abc import Callable

    from nova.types import RobotState
    from novapolicy.jogging.waypoint_session import WaypointJoggingSession
    from rerun import RecordingStream

logger = logging.getLogger(__name__)

_DEFAULT_STATE_SAMPLE_INTERVAL_MS = 1000.0 / 30.0
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
        state_sample_interval_ms: float = _DEFAULT_STATE_SAMPLE_INTERVAL_MS,
    ) -> None:
        self._start_time = start_time
        self._dh_robots = dh_robots
        self._visualizers = visualizers
        self._tcp_trail = tcp_trail
        self._max_trail_points = max_trail_points
        self._recording = recording
        self._image_reader = image_reader
        if not math.isfinite(state_sample_interval_ms) or state_sample_interval_ms <= 0:
            raise ValueError("state_sample_interval_ms must be a positive finite value")
        self._state_stream_period_s = state_sample_interval_ms / 1000.0
        self._sessions: dict[str, WaypointJoggingSession] | None = None
        self._task: asyncio.Task[None] | None = None
        self._running = False
        self._tick_counter = 0
        self._logged_scheduled_chunks: dict[str, int] = {}

    def start(self, sessions: dict[str, WaypointJoggingSession]) -> None:
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
        try:  # noqa: PLR1702, PLW0717
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
                    self._log_controller_timing(mg_id, session)

                if self._image_reader is not None and elapsed >= next_image_time:
                    try:
                        self._log_images(self._image_reader())
                    except (OSError, RuntimeError, ValueError, TypeError) as e:
                        logger.debug("Camera frame logging skipped: %s", e)
                    next_image_time = elapsed + _IMAGE_STREAM_PERIOD_S

                await asyncio.sleep(self._state_stream_period_s)
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

    def _log_controller_timing(self, mg_id: str, session: WaypointJoggingSession) -> None:
        """Log raw controller progress and each scheduled waypoint request."""
        import rerun as rr  # noqa: PLC0415

        server_timestamp_ms = session.last_server_timestamp_ms
        rr.log(
            f"policy/{mg_id}/controller/server_timestamp_ms",
            rr.Scalars(server_timestamp_ms),
            recording=self._recording,
        )

        sequence = session.scheduled_chunk_count
        if sequence <= self._logged_scheduled_chunks.get(mg_id, 0):
            return
        self._logged_scheduled_chunks[mg_id] = sequence

        timestamps = list(session.scheduled_waypoint_timestamps)
        if not timestamps:
            return
        first_timestamp_ms = timestamps[0]
        last_timestamp_ms = timestamps[-1]
        server_dt_ms = timestamps[1] - timestamps[0] if len(timestamps) > 1 else 0
        values = {
            "action_timestep": session.scheduled_action_timestep,
            "scheduled_at_server_ms": session.scheduled_at_server_ms,
            "first_timestamp_ms": first_timestamp_ms,
            "last_timestamp_ms": last_timestamp_ms,
            "count": len(timestamps),
            "dt_ms": server_dt_ms,
        }
        for name, value in values.items():
            rr.log(
                f"policy/{mg_id}/waypoint_request/{name}",
                rr.Scalars(value),
                recording=self._recording,
            )
        rr.log(
            "policy/waypoint_requests",
            rr.TextLog(
                f"{mg_id} | chunk={sequence} | "
                f"action_timestep={session.scheduled_action_timestep} | "
                f"scheduled_at={session.scheduled_at_server_ms}ms | "
                f"timestamps_ms={timestamps}",
                level=rr.TextLogLevel.TRACE,
            ),
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
