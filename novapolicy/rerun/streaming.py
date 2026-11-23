"""Background logging between policy steps: controller timing and camera frames."""

from __future__ import annotations

import asyncio
import contextlib
import logging
import time
from typing import TYPE_CHECKING, Any

from novapolicy.rerun.constants import STATE_SAMPLE_PERIOD_S
from novapolicy.rerun.context import RerunContext
from novapolicy.rerun.observation import log_state
import rerun as rr

if TYPE_CHECKING:
    from collections.abc import Callable

    from nova.types import RobotState
    from novapolicy.jogging.waypoint_session import WaypointJoggingSession
    from rerun import RecordingStream

logger = logging.getLogger(__name__)

_IMAGE_STREAM_PERIOD_S = 1.0 / 15.0


class StateStreamer:
    """Log controller timing and the latest camera frames to Rerun, and the robot state it is
    handed (see :meth:`PolicyRerunLogger.log_state_packet`).

    Camera frames are logged at 15 Hz independently of the policy inference cadence.
    """

    def __init__(
        self,
        *,
        start_time: float,
        dh_robots: dict[str, Any],
        visualizers: dict[str, Any],
        max_trail_points: int,
        pose_frames: dict[str, Any] | None = None,
        tcp_trail: dict[str, list[list[float]]] | None = None,
        recording: RecordingStream | None = None,
        image_reader: Callable[[], dict[str, Any]] | None = None,
    ) -> None:
        self._ctx = RerunContext(
            start_time=start_time,
            recording=recording,
            dh_robots=dh_robots,
            pose_frames=pose_frames if pose_frames is not None else {},
            visualizers=visualizers,
            max_trail_points=max_trail_points,
        )
        self._tcp_trail = tcp_trail if tcp_trail is not None else {}
        self._image_reader = image_reader
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
        next_image_time = 0.0
        try:  # ruff: ignore[too-many-nested-blocks, too-many-statements-in-try-clause]
            while self._running:
                if self._sessions is None:
                    break

                elapsed = time.monotonic() - self._ctx.start_time
                self._ctx.set_time(elapsed=elapsed)
                self._tick_counter += 1
                rr.set_time(
                    "state_tick", sequence=self._tick_counter, recording=self._ctx.recording
                )

                for mg_id, session in self._sessions.items():
                    self._log_controller_timing(mg_id, session)

                if self._image_reader is not None and elapsed >= next_image_time:
                    try:
                        self._log_images(self._image_reader())
                    except (OSError, RuntimeError, ValueError, TypeError) as e:
                        logger.debug("Camera frame logging skipped: %s", e)
                    next_image_time = elapsed + _IMAGE_STREAM_PERIOD_S

                await asyncio.sleep(STATE_SAMPLE_PERIOD_S)
        except asyncio.CancelledError:
            # Expected on shutdown; stop quietly without logging as an error.
            pass
        except (OSError, RuntimeError) as e:
            logger.debug("State stream logging stopped: %s", e)

    def _log_images(self, images: dict[str, Any]) -> None:
        if not images:
            return
        # Looked up per call, so a test can replace it after construction.
        from novapolicy.rerun.images import log_images  # ruff: ignore[import-outside-top-level]

        log_images(self._ctx, images)

    def _log_controller_timing(self, mg_id: str, session: WaypointJoggingSession) -> None:
        """Log raw controller progress and each scheduled waypoint request."""
        recording = self._ctx.recording
        rr.log(
            f"policy/{mg_id}/controller/server_timestamp_ms",
            rr.Scalars(session.last_server_timestamp_ms),
            recording=recording,
        )

        scheduled = session.scheduled
        sequence = scheduled.chunk_count
        if sequence <= self._logged_scheduled_chunks.get(mg_id, 0):
            return
        self._logged_scheduled_chunks[mg_id] = sequence

        timestamps = list(scheduled.waypoint_timestamps)
        if not timestamps:
            return
        values = {
            "action_timestep": scheduled.action_timestep,
            "scheduled_at_server_ms": scheduled.at_server_ms,
            "first_timestamp_ms": timestamps[0],
            "last_timestamp_ms": timestamps[-1],
            "count": len(timestamps),
            "dt_ms": timestamps[1] - timestamps[0] if len(timestamps) > 1 else 0,
        }
        for name, value in values.items():
            rr.log(
                f"policy/{mg_id}/waypoint_request/{name}", rr.Scalars(value), recording=recording
            )
        rr.log(
            "policy/waypoint_requests",
            rr.TextLog(
                f"{mg_id} | chunk={sequence} | "
                f"action_timestep={scheduled.action_timestep} | "
                f"scheduled_at={scheduled.at_server_ms}ms | "
                f"timestamps_ms={timestamps}",
                level=rr.TextLogLevel.TRACE,
            ),
            recording=recording,
        )

    def log_state(self, mg_id: str, state: RobotState) -> None:
        """Log one state at the time already set on the recording."""
        log_state(self._ctx, mg_id, state, self._tcp_trail.setdefault(mg_id, []))
