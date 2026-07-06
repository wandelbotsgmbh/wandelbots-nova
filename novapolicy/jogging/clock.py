"""Server jogger-clock synchronization for waypoint jogging.

The NOVA server exposes ``session_timestamp_ms`` in the state stream
(field on ``ActionChunkStreamingDetails``). ``JoggingTimeClock`` observes it,
compares to the client wall-clock, and derives a speed ratio used to scale
outgoing waypoint timestamps so the robot moves at real-time speed regardless
of the server's internal rate.
"""

from __future__ import annotations

from dataclasses import dataclass, field
import logging
import time

logger = logging.getLogger(__name__)


@dataclass
class JoggingTimeClock:
    """Tracks the server's jogger session clock and computes the speed ratio.

    The server exposes ``session_timestamp_ms`` in the state stream
    (field on ``ActionChunkStreamingDetails``). It starts at 0 after
    ``InitializeActionChunksRequest`` and increments while waypoints are being
    executed.

    This class observes that timestamp, compares it to the client's wall-clock
    elapsed time, and derives the speed ratio (server_time / client_time).
    The ratio is used to scale outgoing waypoint timestamps so that the
    robot moves at real-time speed regardless of the server's internal rate.

    .. note::
        This scaling is **required**, not optional. On **virtual robots** the
        server's jogger clock advances faster than wall-clock, so outgoing
        waypoint timestamps must be scaled by ``speed_ratio`` or the robot
        races ahead / stalls. On **real robots** the server timer closely
        tracks wall-clock, so the ratio stays near ``1.0`` and scaling is
        effectively a no-op.
    """

    speed_ratio: float = 1.0
    synced: bool = False
    max_lookahead_ms: float = 250.0
    """Cap on how far "now" may run ahead of the last acknowledged server time.

    Set to the current chunk horizon (``len(steps) * dt``) by the session, so a
    stalled connection lets the timeline drift at most one lookahead window
    before it freezes — see :attr:`acknowledged_elapsed_ms`.
    """
    _client_start_time: float = field(default=0.0, repr=False)
    _last_server_ts_ms: int = field(default=0, repr=False)
    _last_server_wall: float = field(default=0.0, repr=False)
    _stalled: bool = field(default=False, repr=False)

    def start(self) -> None:
        """Mark the client-side session start time."""
        self._client_start_time = time.monotonic()

    @property
    def client_elapsed_ms(self) -> int:
        """Client wall-clock elapsed since session start."""
        if self._client_start_time == 0.0:
            return 0
        return int((time.monotonic() - self._client_start_time) * 1000)

    @property
    def acknowledged_elapsed_ms(self) -> int:
        """Session "now" driven by acknowledged server progress, not wall-clock.

        The free-running wall clock keeps advancing even when a weak connection
        stalls the jogging stream, so anchoring waypoints (and time-parameterised
        targets) on wall time makes them race ahead of what the robot has
        actually executed — then jump when the backlog finally lands.

        Instead this returns the last acknowledged server time (converted back to
        client-time via the speed ratio) plus the wall time elapsed since that
        acknowledgement, but the extrapolation is **capped** at
        :attr:`max_lookahead_ms`. On a healthy link the last reading is fresh, so
        this tracks wall-clock to within a state-stream tick (no behaviour
        change). When the stream stalls the extrapolation saturates at the cap
        and "now" freezes — so content and anchors stop advancing past what the
        robot has confirmed, and there is no catch-up jump on recovery.

        Before the first server timestamp arrives the clock is unsynced and this
        falls back to plain wall-clock elapsed.
        """
        if not self.synced or self.speed_ratio <= 0.0:
            return self.client_elapsed_ms
        acknowledged_client_ms = self._last_server_ts_ms / self.speed_ratio
        drift_ms = (time.monotonic() - self._last_server_wall) * 1000.0
        if drift_ms >= self.max_lookahead_ms:
            self._note_stall(drift_ms)
        drift_ms = min(max(0.0, drift_ms), self.max_lookahead_ms)
        return int(acknowledged_client_ms + drift_ms)

    def _note_stall(self, drift_ms: float) -> None:
        """Warn once when the server timer stops advancing (edge-triggered).

        Fires when no fresh ``session_timestamp_ms`` has arrived for
        longer than one lookahead window, i.e. the jog clock has frozen at the
        cap. Recovery is logged from :meth:`update` when server time resumes.
        """
        if not self._stalled:
            self._stalled = True
            logger.warning(
                "Jogging connection stalled: no server timestamp for %.0f ms "
                "(> %.0f ms lookahead) — jog clock frozen until it resumes.",
                drift_ms,
                self.max_lookahead_ms,
            )

    def update(self, timestamp_ms: int) -> None:
        """Feed a new ``session_timestamp_ms`` reading from the state stream."""
        if timestamp_ms <= 0:
            return
        if not self.synced:
            self.synced = True
            logger.info(
                "Server time sync established (session_timestamp_ms=%d)", timestamp_ms
            )
        if self._stalled:
            self._stalled = False
            logger.info(
                "Jogging connection recovered (session_timestamp_ms=%d); "
                "server time advancing again.",
                timestamp_ms,
            )
        # Record the latest acknowledged server time and when we saw it, so
        # acknowledged_elapsed_ms can extrapolate from a real ack rather than
        # from a free-running wall clock.
        self._last_server_ts_ms = timestamp_ms
        self._last_server_wall = time.monotonic()
        # Use raw ratio (server_time / client_time) directly.
        # Clamp >= 1.0 since the server is never slower than wall-clock.
        client_ms = self.client_elapsed_ms
        if client_ms > 0:
            self.speed_ratio = max(1.0, timestamp_ms / client_ms)

    def scale_timestamp(self, trajectory_time_ms: int) -> int:
        """Convert a trajectory-time timestamp to server-time."""
        return int(trajectory_time_ms * self.speed_ratio)

    def scale_dt(self, dt_ms: float) -> float:
        """Convert a trajectory-time dt to server-time."""
        return dt_ms * self.speed_ratio

    @staticmethod
    def extract_from_state(state: object) -> int | None:
        """Extract the action-chunk session timestamp from a MotionGroupState.

        Reads ``execute.details.session_timestamp_ms`` (the field on
        ``ActionChunkStreamingDetails``). Returns None if not present.
        """
        execute = getattr(state, "execute", None)
        if execute is None:
            return None
        details = getattr(execute, "details", None)
        if details is None:
            return None
        ts = getattr(details, "session_timestamp_ms", None)
        if isinstance(ts, int):
            return ts
        return None
