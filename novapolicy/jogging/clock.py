"""Server jogger-clock synchronization for waypoint jogging.

The NOVA server exposes ``jogger_session_timestamp_ms`` in the state stream.
``JoggingTimeClock`` observes it alongside the client monotonic clock and derives
a clock-rate ratio from timestamp deltas. That ratio is used to extrapolate
server "now" and scale waypoint intervals.
"""

from __future__ import annotations

from dataclasses import dataclass, field
import logging
import time

logger = logging.getLogger(__name__)


@dataclass
class JoggingTimeClock:
    """Tracks the server's jogger session clock and computes the speed ratio.

    The server exposes ``jogger_session_timestamp_ms`` in the state stream
    (field on ``JoggingDetails``). It starts at 0 after ``InitializeJoggingRequest``
    and increments while waypoints are being executed.

    This class observes that timestamp alongside client monotonic time and
    derives the speed ratio from deltas between clock samples.
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
    _rate_reference_server_ts_ms: int = field(default=0, repr=False)
    _rate_reference_wall: float = field(default=0.0, repr=False)
    _stalled: bool = field(default=False, repr=False)

    def start(self) -> None:
        """Mark the client-side session start time."""
        self._client_start_time = time.monotonic()

    @property
    def last_server_timestamp_ms(self) -> int:
        """Latest acknowledged NOVA jogger-session timestamp."""
        return self._last_server_ts_ms

    @property
    def client_elapsed_ms(self) -> int:
        """Client wall-clock elapsed since session start."""
        if self._client_start_time == 0.0:
            return 0
        return int((time.monotonic() - self._client_start_time) * 1000)

    @property
    def estimated_server_timestamp_ms(self) -> int:
        """Estimate the server clock at the current wall-clock instant.

        The latest state-stream timestamp is extrapolated with the measured
        server-clock rate. This avoids treating an already-aged state sample as
        server "now" when timestamping a waypoint request.
        """
        if not self.synced:
            return self.scale_timestamp(self.client_elapsed_ms)
        drift_ms = (time.monotonic() - self._last_server_wall) * 1000.0
        if drift_ms >= self.max_lookahead_ms:
            self._note_stall(drift_ms)
        drift_ms = min(max(0.0, drift_ms), self.max_lookahead_ms)
        return self._last_server_ts_ms + int(drift_ms * self.speed_ratio)

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
        return int(self.estimated_server_timestamp_ms / self.speed_ratio)

    def _note_stall(self, drift_ms: float) -> None:
        """Warn once when the server timer stops advancing (edge-triggered).

        Fires when no fresh ``jogger_session_timestamp_ms`` has arrived for
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
        """Feed a new ``jogger_session_timestamp_ms`` reading from the state stream."""
        if timestamp_ms <= 0:
            return
        sample_wall = time.monotonic()
        if not self.synced:
            self.synced = True
            self._rate_reference_server_ts_ms = timestamp_ms
            self._rate_reference_wall = sample_wall
            logger.info(
                "Server time sync established (jogger_session_timestamp_ms=%d)", timestamp_ms
            )
        elif timestamp_ms <= self._last_server_ts_ms:
            # Repeated or out-of-order state samples do not move the clock
            # reference forward. Aging the reference here would make the
            # estimated server clock lag behind reality.
            return
        else:
            server_delta_ms = timestamp_ms - self._rate_reference_server_ts_ms
            wall_delta_ms = (sample_wall - self._rate_reference_wall) * 1000.0
            if wall_delta_ms > 0.0:
                self.speed_ratio = max(1.0, server_delta_ms / wall_delta_ms)

        if self._stalled:
            self._stalled = False
            logger.info(
                "Jogging connection recovered (jogger_session_timestamp_ms=%d); "
                "server time advancing again.",
                timestamp_ms,
            )
        # Record both domains from the same sample. Future server "now" values
        # are extrapolated from this pair rather than assuming that the client
        # and server clocks started at the same instant.
        self._last_server_ts_ms = timestamp_ms
        self._last_server_wall = sample_wall

    def scale_timestamp(self, trajectory_time_ms: int) -> int:
        """Convert a trajectory-time timestamp to server-time."""
        return int(trajectory_time_ms * self.speed_ratio)

    def scale_dt(self, dt_ms: float) -> float:
        """Convert a trajectory-time dt to server-time."""
        return dt_ms * self.speed_ratio

    @staticmethod
    def extract_from_state(state: object) -> int | None:
        """Extract jogger_session_timestamp_ms from a MotionGroupState, or None."""
        execute = getattr(state, "execute", None)
        if execute is None:
            return None
        details = getattr(execute, "details", None)
        if details is None:
            return None
        ts = getattr(details, "jogger_session_timestamp_ms", None)
        if isinstance(ts, int):
            return ts
        return None
