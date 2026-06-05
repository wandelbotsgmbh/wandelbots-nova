"""Server jogger-clock synchronization for waypoint jogging.

The NOVA server exposes ``jogger_session_timestamp_ms`` in the state stream.
``JoggingTimeClock`` observes it, compares to the client wall-clock, and derives
a speed ratio used to scale outgoing waypoint timestamps so the robot moves at
real-time speed regardless of the server's internal rate.
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
    _client_start_time: float = field(default=0.0, repr=False)

    def start(self) -> None:
        """Mark the client-side session start time."""
        self._client_start_time = time.monotonic()

    @property
    def client_elapsed_ms(self) -> int:
        """Client wall-clock elapsed since session start."""
        if self._client_start_time == 0.0:
            return 0
        return int((time.monotonic() - self._client_start_time) * 1000)

    def update(self, timestamp_ms: int) -> None:
        """Feed a new ``jogger_session_timestamp_ms`` reading from the state stream."""
        if timestamp_ms <= 0:
            return
        if not self.synced:
            self.synced = True
            logger.info(
                "Server time sync established (jogger_session_timestamp_ms=%d)", timestamp_ms
            )
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
