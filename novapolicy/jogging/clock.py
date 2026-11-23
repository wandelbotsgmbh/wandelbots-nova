"""Session-clock estimation for waypoint jogging (ADR 0012).

``JoggingTimeClock`` extrapolates NOVA's ``jogger_session_timestamp_ms`` between state samples.
One server millisecond is one millisecond of motion: intervals are never rescaled.
"""

from __future__ import annotations

from dataclasses import dataclass, field
import logging
import time
from typing import TYPE_CHECKING, NewType

if TYPE_CHECKING:
    from datetime import datetime

logger = logging.getLogger(__name__)

ServerMs = NewType("ServerMs", int)
"""Milliseconds on the session clock (``jogger_session_timestamp_ms``, ADR 0008); convert
from client time only through :class:`JoggingTimeClock`."""

# State timestamps observed before the server/client UTC skew estimate is frozen (ADR 0012).
_STATE_CLOCK_CALIBRATION_SAMPLES = 20

# Longest calibration may hold startup up; past it the estimate is frozen with what it has.
_STATE_CLOCK_CALIBRATION_WINDOW_S = 1.0

# Largest delivery delay attributed to one state packet (ADR 0012); under max_lookahead_ms.
_MAX_DELIVERY_DELAY_MS = 200.0

_PLACEHOLDER_YEAR = 2000
"""State timestamps before this year are placeholders (controllers that send none) and are
ignored."""


def _usable(state_timestamp: datetime | None) -> datetime | None:
    """The state timestamp, or ``None`` when it is a placeholder."""
    if state_timestamp is None or state_timestamp.year < _PLACEHOLDER_YEAR:
        return None
    return state_timestamp


@dataclass
class JoggingTimeClock:
    """Tracks and extrapolates the server's jogger session clock.

    ``jogger_session_timestamp_ms`` (on ``JoggingDetails``) starts at 0 after
    ``InitializeActionChunksRequest`` and advances while waypoints execute.
    """

    synced: bool = False
    max_lookahead_ms: float = 250.0
    """How long the server timer may go unheard before warning that the link stalled."""
    _client_start_time: float = field(default=0.0, repr=False)
    _last_server_ts_ms: int = field(default=0, repr=False)
    _last_server_wall: float = field(default=0.0, repr=False)
    _best_state_age_ms: float | None = field(default=None, repr=False)
    _state_age_samples: int = field(default=0, repr=False)
    _first_state_age_wall: float | None = field(default=None, repr=False)
    _stalled: bool = field(default=False, repr=False)

    def start(self) -> None:
        """Mark the client-side session start time."""
        self._client_start_time = time.monotonic()

    @property
    def state_clock_calibrated(self) -> bool:
        """Whether server/client wall-clock skew has been measured and frozen.

        After :data:`_STATE_CLOCK_CALIBRATION_SAMPLES` samples, or
        :data:`_STATE_CLOCK_CALIBRATION_WINDOW_S` with at least one.
        """
        if self._state_age_samples >= _STATE_CLOCK_CALIBRATION_SAMPLES:
            return True
        if self._first_state_age_wall is None:
            return False
        return time.monotonic() - self._first_state_age_wall >= _STATE_CLOCK_CALIBRATION_WINDOW_S

    def observe_state_timestamp(self, state_timestamp: datetime | None) -> None:
        """Observe a state generation timestamp during startup calibration."""
        state_timestamp = _usable(state_timestamp)
        if state_timestamp is None or self.state_clock_calibrated:
            return
        observed_age_ms = time.time() * 1000.0 - state_timestamp.timestamp() * 1000.0
        if self._best_state_age_ms is None:
            self._best_state_age_ms = observed_age_ms
            self._first_state_age_wall = time.monotonic()
        else:
            self._best_state_age_ms = min(self._best_state_age_ms, observed_age_ms)
        self._state_age_samples += 1

    @property
    def last_sample_wall(self) -> float | None:
        """Monotonic instant the latest state was *generated* (not received), or ``None``.

        ``None`` until the first jogger timestamp arrives; do not substitute the raw,
        zero-initialised field for it.
        """
        if not self.synced:
            return None
        return self._last_server_wall

    @property
    def last_server_timestamp_ms(self) -> ServerMs:
        """Latest acknowledged NOVA jogger-session timestamp."""
        return ServerMs(self._last_server_ts_ms)

    @property
    def client_elapsed_ms(self) -> int:
        """Client wall-clock elapsed since session start."""
        if self._client_start_time <= 0.0:
            return 0
        return int((time.monotonic() - self._client_start_time) * 1000)

    @property
    def estimated_server_timestamp_ms(self) -> ServerMs:
        """Session "now": the latest jogger timestamp plus the time since its state was
        generated; client elapsed time before the first one. Not capped by
        ``max_lookahead_ms``, which only warns (ADR 0012)."""
        if not self.synced:
            # Before the first server timestamp, the server clock starts with the first send.
            return ServerMs(self.client_elapsed_ms)
        drift_ms = (time.monotonic() - self._last_server_wall) * 1000.0
        if drift_ms >= self.max_lookahead_ms:
            self._note_stall(drift_ms)
        return ServerMs(self._last_server_ts_ms + int(max(0.0, drift_ms)))

    def _note_stall(self, drift_ms: float) -> None:
        """Warn once when no fresh jogger timestamp arrived for one lookahead window."""
        if not self._stalled:
            self._stalled = True
            logger.warning(
                "Jogging connection stalled: no server timestamp for %.0f ms "
                "(> %.0f ms lookahead) — extrapolating until it resumes.",
                drift_ms,
                self.max_lookahead_ms,
            )

    def update(self, timestamp_ms: int, state_timestamp: datetime | None = None) -> None:
        """Feed a jogger timestamp and the time its state was generated.

        The monotonic reference is backdated by the packet's delivery delay (its age beyond the
        smallest ``local UTC - state UTC`` seen), capped at :data:`_MAX_DELIVERY_DELAY_MS`.
        Callers pass every packet through :meth:`observe_state_timestamp` separately; this does
        not calibrate.
        """
        if timestamp_ms <= 0:
            return
        state_timestamp = _usable(state_timestamp)
        received_wall = time.monotonic()
        sample_wall = received_wall
        if state_timestamp is not None:
            observed_age_ms = time.time() * 1000.0 - state_timestamp.timestamp() * 1000.0
            best_age_ms = (
                observed_age_ms if self._best_state_age_ms is None else self._best_state_age_ms
            )
            delivery_delay_ms = min(_MAX_DELIVERY_DELAY_MS, max(0.0, observed_age_ms - best_age_ms))
            sample_wall -= delivery_delay_ms / 1000.0
        if not self.synced:
            self.synced = True
            logger.info(
                "Server time sync established (jogger_session_timestamp_ms=%d)",
                timestamp_ms,
            )
        elif timestamp_ms <= self._last_server_ts_ms:
            # Repeated or out-of-order samples leave the reference alone.
            return

        if self._stalled:
            self._stalled = False
            logger.info(
                "Jogging connection recovered (jogger_session_timestamp_ms=%d); "
                "server time advancing again.",
                timestamp_ms,
            )
        self._last_server_ts_ms = timestamp_ms
        self._last_server_wall = sample_wall


SINGLE_STEP_MARGIN_MS = 67.0
"""Lead the controller needs, beyond the network, for a smooth one-step waypoint (ADR 0002)."""
