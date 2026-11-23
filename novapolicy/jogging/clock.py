"""Server jogger-clock tracking for waypoint jogging.

The NOVA server exposes ``jogger_session_timestamp_ms`` in the state stream.
``JoggingTimeClock`` observes it alongside the client monotonic clock so server
"now" can be extrapolated between state samples.

Server jogger milliseconds are treated as **real milliseconds**: waypoint
intervals are never rescaled. Deriving a server/client rate ratio and scaling by
it is the tempting alternative, and it is wrong — on a UR10e that ratio settles
near 1.09 and stretches the timeline, slowing all motion by that proportion,
where an unscaled timeline executes at exactly the commanded speed.
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
"""A time on the jogger session's clock: milliseconds since the server started executing the
session's first chunk (``jogger_session_timestamp_ms``). Every waypoint timestamp is one. Not
the client's ``time.monotonic()`` and not wall-clock time; convert only through
:class:`JoggingTimeClock`."""

# Number of continuously advancing state-envelope timestamps observed before
# freezing the server/client UTC skew estimate. At a 10ms state rate this adds
# about 200ms to startup and ensures a late first packet cannot bias every
# waypoint timestamp in the session.
_STATE_CLOCK_CALIBRATION_SAMPLES = 20

# Longest calibration may hold startup up: at a slow ``state_rate_ms`` the sample count above
# would outlast ``wait_ready``'s timeout. Past it the estimate is frozen with what it has.
_STATE_CLOCK_CALIBRATION_WINDOW_S = 1.0

# Largest delivery delay that may be attributed to a single state packet. The delay comes
# from wall-clock readings, so an NTP step or an oddly stamped packet inflates it and pushes
# estimated server time ahead, where the never-backwards trajectory clock latches it and the
# robot holds still; bounding it bounds that stall. Kept under ``max_lookahead_ms``.
_MAX_DELIVERY_DELAY_MS = 200.0

_PLACEHOLDER_YEAR = 2000
"""State timestamps before this year are placeholders: controllers that send no timestamp
(FANUC, Staubli, Techman, Yaskawa) arrive stamped 1970-01-01. Taken at face value, a constant
stamp looks older with every packet and is read as ever more delivery delay."""


def _usable(state_timestamp: datetime | None) -> datetime | None:
    """The state timestamp, or ``None`` when it is a placeholder."""
    if state_timestamp is None or state_timestamp.year < _PLACEHOLDER_YEAR:
        return None
    return state_timestamp


@dataclass
class JoggingTimeClock:
    """Tracks and extrapolates the server's jogger session clock.

    The server exposes ``jogger_session_timestamp_ms`` in the state stream
    (field on ``JoggingDetails``). It starts at 0 after
    ``InitializeActionChunksRequest`` and increments while waypoints are being
    executed.

    This class observes that timestamp alongside client monotonic time so that
    server "now" can be extrapolated between state samples. Timestamps are used
    as-is: one server millisecond is one millisecond of motion.
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

        Satisfied by :data:`_STATE_CLOCK_CALIBRATION_SAMPLES` samples, or by
        :data:`_STATE_CLOCK_CALIBRATION_WINDOW_S` elapsing with at least one, so
        that a slow state rate cannot hold startup past its own timeout.
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
        """Monotonic instant the latest state was *generated*, or ``None``.

        Receipt time is not it: packets arrive in bursts, so a dozen states
        generated 8ms apart can land within 3ms of each other. Anything placing
        a state on a time axis has to use this, or a burst is drawn as a spike
        preceded by a gap.

        ``None`` until the first jogger timestamp arrives. Callers must not
        substitute the raw field for it: it is zero-initialised, and zero in the
        monotonic domain is machine boot, which lands a good week before the
        session started.
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
        """Estimate the server clock at the current wall-clock instant.

        The latest state-stream timestamp is extrapolated with the measured
        server-clock rate. This avoids treating an already-aged state sample as
        server "now" when timestamping a waypoint request.
        """
        if not self.synced:
            # Before the first server timestamp, the server clock starts with the first send.
            return ServerMs(self.client_elapsed_ms)
        drift_ms = (time.monotonic() - self._last_server_wall) * 1000.0
        if drift_ms >= self.max_lookahead_ms:
            self._note_stall(drift_ms)
        return ServerMs(self._last_server_ts_ms + int(max(0.0, drift_ms)))

    @property
    def acknowledged_elapsed_ms(self) -> int:
        """Estimated session "now" based on the latest acknowledged server state.

        The latest jogger timestamp is extrapolated from the state generation
        time, not from when its packet reached this process. Before the first
        server timestamp arrives, the clock falls back to client elapsed time.

        The name notwithstanding, the value extrapolates *beyond* the last
        acknowledgement: ``max_lookahead_ms`` is a stall-warning threshold, not a
        cap on it. Capping it deadlocks startup, because the jogger timer only
        advances once waypoints execute, and waypoints can only be placed once
        this advances.
        """
        if not self.synced:
            return self.client_elapsed_ms
        return self.estimated_server_timestamp_ms

    def _note_stall(self, drift_ms: float) -> None:
        """Warn once when the server timer stops advancing (edge-triggered).

        Fires when no fresh ``jogger_session_timestamp_ms`` has arrived for
        longer than one lookahead window. Recovery is logged from :meth:`update`.
        """
        if not self._stalled:
            self._stalled = True
            logger.warning(
                "Jogging connection stalled: no server timestamp for %.0f ms "
                "(> %.0f ms lookahead) — extrapolating until it resumes.",
                drift_ms,
                self.max_lookahead_ms,
            )

    def update(self, timestamp_ms: int, state_timestamp: datetime | None = None) -> None:
        """Feed a jogger timestamp and its server-generated state timestamp.

        ``state_timestamp`` identifies when the state was generated. State
        messages can arrive appreciably late — over 100ms on the deployment this
        was developed against; treating receipt time as generation time drags the
        estimated jogger clock backwards by that delay, which the trajectory clock
        can only absorb by freezing (visible as a ramp-and-snap tracking error).

        Server and client UTC clocks can have a constant offset. The smallest
        observed ``local UTC - state UTC`` is the least-delayed sample and thus
        our clock-skew estimate; any excess age on this sample is delivery delay.
        We backdate the monotonic reference by that delay, capped at
        :data:`_MAX_DELIVERY_DELAY_MS` so a non-monotonic wall clock cannot push
        the estimate arbitrarily far ahead. No bound is placed on future
        extrapolation, so a session that has not started cannot deadlock.

        Calibration is *not* driven from here: the caller must pass every state
        packet through :meth:`observe_state_timestamp`, including the ones with
        no jogger timestamp yet. Observing again here would count each packet
        twice and declare the skew estimate frozen after half the samples it is
        meant to see.
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
            # Repeated or out-of-order state samples do not move the clock
            # reference forward. Aging the reference here would make the
            # estimated server clock lag behind reality.
            return

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


SINGLE_STEP_MARGIN_MS = 67.0
"""What the controller needs, beyond the network, to plan a smooth approach to a one-step
waypoint (33 ms ahead was rough on a virtual UR10e, 67 ms smooth)."""
