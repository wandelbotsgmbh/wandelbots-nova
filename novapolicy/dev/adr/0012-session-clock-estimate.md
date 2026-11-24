# 0012 The session clock is estimated from generation time and never rescaled

Status: accepted. Recorded 2026-11-23 from the code (`transport/clock.py`); the decision is older.
The estimate itself is described in [executor.md](../../docs/executor.md#timestamp-protocol).

**Context.** Every waypoint is dated on NOVA's session clock (`jogger_session_timestamp_ms`), which
the client sees only through state packets. Those arrive late (over 100 ms on the deployment this
was developed against) and in bursts: a dozen states generated 8 ms apart can land within 3 ms.
Server and client wall clocks differ by a constant offset.

**Decision.**

- One server millisecond is one millisecond of motion: intervals are never rescaled.
- "Now" is the last acknowledged timestamp plus the client time since that state was
  *generated*, not received.
- The wall-clock skew is calibrated over the first packets of a session, then held; the delay
  attributed to one packet is capped (`_MAX_DELIVERY_DELAY_MS`), so a wall-clock step cannot push
  the estimate far ahead.
- The extrapolation is not capped: the jogger timer advances only once waypoints execute, and
  waypoints are placed only once the estimate advances, so a cap deadlocks startup.
  `max_lookahead_ms` only warns.

How the estimate is calibrated, and which controller timestamps it ignores, is in
[executor.md](../../docs/executor.md#timestamp-protocol).

**Alternatives measured.** Scaling by a derived server/client rate ratio: on a UR10e the ratio
settles near 1.09 and stretches the timeline, slowing all motion by that proportion. Treating
receipt time as generation time drags the estimate back by the delivery delay, which the
trajectory clock absorbs only by freezing (a ramp-and-snap tracking error); drawn on a time axis,
a burst becomes a spike after a gap.

**Consequences.** A late first packet cannot bias every waypoint of a session, and a burst of
packets does not move the timeline.
