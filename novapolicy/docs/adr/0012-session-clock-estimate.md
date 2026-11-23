# 0012 The session clock is estimated from generation time and never rescaled

Status: accepted. Recorded 2026-11-23 from the code (`transport/clock.py`); the decision is older.
The estimate itself is described in [executor.md](../executor.md#timestamp-protocol).

**Context.** Every waypoint is dated on NOVA's session clock (`jogger_session_timestamp_ms`), which
the client sees only through state packets. Those arrive late (over 100 ms on the deployment this
was developed against) and in bursts: a dozen states generated 8 ms apart can land within 3 ms.
Server and client wall clocks differ by a constant offset.

**Decision.**

- One server millisecond is one millisecond of motion: intervals are never rescaled.
- "Now" is the last acknowledged timestamp plus the client time since that state was
  *generated*, not received. Anything placing a state on a time axis uses the generation instant.
- The skew is the smallest `local UTC - state UTC` seen; a packet's excess age is its delivery
  delay. The estimate is frozen after a number of continuously advancing samples (about 200 ms at
  a 10 ms state rate), or after a time window, so a slow `state_rate_ms` cannot outlast
  `wait_ready`'s timeout. Only `observe_state_timestamp` calibrates: counting a packet in
  `update` too would freeze the estimate after half its samples.
- The delay attributed to one packet is capped (`_MAX_DELIVERY_DELAY_MS`, below
  `max_lookahead_ms`): it comes from wall-clock readings, so an NTP step or an oddly stamped
  packet would push the estimate ahead, where the never-backwards trajectory clock latches it and
  the robot holds still. The cap bounds that stall.
- State timestamps before 2000 are placeholders (FANUC, Stäubli, Techman and Yaskawa controllers
  stamp 1970-01-01); taken at face value a constant stamp reads as ever more delivery delay.
- The extrapolation is not capped. `max_lookahead_ms` only warns: the jogger timer advances only
  once waypoints execute, and waypoints are placed only once the estimate advances, so a cap
  deadlocks startup.

**Alternatives measured.** Scaling by a derived server/client rate ratio: on a UR10e the ratio
settles near 1.09 and stretches the timeline, slowing all motion by that proportion. Treating
receipt time as generation time drags the estimate back by the delivery delay, which the
trajectory clock absorbs only by freezing (a ramp-and-snap tracking error); drawn on a time axis,
a burst becomes a spike after a gap.

**Consequences.** A late first packet cannot bias every waypoint of a session. `last_sample_wall`
is `None` until a jogger timestamp arrives; its zero-initialised field would be machine boot on
the monotonic clock.
