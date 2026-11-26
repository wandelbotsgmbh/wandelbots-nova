# 0017 One-step chunks are spaced by a playback clock

Status: accepted. Recorded 2026-11-23 from the code (`transport/session.py`, `_PlaybackClock`);
the decision is older. The contract is T4; ADR 0003 names the clock and ADR 0004 counts it as the
one bounded retiming. Since 2026-11-24, `Timestamps(grid_ms=...)` can resample these waypoints
onto NOVA's cycle grid (ADR 0021, opt-in).

**Context.** A one-step policy (RL) sends one target per observation, dated a lead after the state
it was computed from. Over a network the state stream is jittery and bursty: evenly spaced targets
came out 0 to 120 ms apart, the follower turned each gap into a speed jump and the arm zig-zagged.
The control loop also runs slower than the policy step over a network (27 instead of 30 Hz).

**Decision.** Waypoints of one-step chunks are spaced as a streaming jitter buffer plays them out,
as FANUC Stream Motion does and as Isaac ROS spaces steps by `step_dt`:

- Each waypoint is due one step after the previous one, nudged toward "observation + lead" by at
  most `_PLAYBACK_SLEW` (10%) of a step, so the commanded speed varies by at most that much.
- The step is the measured time between observations, averaged slowly (`_PLAYBACK_RATE_GAIN`),
  starting from the policy's step: a clock stepping by the nominal step while the loop runs slower
  falls behind and uses up the lead.
- More than `_PLAYBACK_RESYNC_MS` (100 ms) from "observation + lead" (a stall, a burst), or too
  close to now for the controller to plan (`SINGLE_STEP_MARGIN_MS`), the clock starts again from
  the observation, at least half a step after the previous waypoint so timestamps keep increasing.
- A multi-step chunk carries its own spacing and resets the clock.

**Consequences.** State-stream jitter no longer reaches the follower as speed jumps: the commanded
waypoints are evenly spaced, up to 10% off the step. A stall longer than 100 ms shows as one jump
back onto the observations instead of a growing lag. The executed speed still pulses around the
policy's pace, and that is the follower's, not the spacing's: over a VPN (2026-11-24), spacing the
waypoints exactly one step apart cut the commanded speed ripple of the shoulder (0.139 to
0.115 rad/s) but not the executed one (0.183 to 0.185 rad/s).
