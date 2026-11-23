# 0013 The live buffer replays measured targets on a fixed grid

Status: accepted. Recorded 2026-11-23 from the code (`pipeline/modules/live_buffer.py`);
the decision is older. What it does and its measurements: [jogging.md](../jogging.md#live-targets-and-the-replay-buffer-livebuffer),
ADR 0004 and ADR 0009.

**Context.** A live target says only where the target is now, and a lone waypoint is a terminal
target the server brakes to, so one target per tick makes the robot stop and restart. Dense
targets milliseconds apart turn into speed spikes in the follower.

**Decision.**

- Replay, never predict: the horizon is the last `window_ms` of measured targets, interpolated at
  the instant each was taken. The cost is latency, about the window.
- One timeline: the trajectory clock is quantised to `TIMELINE_GRID_MS`, it maps to the session
  clock with a constant lead (`LIVE_LEAD_MS`), and the replay grid sits on absolute multiples of
  its spacing instead of starting at the oldest surviving sample, which changes with every prune.
  A target keeps its absolute timestamp however often the window is re-sent; moving commanded
  points to new times executes as a lurch.
- The trajectory clock runs on the server clock (wall time drifts from it), is quantised and never
  runs backwards (a backward step commands a reversal). Forward jumps are kept: they are the clock
  correcting after a late state read, and limiting them lets the timeline fall behind until every
  waypoint is in the past.
- The resampled span is rounded up, with an epsilon for float division, so the grid reaches the
  newest sample, the leading edge of the horizon. A grid point past it holds its value: the target
  stopped, rather than a guess where it went.
- Too little history sends the bare target instead of waiting, which would never move.
- `MAX_LIVE_SAMPLES` caps the buffer besides the window: the trajectory clock holds at zero until
  the robot executes, so before the session starts or while its state stream stalls nothing ages
  out. A 100 Hz caller fills a 500 ms window with about 50 samples.

**Consequences.** The window is the whole horizon, and the server caps its speed at what it can
brake within, so the window has to be a few hundred milliseconds. Too little lead shows as brief
stalls and catch-ups. `LIVE_LEAD_MS` (100 ms, the joggers' `LEAD_MS` for chunks too) was tuned on
one UR10e; re-tune it per robot and link. The grid spacing is `WaypointConfig.single_step_dt_ms`, configuration
rather than a constant because which spacing executes most smoothly depends on the controller,
and never coarser than half the buffered span, so a short buffer keeps more than its endpoints.
