# 0013 The live buffer replays measured targets on a fixed grid

Status: accepted. Recorded 2026-11-23 from the code (`pipeline/modules/live_buffer.py`);
the decision is older. What it does and its measurements: [jogging.md](../../docs/jogging.md#live-targets-and-the-replay-buffer-livebuffer),
ADR 0004 and ADR 0009.

**Context.** A live target says only where the target is now, and a lone waypoint is a terminal
target the server brakes to, so one target per tick makes the robot stop and restart. Dense
targets milliseconds apart turn into speed spikes in the follower.

**Decision.**

- Replay, never predict: the horizon is the last `window_ms` of measured targets, interpolated at
  the instant each was taken. The cost is latency, about the window.
- One timeline: the replay grid sits on absolute multiples of its spacing, on a trajectory clock
  that runs on the server clock and never runs backwards. A target keeps its absolute timestamp
  however often the window is re-sent; moving commanded points to new times executes as a lurch,
  and a backward step commands a reversal.

How the buffer handles its edges (rounding, too little history, its sample cap, the lead) is in
[jogging.md](../../docs/jogging.md#live-targets-and-the-replay-buffer-livebuffer).

**Consequences.** The window is the whole horizon, and the server caps its speed at what it can
brake within, so the window has to be a few hundred milliseconds. Too little lead shows as brief
stalls and catch-ups; `LIVE_LEAD_MS` (100 ms) was tuned on one UR10e and needs re-tuning per robot
and link.

**2026-11-24: the grid ends at the newest target.** The replay grid reached up to one step past
the newest target and held its value there; the next target re-interpolated that point, so a
request rewrote a waypoint the controller already steered toward (189 times, up to 46 mm, in a
fast teleoperation run). The grid now ends at or before the newest target; only the minimum of
two points can reach past it while the buffer fills. Timestamps are rounded instead of truncated,
which had put 42 waypoints of the same run 1 ms off the grid; the jogger's chunk anchor
(`_anchor`) rounds each of its terms for the same reason. On a virtual UR10e no rewrites and no
off-grid waypoints remained; the tracking error was unchanged. Tests:
`test_the_horizon_stops_at_the_newest_sample`,
`test_grid_points_keep_their_value_when_a_sample_arrives`,
`test_chunks_start_on_one_grid_in_exact_steps`.
