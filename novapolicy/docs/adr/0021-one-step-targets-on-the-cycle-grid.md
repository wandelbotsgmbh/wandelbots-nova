# 0021 One-step joint targets can be put on NOVA's cycle grid (opt-in)

Status: accepted, 2026-11-24. `Timestamps(grid_ms=...)` in `pipeline/base.py`, the resampling in
`pipeline/placement.py` (`resample_on_grid`). It extends the placement step (contracts T1, T4).

**Context.** An RL policy's targets are smooth and evenly paced, yet the virtual UR10e followed
them in pulses: it sped up toward each waypoint and slowed before the next. Spacing the waypoints
exactly one policy step apart did not change that (executed ripple 0.183 against 0.185 rad/s). The
cause is NOVA's waypoint follower: every 8 ms cycle it commands `(next waypoint − q) / (time
left)`, from a state about one cycle old. A waypoint due just after a cycle leaves a fraction of a
cycle of time left, the gain spikes and the arm overshoots its pace. The state timestamps are exact
multiples of 8 ms on the session clock, so the cycle grid is known on the client. In a simulation
of the follower, waypoints due on the grid every 4 cycles held a steady speed; 1 ms off the grid
changed nothing, 3 ms off still kept the speed within ±12%.

**Decision.**

- `Timestamps(grid_ms=32)` resamples the path of one-step joint targets (the lead window, T5) onto
  the multiples of `grid_ms` of the session clock, from now plus `MIN_LEAD_MS` to the newest
  target, linearly between targets. The path starts at the robot's joints and continues across
  requests, so a waypoint sent again keeps its value.
- It is a setting of the placement step, the one stage allowed to time waypoints: filters never
  retime and waypoint modules change values only. Multi-step chunks and TCP targets keep their
  times; `Timestamps()` without it is unchanged on the wire (`test_wire_capture.py`).
- Off by default. It is tuned to the follower of the virtual robot; a physical controller's servo
  differs and needs its own A/B before the default changes. It becomes unnecessary once NOVA's
  follower handles off-grid waypoints.
- A pre-shift of the values by one cycle, for the follower's delay, was tried and left out: in two
  runs it was not distinguishable from the grid alone (below).
- Measured at a lead of 150 ms only. The gridded window ends up to one grid step before the newest
  target, so the controller holds that much less; check a shorter lead before using it there.

**Measured** on a virtual UR10e over a remote network, the LEAPP two-point reach (A, B, A), lead
150 ms, five runs each (mean, then the range). Ripple is the shoulder joint's executed speed minus its 160 ms average, RMS
while moving; path and lag are the flange's distance from the commanded path and from the
commanded point.

| | runs | executed ripple (rad/s) | path max (mm) | path p99 (mm) | lag max (mm) |
| --- | --- | --- | --- | --- | --- |
| `Timestamps()` | 5 | 0.161 (0.134 to 0.187) | 11.1 (8.4 to 13.7) | 7.0 | 45 (41 to 48) |
| `Timestamps(grid_ms=32)` | 5 | 0.116 (0.089 to 0.140) | 5.8 (3.6 to 7.7) | 3.6 | 32 (19 to 40) |
| grid and an 8 ms pre-shift | 2 | 0.167, 0.129 | 6.0, 6.8 | 4.4 | 32, 34 |

The example's `--grid-ms 32` gave the same (one run: ripple 0.124, path max 4.1 mm, waypoints
exactly 32 ms apart). Every run reached its parts within 2.5 mm at 30 Hz.

**Consequences.** On the virtual robot the largest path deviation halves, with no overlap between
the runs; the executed ripple drops by about 28%, with the runs' ranges touching. The policy's own step-to-step speed variation still reaches the arm.
The waypoints are 32 ms apart instead of one policy step, so a request carries about as many.
