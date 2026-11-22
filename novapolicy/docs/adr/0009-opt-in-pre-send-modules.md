# 0009 Ease-in and the live buffer are opt-in modules, not executor or jogger code

Status: accepted, 2026-11-22

**Context.** The live-target buffer lived twice (executor `buffer_window_ms`, jogger, on by default
for `set_target`), and ease-in only in the jogger, where it blended by a guess of when each step
would run. Both shape what is sent, which ADR 0001 keeps out of the executor and the joggers.

**Decision.** Two kinds of opt-in module, none by default:

- **Pre-send stages** (`stages=[EaseIn(...)]`) run in the session after placement, right before
  the request: they get `(timestamp, values)` and return values, so they cannot touch timing
  (contract T13). Every path gets them alike, and a stage sees the final timestamps, so ease-in
  blends each waypoint by its own time and overlapping chunks agree.
- **`LiveBuffer`** sets its own time grid, so it cannot be a values-only stage: it sits in front
  of placement and turns a stream of live targets into chunks with their own timeline (T2), for
  the joggers' `set_target` and, opt-in, the executor.

**Measured (virtual UR10e, in the cluster, 2026-11-21).** Jogging a sine at about 90 Hz:
500 ms buffer 0.015° rms against the target; no buffer 0.45–0.84° with peaks up to twice the
commanded speed; the lead path 2.9°, its waypoints 11 ms apart colliding. One-step RL at 30 Hz:
80 ms buffer path p90 1.1–1.8 mm, peaks 44 mm; 180 ms lead 0.3–0.4 mm, 10 mm.

**Consequences.** The executor and the joggers shape nothing unless asked. Jogging scripts that
relied on the default buffer must now pass `LiveBuffer(window_ms=500)` (migration note); the
shipped live-target examples do. The buffer stays the right tool for dense live targets and the
wrong one for policy steps, which is written next to it.
