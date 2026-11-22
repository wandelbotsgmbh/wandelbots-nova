# 0004 Late steps are dropped, never retimed or smoothed

Status: accepted, 2026-11-20

**Context.** A waypoint due on arrival is ignored by NOVA without an error; one stamped in the
past makes the follower jump to catch up. Client-side remedies were tried.

**Decision.** A step due within 20 ms of now is not sent; every other step keeps the time the
policy meant. Not-yet-due waypoints are re-sent with each request (WBR keeps only the newest
chunk per control cycle). The one bounded retiming is the playback clock for one-step chunks.

**Alternatives measured.** Spline resampling of the waypoints to 8 ms: twice the time at the
acceleration limit (j1 at 853°/s² for 178–229 samples against 88–105), path p90 0.9–2.1 mm
against 0.3–0.4 mm. A live-target buffer (`LiveBuffer`, opt-in): for a one-step RL policy at 30 Hz an
80 ms window gave path p90 1.1–1.8 mm and 44 mm peaks against 0.3–0.4 mm and 10 mm on a 180 ms
lead (2026-11-21). For dense jogging targets at about 90 Hz it is the better tool: 0.015° rms
against 0.45–0.84° without it, and 2.9° on the lead path, whose waypoints 11 ms apart collide.
It stays a separate module in front of placement, not a stage: it sets its own time grid.

**Consequences.** What reaches NOVA is what the policy said, at the time it said it. The remaining
path deviation (about 9 mm peaks) is the follower's, reported to NOVA.
