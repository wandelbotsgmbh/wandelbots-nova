# 0004 Late steps are dropped, never retimed or smoothed

Status: accepted, 2026-11-20

**Context.** A waypoint due on arrival is ignored by NOVA without an error; one stamped in the
past makes the follower jump to catch up. Client-side remedies were tried.

**Decision.** A step due within 20 ms of now is not sent; every other step keeps the time the
policy meant. Not-yet-due waypoints are re-sent with each request, as MoveIt Servo sends a rolling
window: the follower keeps them anyway, but WBR keeps only the newest chunk per control cycle, and
a chunk lost in a burst comes back with the next one. Only the placement step retimes, and only
one-step chunks: the playback clock (ADR 0017) and, opt-in since 2026-11-24, the cycle grid
(ADR 0021).

**Alternatives measured.** Spline resampling of the waypoints to 8 ms: twice the time at the
acceleration limit (j1 at 853°/s² for 178–229 samples against 88–105), path p90 0.9–2.1 mm
against 0.3–0.4 mm. A live-target buffer (`LiveBuffer`): worse for one-step policy steps, better
for dense jogging targets (numbers in ADR 0009), so it stays an opt-in module in front of
placement; it sets its own time grid.

**Consequences.** What reaches NOVA is what the policy said, at the time it said it. The remaining
path deviation (about 9 mm peaks) is the follower's, reported to NOVA.
