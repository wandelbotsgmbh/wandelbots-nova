# 0004 Late steps are dropped, never retimed or smoothed

Status: accepted, 2026-11-20

**Context.** A waypoint due on arrival is ignored by NOVA without an error; one stamped in the
past makes the follower jump to catch up. Client-side remedies were tried.

**Decision.** A step due within 20 ms of now is not sent; every other step keeps the time the
policy meant. Not-yet-due waypoints are re-sent with each request (WBR keeps only the newest
chunk per control cycle). The one bounded retiming is the playback clock for one-step chunks.

**Alternatives measured.** Spline resampling of the waypoints to 8 ms: twice the time at the
acceleration limit (j1 at 853°/s² for 178–229 samples against 88–105), path p90 0.9–2.1 mm
against 0.3–0.4 mm. `buffer_window_ms`: more lag and more jitter than the lead (2026-11-18,
before `max_expected_latency_ms`; to be re-measured before it is removed).

**Consequences.** What reaches NOVA is what the policy said, at the time it said it. The remaining
path deviation (about 9 mm peaks) is the follower's, reported to NOVA.
