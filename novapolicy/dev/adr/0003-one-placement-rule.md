# 0003 One placement rule for both modes

Status: accepted, 2026-11-20

**Context.** Chunks were placed four ways: sequential at send + (k+1)·dt and never trimmed,
continuous one-step chunks a lead after the observation, continuous multi-step chunks at "now"
backdated by the inference time in whole steps, and the policy's own timelines. A user could not
tell from the mode what would happen.

**Decision.** Step `k` is due `reference + max_expected_latency_ms + k·dt`. The reference is the
observation in continuous execution and the send in sequential execution, where the robot was at
rest when it was observed. Multi-step chunks are dated by the session-clock estimate at
observation time, one-step chunks by the state's timestamp through a playback clock (T4).
Timelines of the policy's own (explicit timestamps, timestep queues, RTC) are kept.

**Alternatives measured.** Backdating by inference time: 0.74–1.14° rms at 200 ms inference
(now 0.45°, one run). Dating chunks by the state timestamp: shifted chunks overlapped and the
follower spiked at every seam, 0.014–0.016° rms on the NATS mock policy before any GC pause (now
0.006–0.009°).

**Consequences.** The modes differ only in when the next observation is taken. Continuous chunks
run about one lead later (lag ~108 ms instead of 24–50 ms); sequential chunks start one lead
after the send instead of one step, and a slow sequential policy can no longer lose steps.
Chunks indexed by call time rather than by their observation track a little worse at fast
inference (0.045 against 0.016° rms).
