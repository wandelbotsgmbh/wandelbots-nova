# 0018 Stale inputs end the run, with bounds that catch a dead input, not a slow one

Status: accepted. Recorded 2026-11-23 from the code (`executor.py`); the decision is older. The
contract is M10; what happens is in [executor.md](../executor.md#stale-inputs-and-how-a-run-ends).

**Context.** A camera can freeze and a policy service can hang. Either one, unnoticed, keeps the
robot moving on old information: a frozen frame drives chunk after chunk, a hung inference leaves
the loop waiting forever.

**Decision.**

- A camera frame older than its bound ends the run. The default, `camera_max_age_s=1.0`, is fresh
  in frame terms rather than tens of seconds: loose enough for a slow stream, tight enough that a
  frozen feed surfaces before it has driven many chunks of motion. A channel declares its own bound
  where it runs slower.
- One `get_actions` call has a deadline, `inference_timeout_s=30.0`. It is an outer liveness guard
  against a hung service, applied whatever the transport's own timeout (LeRobot's gRPC calls use
  15 s), and generous by design: it does not police latency and is no real-time guarantee.

**Consequences.** A dead input ends the run with `RuntimeError("Stale policy input ...")` instead of
as silent motion or a hang. A slow but live policy is not stopped by the deadline.
