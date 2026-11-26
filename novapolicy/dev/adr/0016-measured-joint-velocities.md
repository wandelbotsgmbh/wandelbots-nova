# 0016 Joint velocities are read from the raw state stream

Status: accepted. Recorded 2026-11-23 from the code (`mapping/state_stream.py`,
`mapping/joint_velocities.py`);
the decision is older. The contract is M9.

**Context.** A policy trained on measured joint velocities needs them at inference. The typed SDK
path cannot deliver them: `MotionGroupState` has no `joint_velocity` field and pydantic drops
unknown fields, so the value the controller puts on the wire is discarded; `RobotState` carries
only pose, TCP and joints.

**Decision.** Read the state stream as raw JSON, one read-only stream per motion group beside the
session's own, and use `result.joint_velocity` where the controller sends it. Where it does not
(UR controllers today), difference consecutive positions, reported once per motion group. A
sample older than 0.2 s is not used. The stream is requested every 20 ms; at or below the
controller cycle the server streams every cycle (about 8 ms on a live instance). Do not move this
onto the typed stream.

**Alternative measured.** Finite differences at the request cadence, against the controller's own
values on a 7-dof arm at 20 ms: twice the true magnitude, three times as noisy from sample to
sample, and off by up to 4.8 rad/s on single samples. A residual policy fed that produces large,
erratic residuals.

**Consequences.** The fallback differences streamed packets on their own timestamps, across at
least 20 ms (packets a few milliseconds apart turn position quantization into noise), and a
repeated sample keeps the last velocity for up to 0.1 s instead of reading as standstill. When
NOVA's typed state carries joint velocities, the observation uses them without a change for its
users.
