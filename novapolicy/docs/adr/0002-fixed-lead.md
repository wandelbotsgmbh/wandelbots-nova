# 0002 A fixed lead, and a loud stop when it is exceeded

Status: accepted, 2026-11-18. The loud stop (the late-acknowledgement guard) was removed on
2026-11-23 with the other diagnostics; the fixed lead stands.

**Context.** A waypoint must reach the controller about 67 ms before it is due, plus the
network round trip. Over a VPN the round trip ranged from 40 to 220 ms.

**Decision.** Waypoints are due `WaypointConfig.max_expected_latency_ms` (default 100 ms) after
their reference, as MoveIt Servo's `max_expected_latency`. The measured round trip is only a
guard: five one-step waypoints in a row that arrive with less than 67 ms left stop the run with
`MotionError` ("over budget").

**Alternative measured.** A lead adapted to the measured round trip: when the latency rose, the
motion degraded without a word, which proven systems (MoveIt Servo, UMI, Diffusion Policy,
robot streaming interfaces) avoid the same way.

**Consequences.** The arm follows the policy one lead late; closed-loop policies are trained
with that delay. 100 ms fits a policy running on the NOVA instance (93 ms left on arrival); over
a VPN 250 ms was needed.
