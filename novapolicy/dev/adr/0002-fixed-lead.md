# 0002 A fixed lead

Status: accepted, 2026-11-18. The loud stop it also decided (the late-acknowledgement guard) was
removed on 2026-11-23 with the other diagnostics; the fixed lead stands.

**Context.** A waypoint must reach the controller about 67 ms before it is due, plus the
network round trip (`SINGLE_STEP_MARGIN_MS`: 33 ms ahead was rough on a virtual UR10e, 67 ms
smooth). Over a VPN the round trip ranged from 40 to 220 ms.

**Decision.** Waypoints are due `WaypointConfig.max_expected_latency_ms` (default 100 ms) after
their reference, as MoveIt Servo's `max_expected_latency`. The measured round trip is only a
guard: five one-step waypoints in a row that arrive with less than 67 ms left stop the run with
`MotionError` ("over budget"). (Removed 2026-11-23: nothing reads the round trip any more.)

**Alternative measured.** A lead adapted to the measured round trip: when the latency rose, the
motion degraded without a word, which proven systems (MoveIt Servo, UMI, Diffusion Policy,
robot streaming interfaces) avoid the same way.

**Consequences.** The arm follows the policy one lead late; closed-loop policies are trained
with that delay. 100 ms fits a policy running on the NOVA instance (93 ms left on arrival). Over
a VPN 250 ms was needed at first; with the playback clock (ADR 0017), 150 ms is enough (RL reach,
2026-11-24: path max 19 mm and lag 95 mm at 100 ms, 9 mm and 41 mm at 150 ms, no better at 200 ms).
