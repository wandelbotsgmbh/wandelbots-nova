# 0001 The executor maps and transports; NOVA owns the motion

Status: accepted, 2026-11-20

**Context.** The executor grew motion shaping to make policies look smooth: a simulated actuator,
spline and buffer resampling, smoothing filters. Each one changed what the policy commanded
without the policy knowing, and duplicated work NOVA's action chunk streaming does.

**Decision.** The executor maps (robot state, cameras, IO ↔ the policy's observations and
actions) and transports (timestamped waypoints to NOVA). Timing of the transport is in scope;
trajectory generation, filters and actuator models are not. If the arm does not follow the
waypoints faithfully, that is fixed or reported in NOVA. Written into `AGENTS.md`.

**Consequences.** A policy runs as trained, against NOVA's tracking. Tracking limits show up as
NOVA issues (the follower's correction law, per-joint limits) instead of being masked.
