# 0009 One send pipeline of opt-in modules

Status: accepted, 2026-11-22

**Context.** Shaping what is sent lived in three places: the executor's buffered mode
(`buffer_window_ms`), the jogger's live buffer (on by default for `set_target`) and the jogger's
ease-in, which blended by a guess of when each step would run. ADR 0001 keeps shaping out of the
executor and the joggers, but some callers need it on purpose.

**Decision.** Everything that reaches NOVA passes one ordered list per motion group:

    common chunk -> [chunk modules] -> Timestamps() -> [waypoint modules] -> NOVA sender

- The **common chunk** is what a policy, a jogger target or a replay produced, already mapped to
  NOVA units, for one motion group (`PendingChunk`).
- **Chunk modules** (`on_chunk`) run when a chunk is queued, on every chunk, and may change steps
  and timing. `LiveBuffer` is one: it sets its own time grid, so it must run before placement.
- **`Timestamps()`** is the placement step (contracts T1–T3), an item of the list.
- **Waypoint modules** (`on_waypoints`) run at every send, after the timestamps, and return values
  only, so no timing contract changes. `EaseIn` is one: it needs each waypoint's final time.
- The **NOVA sender** is always last.

Modules are opt-in (`pipeline=[...]` on `PolicyExecutor`, `jog_joints`, `jog_tcp`), each motion
group runs its own copy, and a misordered list is refused at construction. The base is two
protocols and a runner in the session; the built-ins (trim, joint clamp, bridge, endpoint ramp,
hold padding, queue smoothing) move into modules later, one at a time, each with a robot A/B.

**Measured (virtual UR10e, in the cluster, 2026-11-21/22).** Jogging a sine at about 90 Hz:
500 ms buffer 0.015° rms against the target; no buffer 0.45–0.84° with peaks up to twice the
commanded speed; the lead path 2.9°, its waypoints 11 ms apart colliding. `EaseIn(1 s)` cut the
peak speed in the first second from 85 to 54°/s. One-step RL at 30 Hz: 80 ms buffer module path
p90 0.6–0.7 mm, peaks 13–16 mm (the old built-in buffer 1.1–1.8 mm, 44 mm); 180 ms lead
0.3–0.4 mm, 9–10 mm. Re-run on the final code (2026-11-22): jogging and dual-arm numbers
unchanged.

**Consequences.** The executor, the joggers and the session shape nothing unless a module is
passed, and new shaping has one place to go. Jogging scripts that relied on the default buffer
pass `pipeline=[LiveBuffer(window_ms=500)]` (migration note). The buffer stays the right tool
for dense live targets and the wrong one for policy steps, which is written next to it.
