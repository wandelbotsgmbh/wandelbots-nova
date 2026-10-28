# Performance (PERF)

## PERF-001 · No fixed sleeps as synchronisation
`severity: warning` · `scope: robotics` · `detect: static`

**Rule.** Replace "wait N seconds just in case" with a wait on the actual condition (IO bit, device
status, camera ready) with a timeout. If no signal exists, measure the real duration and document
it.

**Observed.** About 12 s of fixed sleeps per cycle (4 + 4 + 1 + 2 s + device sleeps), almost all
removable.

**Detect.** `asyncio.sleep(<literal ≥ 0.1>)` or `wait(<seconds>)` actions in job/device code.

---

## PERF-002 · One IO cache, push-fed; no N parallel pollers
`severity: warning` · `scope: nova` · `detect: review`

**Rule.** Use one subscription (push/stream) to maintain an IO snapshot. All consumers read from
that cache. Several independent loops polling the same REST endpoint multiply the load and add
jitter.

**Observed.** Five loops (4 ms, 20 ms, 20 ms, 20 ms, 500 ms) generated **>400 HTTP requests/s**.
Writing telemetry triggered change notifications, which triggered more reads (feedback loop).

NOVA: bus IO changes are pushed over NATS (`nova.utils.io.wait_for_bus_io`); controller state via
`controller.stream_state(rate_msecs)`; motion group state via `mg.stream_state()` (shared
websocket per motion group).

---

## PERF-003 · Poll interval ≥ transport round-trip
`severity: warning` · `scope: robotics` · `detect: review`

**Rule.** A poll interval shorter than the round-trip of the transport it polls only adds load. A
4 ms loop over HTTP is effectively bounded by HTTP latency. Prefer push. If polling, size the
interval to the measured RTT.

---

## PERF-004 · Signal completion with events, not poll + settle
`severity: warning` · `scope: python-async` · `detect: static`

**Rule.** Callers waiting for a job result await a per-job `Future`/`Event` that the worker
resolves. Do not poll a flag and then wait a "settle" timeout to hide a race.

**Observed.** A 1 s settle timeout after each job, plus 50 ms polling, cost up to ~1 s per job.

**Detect.** Constants named `*SETTLE*`, `*GRACE*`; `while pending: await asyncio.sleep(...)`.

---

## PERF-005 · Preplan and cache static trajectories with stable keys
`severity: warning` · `scope: nova` · `detect: review`

**Rule.** Plan motions that are fully determined by configuration (no runtime inputs) at startup
and cache them. Use **explicit, stable cache IDs**. File/line-based identity breaks on every edit.
Version the cache format and invalidate it when poses, TCP or profiles change.

NOVA: `trajectory = await mg.plan(actions, tcp=TCP)` once, then `await mg.execute(trajectory,
tcp=TCP, actions=actions)` per cycle. Pass `start_joint_position=` when planning offline.

---

## PERF-006 · No synchronous file IO per log line in hot loops
`severity: warning` · `scope: python-async` · `detect: static`

**Rule.** Keep log/report file handles open and buffer writes, or use a `QueueHandler`.
Open-write-close per line inside a control loop blocks the loop on disk/network latency.

---

## PERF-007 · Warm up planner and APIs at startup
`severity: info` · `scope: nova` · `detect: review`

**Rule.** Issue one throwaway plan and the required description calls during startup.

**Observed.** First trajectory load 4.03 s, later loads 0.14 s. First motion-group description
4.01 s.

NOVA: `await mg.get_description()`, `await mg.tcp_names()` and one `await mg.plan(...)` at
startup.

---

## PERF-008 · Overlap side tasks with motion
`severity: info` · `scope: nova` · `detect: review`

**Rule.** Trigger cameras, pre-open grippers and write IO while the robot is still moving, at
defined points on the trajectory (trajectory cursor / location-based triggers), instead of
stop → act → move.

NOVA: `io_write(key, value, at=before_target(millimeters=50))` / `after_start(seconds=...)` /
`at_path_fraction(0.8)` from `nova.actions`.

---

## PERF-009 · Deadband continuous telemetry writes
`severity: info` · `scope: robotics` · `detect: static`

**Rule.** Throttle or deadband telemetry (joint angles, forces) published to the PLC/HMI.

**Observed.** Writing all 6 axes on any change caused ~50 writes/s during motion.

---

## PERF-010 · Fast device polling during handshakes
`severity: info` · `scope: robotics` · `detect: static`

**Rule.** If a device can only be polled, poll at 10–20 ms during an active handshake. A 100 ms
interval adds up to 100 ms per checked bit, several times per gripper operation.

---

## PERF-011 · Minimise TCP switches and re-plans
`severity: info` · `scope: nova` · `detect: review`

**Rule.** Standardise on as few TCPs as possible. Each switch forces separate plans and prevents
merging or blending across segments.

---

## PERF-012 · No unbounded work in hot paths
`severity: info` · `scope: python-async` · `detect: static`

**Rule.** Hot loops do not run exponential or unbounded searches (e.g. trying all 2^N IO
combinations to simulate a wait condition).
