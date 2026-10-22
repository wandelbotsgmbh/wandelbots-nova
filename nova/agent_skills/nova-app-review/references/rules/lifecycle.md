# Lifecycle (LIFE)

## LIFE-001 · Ordered shutdown
`severity: error` · `scope: robotics` · `detect: review`

**Rule.** Shut down in reverse dependency order:
1. stop producers (dispatcher, triggers)
2. stop motion and cancel motion tasks
3. stop background loops (home check, telemetry)
4. quiesce IO consumers
5. write final safe outputs (SAF-009)
6. shut down device plugins (reverse init order)
7. close the robot session and HTTP clients
8. stop the event loop

**Why.** Final IO writes contend with still-running pollers and stall; plugins torn down before
motion stops can leave tools in an undefined state.

NOVA: in a NOVAx app, put this sequence in the FastAPI lifespan shutdown; in a `@nova.program`,
in a `finally` around the program body.

---

## LIFE-002 · Each shutdown step is bounded and isolated
`severity: warning` · `scope: python-async` · `detect: static+review`

**Rule.** Wrap each shutdown step in its own timeout and exception guard; log and continue on
failure. Give the whole shutdown an overall budget.

**Observed.** 3 s per step, 8 s for the robot runtime.

---

## LIFE-003 · Connect with timeout and bounded retry
`severity: warning` · `scope: nova` · `detect: static`

**Rule.** Opening the robot session, resolving cell/controller/motion group and fetching TCPs each
have a timeout. Retry a bounded number of times with backoff. "Controller not connected" during
startup is a transient condition.

NOVA: `async with Nova()` and `await cell.controller(name)` have no timeout parameter; wrap them
(see `nova-sdk-mapping.md`, LIFE-003). `ControllerNotFound` is not transient; do not retry it.

---

## LIFE-004 · Guaranteed close of sessions and clients
`severity: warning` · `scope: nova` · `detect: static`

**Rule.** Use `async with` (or try/finally) for the robot client, motion group and HTTP clients. If
the SDK leaks sessions, isolate the workaround in one place and document the SDK version.

**Detect.** `Nova(...)` not used as `async with`; `await nova.open()` without `await nova.close()`
in a `finally`.

---

## LIFE-005 · Symmetric, idempotent plugin init/shutdown
`severity: warning` · `scope: robotics` · `detect: review`

**Rule.** Every device plugin exposes `initialize()` and `shutdown()`. Both are idempotent, and
shutdown runs in reverse init order. Tests and production use the same order.

---

## LIFE-006 · Persist physical state atomically
`severity: warning` · `scope: robotics` · `detect: static+review`

**Rule.** State that reflects the physical world (part in gripper, tool loaded, counters) is
persisted with write-temp-then-rename and reloaded at startup. Otherwise a restart forgets that a
part is still held.

---

## LIFE-007 · Signals route into the normal shutdown path
`severity: info` · `scope: python-async` · `detect: static`

**Rule.** SIGINT/SIGTERM/Ctrl+C trigger the same ordered shutdown as a requested stop. No
`os._exit`, no `sys.exit` from inside tasks.

---

## LIFE-008 · Server-side trajectory cache has a lifecycle
`severity: info` · `scope: nova` · `detect: review`

**Rule.** If trajectories are stored on the robot server, the application defines when to clear
them (startup flag, version change) so that they do not accumulate across restarts.
