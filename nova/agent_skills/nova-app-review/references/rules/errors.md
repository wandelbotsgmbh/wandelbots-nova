# Error handling (ERR)

## ERR-001 · No swallowed exceptions
`severity: error` · `scope: general` · `detect: static`

**Rule.** No `except: pass`, `except Exception: pass`, or `except ...: print(...)` in runtime code.
Either handle meaningfully, or log with context and re-raise.

**Detect.** `except[^:]*:\s*(pass|print\()`; broad `except Exception` without `raise` or logging.

---

## ERR-002 · Invalid parameters raise, never return sentinels
`severity: error` · `scope: general` · `detect: review`

**Rule.** Parameter builders that cannot resolve a valid target (slot, part, pose) raise. They must
not return `0`, `None` or a default that later turns into a motion target.

**Observed.** A slot resolver printed a warning and returned `slot=0`. The job continued and failed
later with an unrelated error.

---

## ERR-003 · Worker errors reach the supervisor immediately
`severity: warning` · `scope: robotics` · `detect: review`

**Rule.** When the motion worker fails, it notifies the supervisor (callback/event), which enters
FAULT. Do not only set a flag that the supervisor discovers on its next poll.

---

## ERR-004 · Distinguish external, runtime and planning faults
`severity: warning` · `scope: robotics` · `detect: review`

**Rule.** Model at least three fault classes with different recovery:
- **external** (PLC/safety): auto-recoverable after reset once the cause clears
- **runtime** (connection, timeout): recoverable after a health check
- **planning/reachability**: manual recovery (SAF-005)

NOVA: planning → `PlanTrajectoryFailed`, `NoInverseKinematicsSolutionFound`,
`InconsistentCollisionScenes`; execution → `InitMovementFailed`, `ErrorDuringMovement`,
`LoadPlanFailed` (all in `nova.exceptions`); runtime → `TimeoutError`, `httpx` errors.

---

## ERR-005 · No private SDK APIs outside one adapter
`severity: warning` · `scope: nova` · `detect: static`

**Rule.** Access to private members (`._api_client`, `._internal`) is confined to one adapter module
that pins the SDK version and has a test. Scattered private access breaks on SDK upgrades.

**Detect.** `\._api_client`, `getattr\(.*, "_` outside the adapter.

NOVA: the public raw API client is `nova.api` (generated `wandelbots_api_client` endpoints); use
it instead of private SDK attributes.

---

## ERR-006 · Operator-facing errors carry a fix tip
`severity: info` · `scope: robotics` · `detect: review`

**Rule.** Errors that reach the HMI/PLC carry a short, actionable instruction ("Move robot with
teach pendant, then press Reset") separate from the technical message.
