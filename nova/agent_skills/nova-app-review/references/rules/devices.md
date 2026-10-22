# Devices (DEV)

## DEV-001 · Verify gripper state by sensor after every actuation
`severity: error` · `scope: robotics` · `detect: static+review`

**Rule.** After open/close, wait (with timeout) for the sensor feedback of the expected state.
"Command sent" is not "part gripped". Lift the part only after grip confirmation.

---

## DEV-002 · Validate vision results before using them as targets
`severity: error` · `scope: robotics` · `detect: review`

**Rule.** A camera result becomes a motion target only after these checks: result present, match
score above threshold, pose within the expected workspace/ROI bounds, orientation within limits,
transformed into the correct frame (camera→robot calibration applied once, centrally).

**Detect.** Camera/vision return value used directly in a motion action; only a `None` check.

---

## DEV-003 · Force-controlled motion has limits, abort and timeout
`severity: error` · `scope: robotics` · `detect: static+review`

**Rule.** Every force-guided/contact motion defines a force threshold, a maximum travel distance, a
timeout, and the action taken on abort (stop, retract). Direction mappings are explicit constants.

NOVA: an IO-driven stop condition can be passed as `execute(..., pause_on_io=...)`. Check that it
is not dropped in simulation (SAF-011).

---

## DEV-004 · Network devices use async clients with timeouts
`severity: error` · `scope: robotics` · `detect: static`

**Rule.** HTTP/TCP device clients (grippers, cameras, IO-Link masters) are async, created once,
configured with explicit connect/read timeouts, and retried on connect.

**Detect.** `requests.` / `urllib.request` / blocking `socket` in device code; `httpx.AsyncClient()`
created per call or without `timeout=`.

---

## DEV-005 · Track tool contents and check before pick/place
`severity: warning` · `scope: robotics` · `detect: review`

**Rule.** Maintain a register of what each gripper holds. Before a pick, assert that it is empty;
before a place, assert that it holds the expected part. Update the register only after sensor
confirmation (DEV-001).

---

## DEV-006 · Validate device parameter invariants before writing
`severity: warning` · `scope: robotics` · `detect: review`

**Rule.** Check documented device constraints in code before writing parameters (e.g. gripper
positions `base < shift < work`, force/speed ranges). Raise with a clear message rather than letting
the device fault with a numeric error code.

---

## DEV-007 · Re-establish device readiness after mode change or fault
`severity: warning` · `scope: robotics` · `detect: review`

**Rule.** After device mode changes, power cycles or faults, re-check and restore readiness (motor
on, referenced/homed, error cleared) before the next command. Grippers may lose their reference
after a fault.

---

## DEV-008 · Open gripper before approach
`severity: warning` · `scope: robotics` · `detect: review`

**Rule.** Ensure the gripper is open (and confirmed) before the final approach to a part. Do this in
parallel with the free-space move to save cycle time (PERF-008).
