# Observability (OBS)

## OBS-001 · No `print()` in runtime code
`severity: warning` · `scope: general` · `detect: static`

**Rule.** Use the logging facade. `print` bypasses log files, levels and HMI listeners.

NOVA: `from nova.logging import logger` or the standard `logging` module.

---

## OBS-002 · Structured, tagged log lines
`severity: info` · `scope: general` · `detect: static`

**Rule.** Prefix log lines with domain tags (`[STATE]`, `[JOB]`, `[NOVA]`, `[IO]`, `[FIX_TIP]`,
`[TIMING]`) or emit key=value/JSON. Silence noisy SDK loggers to WARNING; raise them temporarily
during startup diagnostics.

NOVA: the SDK log level follows the `LOG_LEVEL` environment variable.

---

## OBS-003 · Log every state transition with timing
`severity: info` · `scope: robotics` · `detect: review`

**Rule.** Log source, target, trigger and time-in-previous-state for every supervisor transition.

---

## OBS-004 · Measure cycle time by segment, plan vs execute
`severity: info` · `scope: robotics` · `detect: review`

**Rule.** Record per-job and per-segment durations, and split plan time from execute time and from
IO wait time. Without this split you cannot tell whether to optimise motion, planning or
handshakes.

NOVA: `ctx.cycle()` (`nova.events.Cycle`) publishes cycle start/finish events; call `plan` and
`execute` separately to time them.

---

## OBS-005 · Measure fault-detection-to-stop latency
`severity: info` · `scope: robotics` · `detect: review`

**Rule.** Timestamp first fault detection and the completed motion stop. Report the latency on
every fault.
