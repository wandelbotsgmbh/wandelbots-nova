# Configuration (CFG)

## CFG-001 · One source of truth for cell configuration
`severity: warning` · `scope: robotics` · `detect: static`

**Rule.** Controller name, motion group, TCPs, speed profiles, device endpoints, station definitions
and part vocabularies live in one configuration module or file. Jobs import from it.

NOVA: connection settings come from `NOVA_API`, `NOVA_ACCESS_TOKEN` and `CELL_NAME`
(`nova.config`); do not re-read them in job code.

---

## CFG-002 · Typed, validated, logged configuration
`severity: info` · `scope: general` · `detect: static`

**Rule.** Environment variables are parsed once into typed values with defaults and range
validation. The effective configuration is logged at startup, with secrets redacted (never log
`NOVA_ACCESS_TOKEN`). Unknown/invalid values fail fast.

**Detect.** `os.environ[` / `os.getenv(` / `decouple.config(` outside the config module.

---

## CFG-003 · Independent simulation flags per subsystem
`severity: info` · `scope: robotics` · `detect: static`

**Rule.** Separate flags for virtual robot, virtual PLC and each virtual device enable
hardware-in-the-loop setups (real robot with simulated PLC, and the reverse).

---

## CFG-004 · Safety-relevant config must not freeze silently at import
`severity: info` · `scope: python-async` · `detect: review`

**Rule.** Values like the reduced-speed divider are computed when the module is imported. Make that
explicit (log the value). If they must be switchable at runtime, compute profiles lazily.
