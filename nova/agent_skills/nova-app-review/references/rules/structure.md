# Structure (STR)

## STR-001 · Job identifiers are unique and checked at registration
`severity: error` · `scope: robotics` · `detect: static`

**Rule.** Job names are unique. The registry raises on duplicates at import time. Duplicate names
lead to deduplicated, silently skipped jobs, or to mixed-up outcomes.

**Detect.** Collect all `id=` / `name=` arguments of job definitions; report duplicates.

NOVA: the `@nova.program` registry **silently replaces** a program registered with the same id
(the id defaults to the function name). Duplicate ids across modules mean one program disappears
from NOVAx.

---

## STR-002 · Every job follows the same skeleton
`severity: info` · `scope: robotics` · `detect: review`

**Rule.** Each job module exposes the same parts: `PRECONDITIONS`, `build_params(snapshot)`,
`check_conditions(snapshot, params)`, `run(ctx)` with home handling (SAF-001), and the
registration. Keep a template file for new jobs.

---

## STR-003 · No dead poses, commented-out motion or orphan TODOs
`severity: info` · `scope: general` · `detect: static`

**Rule.** No commented-out motion blocks (>5 lines), unused pose variables, or TODOs without an
owner/issue in job code. Leftover motion code is a commissioning hazard when someone uncomments it.
