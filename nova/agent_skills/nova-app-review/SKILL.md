---
name: nova-app-review
description: "Review a robot cell application built on the Wandelbots NOVA Python SDK (wandelbots-nova) for safety, motion, PLC/IO, async runtime, performance, lifecycle, device handling and NOVA SDK usage, and report findings with severity, evidence and concrete fixes. Use when the user asks to review, audit, check, lint or harden a NOVA app, a @nova.program, a NOVAx app or any robot application; asks about best practices, common pitfalls, cycle time, safety or commissioning readiness; or asks 'is my robot code OK?'."
---

# NOVA app review

Review a customer robot application against the rule catalogue in `references/rules/` and report
findings **in chat**. This is a read-only review: do **not** edit project files unless the user
asks for fixes afterwards.

Software rules are **defence in depth**. They never replace certified safety functions (safety
PLC, light curtains, SafeOperation, collaborative modes). Say so once in the report.

All paths below are relative to this skill's directory (the folder containing this file).

## Workflow

Track these steps with a todo list.

### 1. Establish context

- Find the app root (the directory with `pyproject.toml` / `requirements.txt` / `.nova`).
- Read the project's `AGENTS.md`, `CLAUDE.md` or `README.md` if present.
- Note the installed SDK version: `uv run python -c "import importlib.metadata as m; print(m.version('wandelbots-nova'))"`
  (or the project's own interpreter). If an `.installed-from.json` file next to this `SKILL.md`
  names a different `wandelbots-nova` version, tell the user to refresh the skill with
  `nova-agent-skills install` and continue with a note that SDK mappings may be stale.

### 2. Discover the architecture

Static search alone misses most safety findings. Locate and write down (file + line):

- entry point and event loop setup (`@nova.program`, `Novax`, `asyncio.run`, FastAPI lifespan)
- robot connection / session lifecycle (`async with Nova()`, `cell.controller(...)`, motion group)
- motion entry points (`plan`, `execute`, `plan_and_execute`, `TrajectoryCursor`)
- IO layer (controller IO `read`/`write`/`wait_for_bool_io`, bus IO `nova.utils.io`, `io_write`)
- state machine or supervisor (modes, fault handling, reset)
- fault path (what happens between "error detected" and "robot stopped")
- shutdown path and signal handling
- job/program definitions and how they are selected
- device plugins (gripper, vision, force sensor, ...)
- configuration sources (env, config modules, datasets, PLC tag exports)

### 3. Run the scanner

```bash
uv run python <skill-dir>/scripts/scan.py <app-root>     # or: python3 <skill-dir>/scripts/scan.py <app-root>
```

The scanner needs only the Python standard library. It prints JSON with:

- `candidates`: `{rule, severity, confidence, file, line, excerpt, message, path_kind}`
- `facts`: project-wide signals (e.g. `queries_tcp_names`, `uses_payload`, `nova_entry_points`)
  that rules use to decide whether a feature is missing

Candidates are **hints, not findings**. Confirm each one by reading the code. Drop false positives
silently; never report a candidate you have not confirmed.

### 4. Review every rule

Load the rule files for the areas you found in step 2. Load `references/nova-sdk-mapping.md` for
every NOVA-specific rule (`scope: nova`) and whenever the fix involves an SDK call. It lists the
SDK symbol to use, known SDK gaps and the recommended workaround for this SDK version.

| File | Rules |
|---|---|
| `references/rules/safety.md` | SAF-001 … SAF-014 |
| `references/rules/motion.md` | MOT-001 … MOT-013 |
| `references/rules/io.md` | IO-001 … IO-007 |
| `references/rules/async.md` | ASY-001 … ASY-009 |
| `references/rules/performance.md` | PERF-001 … PERF-012 |
| `references/rules/lifecycle.md` | LIFE-001 … LIFE-008 |
| `references/rules/errors.md` | ERR-001 … ERR-006 |
| `references/rules/devices.md` | DEV-001 … DEV-008 |
| `references/rules/configuration.md` | CFG-001 … CFG-004 |
| `references/rules/observability.md` | OBS-001 … OBS-005 |
| `references/rules/testing.md` | TST-001 … TST-003 |
| `references/rules/structure.md` | STR-001 … STR-003 |
| `references/nova-sdk-mapping.md` | rule → SDK symbol, gaps, workarounds |
| `references/reference-numbers.md` | measured timeouts and loop rates for sizing fixes |

Each rule has a `detect` hint: `static` (pattern is enough), `review` (read the control flow) or
`static+review` (search, then confirm by reading).

Skip rules that cannot apply (e.g. DEV-002 in a cell without vision) and list them as
"not applicable" in one line. Do not demand a feature the cell does not need.

### 5. Report in chat

Use the format below. Do not write a report file unless the user asks for one.

## Severity

| Severity | Meaning | Gate |
|---|---|---|
| `error` | Can injure people, damage hardware/parts, or leave the cell in an undefined state. | Block release |
| `warning` | Measurable cycle-time loss, reliability risk, or recovery problems. | Fix before commissioning |
| `info` | Maintainability, observability, ergonomics. | Backlog |

Raise severity by one level if the code path runs in **automatic mode on real hardware**; lower it
by one if it is provably **simulation/test-only**. State the adjustment in the finding.

## False-positive guidance

- Tests, simulators and one-off scripts (`tests/`, `scripts/`, `tools/`) may break performance and
  async rules. Still report **safety** rules there if the script can move a real robot.
- Generated files (e.g. IO symbol modules) are exempt from naming/structure rules.
- A rule is satisfied if the requirement is met **centrally** (e.g. a shared `run_job()` wrapper
  that always returns home). Do not demand per-job duplication.
- Hardware safety is outside software scope.
- Code copied from SDK examples is not exempt: examples favour brevity over production rules.

## Report format

```markdown
## NOVA app review — <app name>

SDK: wandelbots-nova <version> · Scope: <dirs reviewed> · Rules checked: <n> · Not applicable: <ids>

> Software checks are defence in depth and do not replace certified safety functions.

| Severity | Count |
|---|---|
| error | n |
| warning | n |
| info | n |

### Top 5 fixes
1. RULE-ID — one line: what to change and why it matters most

### Safety
#### [ERROR] SAF-003 — Fault path stops motion first, with the shortest timeout
- **Location:** [app/supervisor.py](app/supervisor.py#L120-L134)
- **Evidence:** `await log_fault(...)` runs before the motion task is cancelled
- **Impact:** the robot keeps moving for the duration of the log call when a fault arrives
- **Fix:** cancel the execute task first, bounded by `asyncio.timeout(STOP_TIMEOUT_S)`; then log
  ```python
  motion_task.cancel()
  async with asyncio.timeout(STOP_TIMEOUT_S):
      await asyncio.gather(motion_task, return_exceptions=True)
  ```
- **Reference:** `references/rules/safety.md` (SAF-003); SDK: `nova-sdk-mapping.md` (SAF-003)
- **Confidence:** high

### Motion
...
```

Rules for the report:

- Group findings by category in catalogue order; within a category order by severity.
- Every finding has file + line evidence. No evidence, no finding.
- **Fix** is the minimal concrete change, with a short code snippet where it helps. Use SDK symbols
  from `nova-sdk-mapping.md`; never invent SDK APIs. If the SDK lacks a feature, say so and give
  the workaround from the mapping.
- **Reference** names the rule file and rule ID, plus the SDK symbol or the SDK source/doc link
  from the mapping.
- If the same violation repeats, report it once and list all locations.
- End with a short "Next steps" list and offer to apply the fixes.
