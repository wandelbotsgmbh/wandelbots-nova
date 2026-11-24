# AGENTS.md

## Coding agent guidelines

Behavioral guidelines to reduce common mistakes when automated coding assistants edit this repository. Use together with the project-specific sections below.

**Tradeoff:** These guidelines bias toward caution over speed. For trivial tasks, use judgment.

### 1. Think before coding

**Do not assume. Do not hide confusion. Surface tradeoffs.**

Before implementing:

- State your assumptions explicitly. If uncertain, ask.
- If multiple interpretations exist, present them—do not pick silently.
- If a simpler approach exists, say so. Push back when warranted.
- If something is unclear, stop. Name what is confusing. Ask.

### 2. Simplicity first

**Minimum code that solves the problem. Nothing speculative.**

- No features beyond what was asked.
- No abstractions for single-use code.
- No flexibility or configurability that was not requested.
- No error handling for impossible scenarios.
- If you write 200 lines and it could be 50, rewrite it.

Ask: “Would a senior engineer say this is overcomplicated?” If yes, simplify.

### 3. Surgical changes

**Touch only what you must. Clean up only your own mess.**

When editing existing code:

- Do not “improve” adjacent code, comments, or formatting.
- Do not refactor things that are not broken.
- Match existing style, even if you would do it differently.
- If you notice unrelated dead code, mention it—do not delete it unless asked.

When your changes create orphans:

- Remove imports, variables, and functions that **your** changes made unused.
- Do not remove pre-existing dead code unless asked.
- Never edit generated dependency lockfiles manually. Regenerate `uv.lock` with `uv sync` or
  `uv lock`, and use the corresponding package-manager command for other lockfiles.

**Test:** Every changed line should trace directly to the user’s request.

### 4. Goal-driven execution

**Define success criteria. Loop until verified.**

Turn tasks into verifiable goals:

- “Add validation” → write tests for invalid inputs, then make them pass.
- “Fix the bug” → reproduce with a test (or clear steps), then make it pass.
- “Refactor X” → ensure tests pass before and after.

For multi-step tasks, state a brief plan:

```text
1. [Step] → verify: [check]
2. [Step] → verify: [check]
3. [Step] → verify: [check]
```

Strong success criteria allow independent iteration. Weak criteria (“make it work”) invite endless clarification.

---

## NOVA API (v2)

The SDK targets the **released** NOVA API v2. Treat the installed `wandelbots_api_client`
(`nova.api` re-exports `wandelbots_api_client.v2_pydantic`) as the source of truth — it matches
the released API, whereas an internal/nightly spec may contain fields that are not released yet.

**For anything involving the NOVA API — model field names, motion planning requests
(`plan_trajectory`, `MotionGroupSetup`, `LimitSet`, `LimitsOverride`), robot limits, or
torque/velocity/acceleration/jerk limits — use the `nova-api-v2` skill**
(`.agents/skills/nova-api-v2/SKILL.md`). It documents how to verify fields against the installed
client, the planning data flow, and the key limits/torque rules (e.g. `torque` is a
motion-group-global per-joint limit with no per-segment override, and controller-reported torque
is already forwarded to the planner via `global_limits`).

## Programs & NOVAx

`docs/programs.md` is the public quickstart for writing `@nova.program` functions and deploying
them with NOVAx. It is linked from other apps, so **keep it up to date whenever program or NOVAx
behavior changes.** Update it when you touch any of:

- The `@nova.program` decorator or `ProgramContext` (`nova/program/`) — e.g. required parameters,
  the input-schema derivation, preconditions.
- NOVAx registration, scanning, or serving (`novax/novax.py`, `novax/config.py`) —
  e.g. `Novax(...)` options, `programs_dir` scanning rules, `serve()`.
- Relevant environment variables (`NOVA_API`, `NOVA_ACCESS_TOKEN`, `CELL_NAME`, `BASE_PATH`).
- Deployment flow (`nova app create` / `nova app install`, Skaffold hot reload) or the
  `examples/your-nova-app/` scaffold it mirrors.

Verify example code and commands against the current SDK before editing, and keep them consistent
with `examples/your-nova-app/`.

## Bundled agent skills (`nova/agent_skills/`)

The SDK ships the customer-facing `nova-app-review` skill in the wheel. Customers install it with
`uv run nova-skills install` (`nova/helper_scripts/install_agent_skills.py`; documented in the
`README.md` section "Review your app with a coding agent"). It is **not** a skill
for working on this repository.

- When you change a public motion, IO, TCP, payload, program or exception API, update
  `nova/agent_skills/nova-app-review/references/nova-sdk-mapping.md` (symbols, gaps, snippets).
- When an SDK gap listed there is fixed (e.g. plan/execute timeouts, `MotionGroup.stop()`), replace
  the workaround with the SDK feature.
- Keep `scripts/scan.py` stdlib-only and Python 3.9 compatible; it runs in customer environments.
  Cover new scanner checks in `tests/nova/agent_skills/test_scan.py`.
- When you change the `nova-skills` command or its options, update the `README.md` section and the
  hint in `docs/programs.md`.

## novapolicy: the executor maps and transports

`PolicyExecutor` turns robot state, cameras and IOs into the observations the `PolicySchema`
declares, calls the policy, converts its actions back to NOVA (units, frames, relative targets),
and streams them as timestamped action chunks. How well the robot follows those waypoints
(interpolation, smoothing, limits, tracking) is NOVA's job, in its action chunk streaming.

- Do not build trajectory generation, filters, actuator models or controllers into the executor,
  the joggers or the jogging sessions. If the robot does not follow the commanded waypoints
  faithfully, fix or report it on the NOVA side. The only place to shape what is sent is a module
  of the send pipeline (`novapolicy/pipeline/`: `base.py` and `modules/`; chunk modules before
  `Timestamps()`, filters that may drop but never retime waypoints, values-only waypoint modules
  last). The built-ins (smoothing, timestep timeline, bridge, ramp, clamp, padding, dropping late
  waypoints; opt-in: `ServoHorizon`, `EaseIn`) are such modules; the opt-in cycle
  grid, `Timestamps(grid_ms=...)`, is a setting of the placement step, the one stage that times
  waypoints. `pipeline=None` runs `default_pipeline()`, an explicit list exactly what it lists. New
  shaping goes there as a module, never into the executor, the joggers or the session. Check
  `novapolicy/tests/api/test_wire_capture.py` stays unchanged when moving one.
- Transport timing is in scope: waypoint timestamps, the lead, loop pacing, and guards that stop
  a run loudly instead of degrading it silently.
- Light output smoothing is acceptable only if it stays small; prefer none.

### Customer docs and developer docs

The repository is customer facing. `novapolicy/README.md` and `novapolicy/docs/` explain how to
use the package: setup, verified snippets, options, troubleshooting. Measurements, design
rationale, alternatives tried, limitations, todos, ADR and contract references go to
`novapolicy/dev/` (`adr/`, `contracts.md`, `architecture.md`, `notes/`), never into the customer
docs. When a change needs a decision record or a measurement, add it under `dev/`; when it changes
usage, update `docs/`. Example docstrings and `--help` texts say what the example does and how to
run it, without measurements.

### Debugging a run from its Rerun recording

A run with a Rerun viewer can be saved as an `.rrd` file (viewer: File → Save). Read it as a
dataframe with Rerun's data access, the `catalog` extra (`datafusion`, `pandas`). Keep it out of the
project's extras and run it ad hoc, at the version `pyproject.toml` pins:

```bash
uv run --no-project --with "rerun-sdk[catalog]==0.43.2" python
```

```python
from rerun.server import Server

srv = Server(datasets={"run": ["run.rrd"]})
df = srv.client().get_dataset("run").filter_contents(["/policy/**"]).reader(index="log_time").to_pandas()
# columns are "<entity path>:<archetype>:<component>"; a scalar cell is a list of one value
```

- Filter with `/policy/**`: motion group ids appear escaped in entity paths (`0\@ur10e`).
- Timelines: `log_time` (wall clock), `policy_step` and `policy_time` (the policy loop), `state_tick`.
- `/policy/<mg>/tcp_error/position_norm_mm`: commanded against measured TCP per state packet; the
  commanded value is what the session actually sent for that moment. With `tcp_target/position/*`
  and `tcp_actual/position/*`, tell lag (the arm on the path but late) from leaving the path.
- `/policy/<mg>/joints/j*`: measured joints. `/policy/<mg>/waypoint_request/*`: per request,
  `scheduled_at_server_ms` (sent), `first_timestamp_ms` (first waypoint due; the difference is the
  lead), `last_timestamp_ms`, `dt_ms`, `count`, `action_timestep`. `/policy/action_chunks`: the
  policy's outputs.
- Without the extra, `rerun.chunk.RrdReader(path).stream()` yields the raw chunks
  (`chunk.entity_path`, `chunk.to_record_batch()`).
