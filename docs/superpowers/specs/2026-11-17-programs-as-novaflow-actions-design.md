# Write once: `@nova.program` as NOVA program **and** NovaFlow action

Status: Draft for review · Date: 2026-11-17 · Repos: `wandelbots-nova` (SDK), `novaflow`

## 1. Goal

A developer writes **one** `@nova.program` function — typed inputs, typed **output** — and deploys
it **once**. It then:

- shows up in the NOVA frontend as a program (today's behavior), and
- shows up in NovaFlow (Canopy palette / trees) as a **typed node** whose inputs can be wired
  and whose outputs can be referenced by later nodes.

### End picture

**One NOVAx app = one NovaFlow service. One program = one node of that service.**

```text
NOVAx app "pick-cell"  (one container, one `nova app install`)
└── NovaFlow service "PickCell"
    ├── node "Pick"      ← @nova.program pick
    ├── node "Place"     ← @nova.program place
    └── node "Calibrate" ← @nova.program calibrate
```

- **Every** program becomes a node; none is skipped (§6.2).
- The service appears when the app starts and disappears when it stops. A redeploy or hot reload
  re-registers the service with the current set of programs.
- New apps get this by default: the scaffold ships with the plugin (§7).

### Constraints (from the user)

- Seamless for the author; simplify deployment.
- Change as little as possible; prefer SDK changes over NovaFlow changes.
- **No NovaFlow dependency in the SDK** (not even an optional extra in `wandelbots-nova`).
  Dependency direction is NovaFlow → SDK, or both → NOVA.

### Non-goals

- Replacing NovaFlow's `@router.action` / `ActionService` or the action template.
- Image-less "push a .py into the cell" deployment (noted as a later spike, §7.4).
- Changing the released NOVA API beyond one additive field (`Program.output_schema` in
  `service-manager`'s `program-discovery`, §8).

## 2. Current state (verified 2026-11-17, novaflow `main` @ `881645f`)

| Concern | `@nova.program` / NOVAx (SDK) | NovaFlow custom action |
|---|---|---|
| Authoring | `async def f(ctx, **typed)` + `@nova.program` | `@router.action(name, In, Out)` in an `ActionService` |
| Input schema | derived from signature (`Program.input_model`) | flat Pydantic model, structural `type_id`s |
| Output | `output_model` derived from return type **but the value is discarded**: `PythonProgramRunner._run` returns it, `_run_program` ignores it; `ProgramRun.output_data` is always `{}` | required typed output model, validated |
| Publication | `api.models.Program` (no `output_schema`) written to NATS KV `nova_cells_{cell}_programs` by NOVAx | registration in control-plane registry over NATS |
| Run events | SDK publishes `ProgramStatus` (no output) on `nova.v2.cells.{cell}.programs` | CloudEvents RUNNING/SUCCESS/FAILED on `svc.cmd.*` |
| Lifecycle | start/stop; **one run at a time per app** (`ProgramManager.start_program`) | `HaltToken`, `ctx.run_rest` (pause), progress, `moves_robot`, hold-before-motion |
| Runtime | uvicorn/FastAPI process | `asyncio.run` + multiprocessing worker pool (owns its process) |
| Deploy | `nova app create -g python_app` → Dockerfile + `.nova` + `nova app install` | copier template → Dockerfile + `.nova` + `nova app install` |

Existing bridge in NovaFlow: `actions/builtin/program` → `ProgramService.RunProgram`
(start via NOVA `ProgramApi`, wait for terminal event on `nova.v2.cells.{cell}.programs`,
halt ⇒ `stop_program`). `arguments` is a **JSON string**, output is only `{run, state}`.
Canopy already fetches a program's `input_schema` (control-plane `routes/programs.py`) to render
the argument form.

## 3. Target experience

```python
# programs/pick.py
import nova
from pydantic import BaseModel
from nova.types import Pose

class PickResult(BaseModel):
    picked: bool
    grasp_pose: Pose

@nova.program(name="Pick part", preconditions=...)
async def pick(ctx: nova.ProgramContext, slot: int, speed: float = 200.0) -> PickResult:
    ...
    return PickResult(picked=True, grasp_pose=pose)
```

```bash
nova app install .   # unchanged
```

- **Without** the plugin (Phase A): `pick` is a NOVA program; in NovaFlow the `RunProgram` node
  pointed at `my-app.pick` runs it and returns its output as JSON (`output.picked`,
  `output.grasp_pose`), untyped.
- **With** `wandelbots-nova-novaflow` added to the app's dependencies (Phase C): additionally,
  `pick` becomes a native NovaFlow action `MyApp.Pick` with typed, wireable `slot`/`speed` inputs
  and `picked`/`grasp_pose` outputs, halt, pause points and progress — in the same container, no
  second project.

### Ownership

NovaFlow is owned by the BA team. To keep their work small, the typed path is the plugin, which
we own as a separate package in this repo. NovaFlow itself only needs one small `RunProgram`
change (A.1, plus the optional A.3), which we open as PRs for BA to review.

## 4. Phase 0 — SDK program output contract (shared by A and C)

All in `wandelbots-nova`. Additive; no behavior change for programs without a return annotation.

1. **Add a published output schema** (`nova/program/function.py`). `output_model` and the existing
   `Program.output_schema` stay unchanged (a test asserts `{"type": "number"}` for `-> float`).
   Add `Program.output_json_schema`, which is always an object schema or `None`:
   - Return annotation is a `BaseModel` → its schema.
   - No annotation or `-> None` → `None` (no output).
   - Any other type `T` → `{"result": T}`. NovaFlow requires object outputs with required fields.
2. **Capture the return value** (`nova/program/runner.py`): keep the result of `_run`, validate it
   against `output_model`, and normalize it with the same rules into a **new**
   `ProgramRun.output: dict | None`. Do not reuse `output_data`: the wandelscript runner already
   stores its variable store there (`wandelscript/runner.py:68`). If validation fails, the run is
   `FAILED` with a clear error.
3. **Publish the output on the terminal event**: add an optional `output: dict | None` to
   `ProgramStatus`, set only on `COMPLETED`. Cap the size (e.g. 128 KiB, same as the store's
   message size); if it's larger, fail with "return an object-store reference instead".
4. **Publish the output schema in the store**: NOVAx writes an SDK-side subclass of
   `api.models.Program` with an extra `output_schema` to `nova_cells_{cell}_programs`. NOVAx
   `GET /programs/{id}` returns it too. Verified safe: NOVA's Program API ignores the extra field
   (§8, Q1).
5. **Robot hint** (helps NovaFlow's `moves_robot` / `robot_field` contract, ADR-034): derive
   "controllers this program moves" from `preconditions.controllers`. It's already in the store,
   so no new field is needed unless we add an explicit `@nova.program(moves=...)` (decide later;
   YAGNI).
6. **Concurrency (decided: allow parallel):** `ProgramManager` tracks runners per program instead
   of one global runner, so different programs of one app can run at the same time (NovaFlow
   `parallel` nodes). A second run of the *same* program is still rejected with a clear "program
   already running" error, because NOVA's stop endpoint addresses a program, not a run (§8, Q5).
   The NOVAx app's shutdown stops all running programs.
7. **Docs/tests:** update `docs/programs.md` (required by `AGENTS.md`): typed outputs, the
   NovaFlow section. Unit tests: output capture, validation failure, scalar wrapping,
   `None`-output, size cap, and the `ProgramStatus` serialization golden.

## 5. Phase A — NovaFlow consumes what NOVA/SDK publishes

Small NovaFlow change (BA team reviews, we open the PRs); no new process, no new deployment.

1. **`RunProgram` returns the program output.** Read `output` from the terminal event it already
   receives, and return it as an untyped JSON output field `output`. Older SDKs send no `output`
   → `{}`.
2. **Default `controller` (optional)** from the program's preconditions when it names exactly one
   controller. This closes the "RunProgram: name the robot it moves" warning (`continue-run.md`
   §4.7).
3. **Semantics, documented:** halt = stop program (exists). Pause: the program is one step that
   "can't pause, finishes first". No hold-before-motion inside a program.
4. **Tests:** a component test with a fake program-run publisher (output round-trip, FAILED).

**Dropped: per-program types on `RunProgram`.** Typing `arguments` and `output` from the selected
program's schemas would need per-node types in the control-plane validator and Canopy, which
NovaFlow does not have today (M–L work for the BA team). The plugin (Phase C) gives the same
typed, wireable nodes, so this is deferred until someone needs typed programs *without* the
plugin.

## 6. Phase C — NovaFlow plugin for NOVAx (in-process, native actions)

### 6.1 SDK: a NovaFlow-agnostic extension hook (small)

- `Novax` discovers extensions through the `importlib.metadata` entry-point group
  `novax.extensions`. It also accepts `Novax(extensions=[...])` for explicit or test use, and
  `NOVAX_EXTENSIONS=none` to disable discovery.
- Minimal protocol, versioned (`NOVAX_EXTENSION_API = 1`):
  `async on_startup(ctx: NovaxExtensionContext)` and `async on_shutdown()`.
  `NovaxExtensionContext` exposes `app_name`, `cell_name`, `programs: list[Program]`,
  `programs_dir`, `nova_config`.
- Called from the existing `program_store_lifespan` / `serve()` path. Extension errors are
  logged, and the app still starts.
- **Host hooks on `ProgramContext`** (optional, so one program can use pause and progress):
  `await ctx.checkpoint()` and `await ctx.report_progress(i, msg)`. These are no-ops (or log
  lines) under NOVA. A host such as the NovaFlow plugin injects real implementations through a
  private setter. The SDK still imports nothing from NovaFlow.

### 6.2 Plugin package `wandelbots-nova-novaflow` (this repo, `nova_novaflow/`, owned by us)

A separate package next to `novapolicy/`, with its own `pyproject.toml`. It depends on
`wandelbots-nova` and `wandelbots-novaflow` (pinned to a NovaFlow release, ADR-034). The
`wandelbots-nova` package itself still has no NovaFlow dependency. It uses only NovaFlow's public
SDK (`ActionService`, `Router`, `ActionContext`, `HaltToken`, `ActionFailure` from
`novaflow.core`), like any action service built from the action template.

- Registered under the `novax.extensions` entry point. `on_startup` spawns **a child process**
  running an `ActionService`, because `ActionService.run()` owns its event loop and worker pool.
  `on_shutdown` terminates it and deregisters.
- **One service per app:** service name = PascalCase(`APP_NAME`), e.g. `pick-cell` → `PickCell`
  (NovaFlow requires `^[A-Z][A-Za-z0-9]*$`).
- **One node per program, always:** action name = PascalCase(program id), e.g. `pick` → `Pick`.
  A leading digit gets a `P` prefix. Two programs that map to the same name fail app startup with
  a clear error.
  - `input_model` = `program.input_model`; the output model follows the `output_json_schema` rules
    (§4.1). `nova.types.Pose` fields are swapped for the api-client `Pose`, and values are
    converted with `to_api_model()` (§8, Q3).
  - A field whose type NovaFlow can't identify (e.g. `dict[str, float]`, a heterogeneous tuple)
    is published as `Any` with a startup warning, instead of skipping the program. NovaFlow
    accepts `Any` fields (verified against `NodeSchema._extract_fields` on novaflow `881645f`).
    Such a field can still be passed as a value, but is not type-checked when wired.
  - `moves_robot`/`robot_field` come from preconditions; `holds_before_motion=False` initially.
- **Lifecycle:** the service registers when the app starts and is removed from NovaFlow's
  registry when it stops (NovaFlow's SDK already deletes the registry keys on shutdown). A
  redeploy or hot reload re-registers it with the current set of programs.
- Handler (runs in a worker subprocess): re-import `programs_dir`, look up the program by id,
  open `Nova()`, build a `ProgramContext`, and call the program. `HaltToken` → cancel the
  program task, and the robot cell stops, as on the NOVA stop path. `ctx.checkpoint()` →
  `await action_ctx.run_rest.wait(False)`. `ctx.report_progress` → `action_ctx.report_progress`.
  Exceptions → `ActionFailure`.
- **Custom actions in the same app:** the plugin also includes plain NovaFlow routers named in
  config (e.g. `[tool.novax.novaflow] routers = ["app.actions:router"]`). One app can then host
  programs *and* hand-written `@router.action`s.
- **Duplicates accepted:** a program served by the plugin also stays listed in the `RunProgram`
  picker. Hiding it would need a Canopy change. Once every app ships the plugin, `RunProgram` is
  only needed for older apps; BA can deprecate it then.
- **Compatibility:** CI tests the plugin against the pinned NovaFlow SDK release; the SDK hook is
  versioned by `NOVAX_EXTENSION_API`. Optionally, BA adds the plugin to the SDK matrix of
  NovaFlow's `test_external_action_contract.py`.
- **Tests:** a component test running a NOVAx app with a fake program through the plugin
  (register → dispatch → output → halt → pause point) against a NATS testcontainer.

## 7. Deployment simplification

1. **One app for both** (from C): programs and custom NovaFlow actions in one NOVAx project →
   one scaffold, one image, one `nova app install`. The copier action template remains for pure
   action services.
2. **`novax` console script** (SDK): `novax serve [--programs-dir programs]`. It reads `NOVA_API`,
   `CELL_NAME`, `BASE_PATH` (→ `root_path`), `static/`, and loads extensions. The scaffold
   shrinks to `programs/`, `pyproject.toml`, `uv.lock`, `.nova`, and a ~6-line Dockerfile ending
   in `CMD ["novax", "serve"]`. `app/__main__.py` and `register_programs.py` go away. Update
   `examples/your-nova-app/` and the nova-cli `python_app` generator (separate repo).
3. **NovaFlow by default:** the scaffold (`examples/your-nova-app/` and the nova-cli
   `python_app` generator) lists `wandelbots-nova-novaflow` as a dependency, so every new app is
   a NovaFlow service with one node per program. The entry point activates it; no code is
   needed. Existing apps add it with `uv add wandelbots-nova-novaflow`; removing it turns the
   service off. `wandelbots-nova` itself still has no NovaFlow dependency.
4. **Later spike (out of scope):** image-less deploy, where a shared runtime app loads program
   files from the NATS object store (`nova program push`). Open issues: dependency management
   and security.

## 8. Open questions / risks

Answered on 2026-11-17 from code (`service-manager` @ `c229a19e1`, novaflow `main` @ `881645f`):

1. **NOVA store tolerance — answered: safe, but dropped.** NOVA's Program API is
   `service-manager/services/program-discovery`. It reads the KV through the SDK's own
   `ProgramStore` (pinned `wandelbots-nova==2.20.4`); `api.models.Program` ignores unknown
   fields, and `program-discovery` re-maps entries to its own `Program` model, which has no
   `output_schema`. So an extra `output_schema` never hides a program, but NOVA's API does not
   return it. → Add `output_schema` to `program-discovery`'s `Program` model and
   `api/v2/components/schemas/Program.yaml`, and bump its SDK pin (task S.1).
2. **NOVA API ask — answered: we can do it ourselves.** The Program API lives in
   `service-manager`, so S.1 is the NOVA API change. `ProgramRun.output` is not needed there:
   NovaFlow reads the output from the NATS event.
3. **Output type identity — answered: mismatch, the SDK must map it.** `nova.types.Pose` is a
   different class from the api-client `Pose` and adds an optional `kinematic_configuration`.
   NovaFlow rejects a field class named `Pose` that is not the registered class (ADR-029
   "no-shadows" rule in `NodeSchema._extract_fields`), and its JSON schema would get a different
   structural id. Its serialized value (`{position, orientation}`) does validate as the api
   `Pose`. → The SDK publishes the api `Pose` schema for `nova.types.Pose` fields in
   `output_json_schema` (task 0.5); the plugin swaps the type and converts values with
   `to_api_model()`.
4. **Duplicate visibility — answered: accept.** Plugin-served programs also stay in the
   `RunProgram` picker; hiding them needs a Canopy change.
5. **Concurrency of the same program — answered: reject.** The released API has only
   `stop_program(cell, program)`; there is no run id to stop one of two runs. Different
   programs run in parallel (0.6).
6. **Worker re-import cost / spawn method** in NovaFlow's worker pool with heavy program modules.
   Measure in C.2.
7. **Owner of the NovaFlow side — answered: BA team.** Their scope is reduced to A.1 (+ optional
   A.3), which we open as PRs; the plugin is ours (§3 Ownership).

## 9. Work breakdown

| # | Repo | Item | Size | Depends on |
|---|---|---|---|---|
| 0.1 | SDK | Add `Program.output_json_schema` (object or None) | S | — |
| 0.2 | SDK | Capture + validate return value into new `ProgramRun.output` | S | 0.1 |
| 0.3 | SDK | `ProgramStatus.output` on COMPLETED (+ size cap, golden) | S | 0.2 |
| 0.4 | SDK | `output_schema` in store entry + NOVAx GET | S | 0.1 |
| 0.5 | SDK | Publish api `Pose` schema for `nova.types.Pose` fields | S | 0.1 |
| 0.6 | SDK | Parallel runs of different programs per app | S | — |
| 0.7 | SDK | Docs `docs/programs.md`, example with typed output | S | 0.1–0.4 |
| C.1 | SDK | `novax.extensions` hook + `NovaxExtensionContext` | S | — |
| C.1b | SDK | `ctx.checkpoint()` / `ctx.report_progress()` host hooks | S | C.1 |
| D.2 | SDK | `novax serve` console script | S | C.1 |
| D.2b | SDK + nova-cli | Slim scaffold (`examples/your-nova-app`, generator) | S–M | D.2 |
| C.2 | plugin (this repo) | `wandelbots-nova-novaflow`: child-process ActionService, program→action adapter, Pose swap | M–L | C.1, 0.1, 0.5 |
| C.2b | plugin (this repo) | Extra routers from config | S | C.2 |
| C.3 | plugin (this repo) | Component tests incl. halt + pause, CI against pinned NovaFlow SDK | M | C.2 |
| S.1 | service-manager (us) | `output_schema` in `program-discovery` Program API, bump SDK pin | S | 0.4 |
| A.1 | NovaFlow (PR by us, BA reviews) | `RunProgram` returns untyped `output` + component test | S | 0.3 |
| A.3 | NovaFlow (PR by us, BA reviews, optional) | Default `controller` from preconditions | S | — |
| ~~A.2~~ | NovaFlow | ~~Per-program types on `RunProgram`~~ — dropped, plugin covers it | — | — |

**Suggested order:** 0.1→0.7 ∥ C.1, C.1b, D.2 (one SDK release) → C.2 → C.3 (plugin release)
∥ A.1, A.3, S.1 → D.2b.
