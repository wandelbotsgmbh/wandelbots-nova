# Write once: `@nova.program` as NOVA program **and** NovaFlow action

Status: Draft for review · Date: 2026-11-17 · Repos: `wandelbots-nova` (SDK), `novaflow`

## 1. Goal

A developer writes **one** `@nova.program` function — typed inputs, typed **output** — and deploys
it **once**. It then:

- shows up in the NOVA frontend as a program (today's behavior), and
- shows up in NovaFlow (Canopy palette / trees) as a **typed node** whose inputs can be wired
  and whose outputs can be referenced by later nodes.

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

- **Without** NovaFlow in the app (Phase A): `pick` is a NOVA program; in NovaFlow the
  `RunProgram` node pointed at `my-app.pick` gets typed `slot`/`speed` inputs and typed
  `picked`/`grasp_pose` outputs.
- **With** `wandelbots-novaflow` added to the app's dependencies (Phase C): additionally, `pick`
  becomes a native NovaFlow action `MyApp.Pick` with halt, pause points and progress — in the
  same container, no second project.

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

Mostly NovaFlow; no new process, no new deployment.

1. **`RunProgram` returns the program output.** Read `output` from the terminal event it already
   receives, and return it as an output field `output`.
2. **Program-parameterized node types.** The node's input and output types derive from the
   selected program's `input_schema` / `output_schema`, the same way Canopy already derives the
   argument form:
   - `arguments` becomes an object whose fields can be wired (`{ref: "output", ...}`), instead
     of a JSON string. Keep reading the string form for stored trees.
   - The control-plane validator resolves `RunProgram` output types per node from the program
     schema, so `$outputs.<node>.output.grasp_pose` type-checks with structural `type_id`s
     (ADR-029).
   - Schema source: the NOVAx app's `GET /programs/{id}` until the NOVA Program API
     (`program-discovery`) carries `output_schema` (§8, S.1).
3. **Default `controller`** from the program's preconditions when it names exactly one
   controller. This closes the "RunProgram: name the robot it moves" warning (`continue-run.md`
   §4.7).
4. **Semantics, documented:** halt = stop program (exists). Pause: the program is one step that
   "can't pause, finishes first". No hold-before-motion inside a program.
5. **Tests:** a component test with a fake program-run publisher (output round-trip, oversized
   output, FAILED), plus an e2e test against a real NOVAx app in the k3d cell.

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

### 6.2 NovaFlow: `novaflow.integrations.novax` (in `wandelbots-novaflow[novax]`, which depends on `wandelbots-nova`)

- Registered under the `novax.extensions` entry point. `on_startup` spawns **a child process**
  running an `ActionService`, because `ActionService.run()` owns its event loop and worker pool.
  `on_shutdown` terminates it and deregisters.
- Service name = PascalCase(`APP_NAME`). One action per program: `Pick` ← `pick`.
  - `input_model` = `program.input_model`; the output model follows the `output_json_schema` rules
    (§4.1).
  - Programs whose models violate NovaFlow's flat/supported-type rules are **skipped with a
    warning**; they remain reachable via Phase A.
  - `moves_robot`/`robot_field` come from preconditions; `holds_before_motion=False` initially.
- Handler (runs in a worker subprocess): re-import `programs_dir`, look up the program by id,
  open `Nova()`, build a `ProgramContext`, and call the program. `HaltToken` → cancel the
  program task, and the robot cell stops, as on the NOVA stop path. `ctx.checkpoint()` →
  `await action_ctx.run_rest.wait(False)`. `ctx.report_progress` → `action_ctx.report_progress`.
  Exceptions → `ActionFailure`.
- **Custom actions in the same app:** the plugin also includes plain NovaFlow routers named in
  config (e.g. `[tool.novax.novaflow] routers = ["app.actions:router"]`). One app can then host
  programs *and* hand-written `@router.action`s.
- **No duplicates:** when the plugin is active, mark the store entry so Canopy hides those
  programs from the `RunProgram` picker and offers the native nodes instead. Store flag name
  TBD in §8, Q4.
- **Compatibility:** the plugin ships with the NovaFlow release (ADR-034 pins apply); the SDK
  hook is versioned by `NOVAX_EXTENSION_API`.
- **Tests:** a NovaFlow component test running a NOVAx app with a fake program through the
  plugin (register → dispatch → output → halt → pause point), and an entry in
  `test_external_action_contract.py`'s SDK matrix.

## 7. Deployment simplification

1. **One app for both** (from C): programs and custom NovaFlow actions in one NOVAx project →
   one scaffold, one image, one `nova app install`. The copier action template remains for pure
   action services.
2. **`novax` console script** (SDK): `novax serve [--programs-dir programs]`. It reads `NOVA_API`,
   `CELL_NAME`, `BASE_PATH` (→ `root_path`), `static/`, and loads extensions. The scaffold
   shrinks to `programs/`, `pyproject.toml`, `uv.lock`, `.nova`, and a ~6-line Dockerfile ending
   in `CMD ["novax", "serve"]`. `app/__main__.py` and `register_programs.py` go away. Update
   `examples/your-nova-app/` and the nova-cli `python_app` generator (separate repo).
3. **Opt-in NovaFlow** is one line: `uv add "wandelbots-novaflow[novax]"`. No code change; the
   entry point activates it.
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
   `api/v2/components/schemas/Program.yaml`, and bump its SDK pin (task S.1). Until that ships,
   NovaFlow reads the schema from the NOVAx app (`GET /programs/{id}`).
2. **NOVA API ask — answered: we can do it ourselves.** The Program API lives in
   `service-manager`, so S.1 is the NOVA API change; it needs a NOVA release before NovaFlow can
   rely on it. `ProgramRun.output` is not needed there: NovaFlow reads the output from the NATS
   event.
3. **Output type identity — answered: mismatch, the SDK must map it.** `nova.types.Pose` is a
   different class from the api-client `Pose` and adds an optional `kinematic_configuration`.
   NovaFlow rejects a field class named `Pose` that is not the registered class (ADR-029
   "no-shadows" rule in `NodeSchema._extract_fields`), and its JSON schema would get a different
   structural id. Its serialized value (`{position, orientation}`) does validate as the api
   `Pose`. → The SDK publishes the api `Pose` schema for `nova.types.Pose` fields in
   `output_json_schema`; the NovaFlow plugin swaps the type and converts values with
   `to_api_model()` (task 0.5).
4. **Duplicate visibility (A vs C):** store flag name and Canopy behavior. Decide in C.2b.
5. **Concurrency of the same program — answered: reject.** The released API has only
   `stop_program(cell, program)`; there is no run id to stop one of two runs. Different
   programs run in parallel (0.6).
6. **Worker re-import cost / spawn method** in NovaFlow's worker pool with heavy program modules.
   Measure in C.2.
7. **Owner of the NovaFlow side (A, C):** open.

## 9. Work breakdown

| # | Repo | Item | Size | Depends on |
|---|---|---|---|---|
| 0.1 | SDK | Add `Program.output_json_schema` (object or None) | S | — |
| 0.2 | SDK | Capture + validate return value into new `ProgramRun.output` | S | 0.1 |
| 0.3 | SDK | `ProgramStatus.output` on COMPLETED (+ size cap, golden) | S | 0.2 |
| 0.4 | SDK | `output_schema` in store entry + NOVAx GET | S | 0.1 |
| 0.5 | SDK | Publish api `Pose` schema for `nova.types.Pose` fields | S | 0.1 |
| S.1 | service-manager | `output_schema` in `program-discovery` Program API, bump SDK pin | S | 0.4 |
| 0.6 | SDK | Parallel runs of different programs per app | S | — |
| 0.7 | SDK | Docs `docs/programs.md`, example with typed output | S | 0.1–0.4 |
| A.1 | NovaFlow | `RunProgram` returns `output` | S | 0.3 |
| A.2 | NovaFlow | Program-parameterized input/output types (validator + Canopy) | M–L | 0.4, A.1 |
| A.3 | NovaFlow | Default `controller` from preconditions | S | — |
| A.5 | NovaFlow | Component + e2e tests | M | A.1–A.3 |
| C.1 | SDK | `novax.extensions` hook + `NovaxExtensionContext` | S | — |
| C.1b | SDK | `ctx.checkpoint()` / `ctx.report_progress()` host hooks | S | C.1 |
| C.2 | NovaFlow | `novaflow.integrations.novax` plugin (child-process ActionService, program→action adapter) | M–L | C.1, 0.1 |
| C.2b | NovaFlow | Extra routers from config, duplicate hiding | S–M | C.2 |
| C.3 | NovaFlow | Compat/e2e tests incl. halt + pause | M | C.2 |
| D.2 | SDK | `novax serve` console script | S | C.1 |
| D.2b | SDK + nova-cli | Slim scaffold (`examples/your-nova-app`, generator) | S–M | D.2 |

**Suggested order:** 0.1→0.6 (SDK release) ∥ C.1, D.2 (SDK) → A.1, A.3 (quick win) and S.1 →
C.2 → A.2 → D.2b.
