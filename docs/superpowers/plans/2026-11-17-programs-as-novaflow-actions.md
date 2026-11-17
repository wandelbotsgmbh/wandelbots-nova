# Implementation plan: `@nova.program` as NOVA program and NovaFlow action

Spec: [../specs/2026-11-17-programs-as-novaflow-actions-design.md](../specs/2026-11-17-programs-as-novaflow-actions-design.md)

Conventions: every SDK task is test-first (write failing test → implement → green), run
`uv run ruff format && uv run ruff check . && uv run ty check`, and the targeted tests
`PYTHONPATH=. uv run pytest -rs -v <files>`. One PR per milestone, Angular titles
(`feat(program): …`). NovaFlow tasks are listed at task level and are planned in detail in the
`novaflow` repo.

## Milestone 0 — Spikes (resolve before M1.4 and M3.2)

- [ ] **S1 (Q1) NOVA store tolerance.** On a live 26.6 cell, put a `Program` entry with an extra
  `output_schema` key into `nova_cells_{cell}_programs`. Check `GET /cells/{cell}/programs` and
  `/programs/{id}` (via the api-client) and the NOVA frontend program list. Outcome: "extra key is
  tolerated / dropped / rejected" → decides the M1.4 storage path.
- [ ] **S2 (Q3) Pose type identity.** Compare the JSON schema of `nova.types.Pose` (as a program
  return field) with the structural `type_id` NovaFlow assigns to its `Pose`
  (`novaflow.types`, ADR-029). Outcome: "matches" or "SDK must publish the api `Pose` schema for
  `nova.types.Pose` fields" → becomes an extra M1 task if needed.

## Milestone 1 — SDK program output contract (`feat(program): typed program outputs`)

### M1.1 `Program.output_json_schema`
- Files: `nova/program/function.py`, `tests/program/test_function.py`.
- Tests:
  - `-> SomeModel` → the model's schema (`type: object`).
  - `-> float` → `{"type": "object", "properties": {"result": {"type": "number", …}},
    "required": ["result"]}`.
  - no annotation / `-> None` → `None`.
  - `Annotated[float, Field(description=…)]` keeps the description on `result`.
  - The existing `output_schema` test stays unchanged.
- Impl: a property next to `output_schema`, built from `output_model` (unwrap the `RootModel`
  root annotation; `NoneType`/missing → `None`).

### M1.2 Capture the return value into `ProgramRun.output`
- Files: `nova/program/runner.py`, `tests/program/test_runner.py`.
- Tests (use `robot_cell_override` / a no-op program as existing runner tests do):
  - Program returning a `BaseModel` → `runner.program_run.output == model.model_dump(mode="json")`.
  - Returning `3.5` with `-> float` → `{"result": 3.5}`.
  - `-> None` → `output is None`.
  - Returning the wrong type → state `FAILED`, the error mentions output validation.
  - A wandelscript run still fills `output_data` and leaves `output is None`.
- Impl: add `output: dict[str, Any] | None = None` to the SDK `ProgramRun`. Make `_run` return
  its value through `_run_program`. Add a `Program.serialize_output(value) -> dict | None` helper
  in `function.py` that shares the M1.1 rules, so both are tested in one place. Only
  `PythonProgramRunner` sets `output`.

### M1.3 Publish the output on the terminal `ProgramStatus`
- Files: `nova/program/runner.py`, `tests/novax/api/test_program_run.py`.
- Tests:
  - COMPLETED message carries `output`; RUNNING/FAILED/STOPPED carry `output: null`.
  - Output larger than `PROGRAM_OUTPUT_MAX_BYTES` (128 KiB) → run `FAILED` with a "return an
    object-store reference" message, and no oversized NATS publish.
  - Old consumers: `ProgramStatus.model_validate_json` of a message without `output` still
    works (additive).
- Impl: `output: dict[str, Any] | None = None` on `ProgramStatus`, filled in `program_status`
  when state is COMPLETED. Check the size before the COMPLETED transition.

### M1.4 `output_schema` in the program store and NOVAx API
- Depends on S1. Files: `novax/program_manager.py`, `novax/novax.py`, `novax/api/programs.py`,
  `nova/program/store.py`, `tests/novax/api/test_program_registry.py`.
- If S1 = tolerated: an SDK model `ProgramDescriptor(api.models.Program)` with
  `output_schema: dict | None`. Use it in `ProgramManager`, `ProgramStore`, and the NOVAx
  `response_model`s. Tests: registry round-trip includes `output_schema`; GET `/programs/{id}`
  returns it.
- If S1 = dropped/rejected: keep the store payload unchanged and expose `output_schema` only via
  NOVAx `GET /programs/{id}`. NovaFlow reads it from the app (M3.2 adapts).

### M1.5 Parallel runs of different programs
- Files: `novax/program_manager.py`, `novax/novax.py`, `novax/api/programs.py`,
  `tests/novax/test_program_manager.py`.
- Tests:
  - Two different programs start and run at the same time; each reaches COMPLETED with its own
    run id and output.
  - Starting a program that is already running → clear "program already running" error.
  - Stopping one program leaves the other running.
  - App shutdown stops all running programs.
- Impl: replace the single `_runner` with `dict[program_id, ProgramRunner]`; drop finished
  runners. `running_program` becomes `running_programs`; keep the old property as a deprecated
  alias that returns the first running program.

### M1.6 Docs and example
- `docs/programs.md`: a "Program outputs" section (return a model or a scalar, the `result`
  wrapping, the size limit), and a short "Use in NovaFlow" section (filled in further in M2/M5).
- `examples/your-nova-app/app/programs/start_here.py` or a new example returning a typed result.
- `CHANGELOG` comes from release-please; nothing manual.

**M1 exit:** SDK release with typed outputs; `ProgramStatus.output` + `output_schema` are
contract fields (note in `nova-api-v2` skill / AGENTS.md that NovaFlow consumes them).

## Milestone 2 — SDK extension hook + `novax serve` (`feat(novax): extensions and serve CLI`)

Independent of M1; can run in parallel.

### M2.1 `novax.extensions` entry-point hook
- Files: new `novax/extensions.py`, `novax/novax.py`, `novax/__init__.py`,
  `tests/novax/test_extensions.py`.
- API (no NovaFlow names anywhere):
  ```python
  NOVAX_EXTENSION_API = 1

  @dataclass(frozen=True)
  class NovaxExtensionContext:
      app_name: str
      cell_name: str | None
      programs: list[Program]
      programs_dir: Path | None
      nova_config: NovaConfig

  class NovaxExtension(Protocol):
      api_version: int
      async def on_startup(self, ctx: NovaxExtensionContext) -> None: ...
      async def on_shutdown(self) -> None: ...
  ```
- `Novax(..., extensions: list[NovaxExtension] | None = None)`. Default `None` = discover via
  `importlib.metadata.entry_points(group="novax.extensions")`; `NOVAX_EXTENSIONS=none` disables
  discovery.
- Run `on_startup` after programs are registered, and `on_shutdown` before store
  deregistration. Use the lifespan path for both `include_programs_router` and `serve()`. Also
  run them when `CELL_NAME` is unset (local dev), which today has no lifespan, so add one.
- Tests:
  - An explicit extension receives every program.
  - An extension that raises is logged, and the app still starts.
  - A wrong `api_version` is skipped with a warning.
  - Entry-point discovery, via monkeypatched `entry_points`.
  - `NOVAX_EXTENSIONS=none` disables discovery.

### M2.2 Host hooks on `ProgramContext`
- Files: `nova/program/context.py`, `tests/program/test_context.py` (new).
- `await ctx.checkpoint()` → no-op by default. `await ctx.report_progress(index: int, message:
  str = "")` → `logger.info` by default. A private `_set_host(host)` lets an extension inject
  implementations. Tests: defaults are harmless; an injected host receives the calls.
- Docs: `docs/programs.md` "Pause points and progress".

### M2.3 `novax serve` console script
- Files: `pyproject.toml` (`[project.scripts] novax = "novax.cli:main"`; the CLI uses
  `argparse`, so no new dependency), new `novax/cli.py`, `tests/novax/test_cli.py`.
- `novax serve [--programs-dir programs] [--host] [--port 3000] [--static-dir static]`.
  `root_path` defaults to `BASE_PATH`. It calls `Novax(programs_dir=…).serve(...)`.
- Tests: argument parsing → the `serve` kwargs (monkeypatch `Novax.serve`).

### M2.4 Slim scaffold
- `examples/your-nova-app/`: replace `app/__main__.py` + `register_programs.py` with
  `CMD ["novax", "serve", "--programs-dir", "app/programs"]`. Keep `static/`, `.nova`,
  `skaffold.yaml` (hot reload: `novax serve --reload` passthrough if needed). Update `README.md`
  and `docs/programs.md` §3 and §5.
- Follow-up in the `nova-cli` repo: the `python_app` generator mirrors the slim scaffold (issue,
  not in this plan's PRs).

## Milestone 3 — NovaFlow Phase A (repo `novaflow`, after M1 release)

- [ ] **M3.1** `ProgramService.RunProgram`: add an output field `output` (type `Any` / JSON
  object) filled from the terminal event's `output`. Older SDKs send no `output` → `{}`. Tests in
  `actions/builtin/program/tests/test_program_actions.py`.
- [ ] **M3.2** Program-parameterized types. Control plane: `routes/programs.py` returns
  `output_schema` (source per S1). The validator resolves the `RunProgram` node's `arguments`
  fields and `output` sub-fields from the selected program's schemas (structural `type_id`s).
  Canopy: argument fields become wireable, output fields are referenceable. Keep accepting the
  legacy string `arguments`.
- [ ] **M3.3** Default `controller` from the program's single precondition controller (Canopy +
  validation warning text).
- [ ] **M3.4** Component tests and an e2e test with a real NOVAx app in the k3d cell (output
  round-trip, FAILED, halt → STOPPED).

## Milestone 4 — NovaFlow Phase C plugin (repo `novaflow`, after M2 release)

- [ ] **M4.1** `novaflow.integrations.novax` in `wandelbots-novaflow`, extra `[novax]` →
  `wandelbots-nova>=<M2 release>`. Entry point `novax.extensions = novaflow =
  novaflow.integrations.novax:extension`.
- [ ] **M4.2** Program → action adapter:
  - Service name = PascalCase(app); action name = PascalCase(program id).
  - Validate input/output models against NovaFlow's flat/supported-type rules; skip and warn
    otherwise.
  - `moves_robot` + `robot_field` from preconditions.
  - Handler: re-import `programs_dir` in the worker, build `Nova` + `ProgramContext`, inject the
    host (`checkpoint` → `run_rest.wait(False)`, `report_progress` → `ActionContext`), and run
    with `HaltToken` → cancel. Errors → `ActionFailure`.
- [ ] **M4.3** Run the `ActionService` in a child process started from `on_startup`;
  `on_shutdown` terminates it (graceful halt, then kill).
- [ ] **M4.4** Extra routers from `[tool.novax.novaflow] routers = [...]`; store flag so Canopy
  hides the duplicate `RunProgram` entries for plugin-served programs.
- [ ] **M4.5** Tests: component (register → dispatch → output → halt → pause point) and an
  `_SDKS` matrix entry in `test_external_action_contract.py`.

## Milestone 5 — Docs and rollout

- [ ] `docs/programs.md`: "Use your programs in NovaFlow". Without the plugin, `RunProgram`
  works automatically; with the plugin, `uv add "wandelbots-novaflow[novax]"` gives native
  nodes. Include a decision table.
- [ ] NovaFlow docs: `novaflow/README.md` section pointing to programs as the
  "write once" path.
- [ ] Ask the NOVA team for `Program.output_schema` + `ProgramRun.output` in the released API
  (link this spec).

## Order and parallelism

```text
S1, S2 ──┐
M1.1 → M1.2 → M1.3 ─┬→ M1.4(S1) → M1.5 → M1.6 → [SDK release] → M3.1 → M3.3 → M3.2(S2) → M3.4
M2.1 → M2.2 ────────┤                                   → M4.1 → M4.2 → M4.3 → M4.4 → M4.5
M2.3 → M2.4 ────────┘                                                                 → M5
```
