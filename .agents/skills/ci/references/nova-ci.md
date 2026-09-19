# wandelbots-nova — CI map

Workflow files live in `.github/workflows/`. Most run on `pull_request` to
`main`. Match each failing workflow to the local command it runs.

## Setup for local repro

CI installs with `uv` and the same extras. Match it before reproducing:

```bash
pip install uv        # if uv not installed
uv sync --extra "nova-rerun-bridge" --extra "wandelscript" --extra "novax"
```

Python is pinned to **3.11** in the workflows (SDK supports `>=3.11, <3.13`).

## Always runs (broad paths)

| Workflow file | Job(s) | Local repro |
|---------------|--------|-------------|
| `nova-dev.yaml` | `test` (format, import order, lint, typecheck, unit tests) | see table below |
| `yamllint.yaml` | `yamllint` | `yamllint .` (config `.yamllint`) |
| `pr-title-check.yaml` | `validate-title` | Retitle PR to Conventional Commits |
| `uv-audit.yaml` | `uv audit --preview-features audit-command` |

### `nova-dev.yaml` steps → local commands

| CI step | Local command |
|---------|---------------|
| Check formatting with ruff | `uv run ruff format --check .` |
| Check import order | `uv run ruff check --select I` |
| Check ruff for linting | `uv run ruff check .` |
| Typecheck | `uv run ty check` |
| Run tests | `PYTHONPATH=. LOG_LEVEL=WARNING uv run pytest -rs -v -m "not integration"` |

Fixers: `uv run ruff format .` and `uv run ruff check --select I --fix`.

## Integration tests & examples (needs a live NOVA instance)

`nova-run-examples.yaml` spins up a temporary NOVA instance, runs integration
tests and each example as a separate matrix entry, then tears the instance down.
These require `NOVA_API` and `NOVA_ACCESS_TOKEN` and are usually only reproducible
against a real instance.

| Matrix entry | Command CI runs |
|--------------|-----------------|
| `integration` | `pytest -rs -v -m integration` |
| `start_here` | `python examples/start_here.py` |
| `plan_and_execute` | `python examples/plan_and_execute.py` |
| `move_multiple_robots` | `python examples/move_multiple_robots.py` |
| `run_wandelscript_file` | `python examples/run_wandelscript_file.py` |
| `welding` | `python examples/welding.py` |

Local repro (only if you have instance credentials):

```bash
export NOVA_API="https://<your-instance-host>"
export NOVA_ACCESS_TOKEN="<token>"
export CELL_NAME=cell
export BASE_PATH="/cell/novax"
PYTHONPATH=. uv run pytest -rs -v -m integration
PYTHONPATH=. uv run python examples/start_here.py
```

If an example fails after an SDK change, update the example to the current API
(see `docs/programs.md` and `examples/`), or fix the underlying regression.

## Security / audit

- `uv-audit.yaml` runs `uv audit` (experimental SAST) on PRs, pushes to `main`,
  and weekly. Fix by bumping the vulnerable dependency in `pyproject.toml`;
  escalate if no patched version exists.
- `codeql.yaml` and `dependency-review.yaml` are GitHub-native security scans —
  read the alert; changes are usually dependency bumps or code fixes.

## Docs (main only)

`autodocs.yaml` builds API docs with `pdoc` on push to `main` and deploys to
GitHub Pages. It does not gate PRs, but keep public docstrings valid:

```bash
uv run pdoc --docformat google nova nova_rerun_bridge -o ./docs
```

## Env vars used in CI

| Var | Where | Purpose |
|-----|-------|---------|
| `CELL_NAME` | unit + integration jobs | Cell name (`cell`) |
| `NOVA_API` | integration / examples | Base URL of the NOVA instance |
| `NOVA_ACCESS_TOKEN` | integration / examples | Auth token |
| `BASE_PATH` | examples | NOVAx base path (`/cell/novax`) |
| `PYTHONPATH=.` | tests / examples | Import the workspace packages |
