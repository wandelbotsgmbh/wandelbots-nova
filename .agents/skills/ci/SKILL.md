---
name: ci
description: "Read GitHub Actions CI for the current branch or open PR, triage failing checks, reproduce locally, and fix scoped issues until green. Use whenever the user mentions CI failures, red checks, failed pipeline, GitHub Actions, merge blocked, fix the build, babysit CI, gh run failed, ruff/ty errors, failing pytest, example run failed, or asks to check CI on this branch — even if they only paste a log snippet. Also use after implementing a feature when tests, lint, or typecheck might have broken."
---

# Fix branch CI

Get the **current branch** green on GitHub Actions. Work in a loop:
**read pipeline → reproduce locally → fix → verify → re-check CI**.

Read root `AGENTS.md` and `CLAUDE.md` before changing code — especially the code
style rules, the async conventions, and the exact lint/typecheck/test commands.

Do **not** weaken CI workflows or skip checks to get green. Do **not** fix
unrelated pre-existing failures unless merging `main` into the branch resolves
them.

## Phase 1 — Read the pipeline

### 1. Establish git context

Run in parallel when possible:

```bash
git branch --show-current
git status -sb
gh pr view --json url,headRefName,baseRefName,statusCheckRollup 2>/dev/null || true
```

- No open PR: use `gh run list --branch "$(git branch --show-current)" --limit 15`
- Open PR: prefer `statusCheckRollup` from `gh pr view` for a single summary

### 2. List failing checks

From `statusCheckRollup`, collect every item with `conclusion: FAILURE` or
`state: FAILURE`. Note:

- **Workflow name** (e.g. `@wandelbots/wandelbots-nova: Typecheck, Lint and Format`)
- **Job name** (e.g. `test`, `yamllint`, `validate-title`)
- **detailsUrl** (link to the run)

For each failure, fetch logs:

```bash
gh run view <run-id> --log-failed
```

If the URL is a job link, extract `run-id` from the path or use:

```bash
gh run list --branch "$(git branch --show-current)" --workflow "<workflow-file-name>" --limit 3
```

See `references/gh-commands.md` for more patterns.

### 3. Build a failure table (show the user)

Before editing code, summarize:

| Check | Job | Likely scope | Local repro command |
|-------|-----|--------------|---------------------|
| … | … | paths touched on branch | `uv run …` |

Map workflow → local command using `references/nova-ci.md`.

**Prioritize:** the fast `Typecheck, Lint and Format` gate first (format → import
order → lint → typecheck → unit tests), then integration/example jobs.

## Phase 2 — Reproduce locally

Run the **same command CI runs**. Make sure the environment matches first:

```bash
uv sync --extra "nova-rerun-bridge" --extra "wandelscript" --extra "novax"
```

Common repro commands (from `nova-dev.yaml`):

```bash
uv run ruff format --check .          # formatting gate
uv run ruff check --select I          # import order
uv run ruff check .                   # lint
uv run ty check                       # typecheck
PYTHONPATH=. LOG_LEVEL=WARNING uv run pytest -rs -v -m "not integration"   # unit tests
```

Single test file / test:

```bash
PYTHONPATH=. uv run pytest -rs -v path/to/test_file.py
PYTHONPATH=. uv run pytest -rs -v path/to/test_file.py::test_name
```

If local passes but CI fails: check **Python version** (`3.11` in the workflows),
missing extras in `uv sync`, `PYTHONPATH=.`, or a required env var
(`CELL_NAME`, `NOVA_API`, `NOVA_ACCESS_TOKEN`).

## Phase 3 — Fix (surgical)

### Failure patterns in this repo

| CI signal | Usual cause | Fix direction |
|-----------|-------------|---------------|
| `ruff format --check` fails | Unformatted code | `uv run ruff format .` |
| `ruff check --select I` fails | Import order | `uv run ruff check --select I --fix` |
| `ruff check .` fails | Lint violations | Fix the reported rule; avoid blanket `# noqa` |
| `ty check` errors | Type drift / bad annotation | Fix types (modern `list[T]` syntax); do not add `# type: ignore` to silence real issues |
| Unit test failure | Real bug or stale test | Fix code or test logic; keep tests async-correct (`await`, `async with`) |
| Integration test failure (`-m integration`) | Needs live NOVA instance | Usually CI-infra; only debug locally if you have `NOVA_API` + `NOVA_ACCESS_TOKEN` |
| Example run failed (`python examples/…`) | API/SDK change broke an example | Update the example to the current SDK, or fix the regression |
| PR title check | Title not Conventional Commits | Retitle PR to `chore\|feat\|fix[(scope)][!]: Description` |
| YAML Lint | `.github`/workflow YAML style | Fix per `.yamllint`; run `yamllint .` locally if installed |
| `uv audit` (SAST) | Vulnerable dependency | Bump the dependency in `pyproject.toml`; escalate if no fix available |

### Fix rules

1. **Match the failure** — only change what the failing check implies.
2. **Prefer local verify** before telling the user to push.
3. **Never disable a check** (no removing `ty` errors with ignores, no `--no-verify`,
   no deleting/relaxing tests) to force green without understanding the diff.
4. **Async**: everything in this SDK is async — keep `await` / `async with` correct
   when touching runtime code or tests.
5. **Escalate** when: failure is on `main` too, needs a running NOVA instance or
   secrets you don't have, or the fix would weaken a check.

## Phase 4 — Verify and loop

After fixes, re-run the exact failing command(s):

```bash
uv run ruff format --check . && uv run ruff check --select I && uv run ruff check . && uv run ty check
PYTHONPATH=. LOG_LEVEL=WARNING uv run pytest -rs -v -m "not integration"
```

Optional — re-query CI (user must push first for new runs):

```bash
gh pr view --json statusCheckRollup
gh run list --branch "$(git branch --show-current)" --limit 5
```

Tell the user:

- What failed and why (one sentence per check)
- What you changed
- What you ran locally
- Whether they need to **push** for CI to re-run
- Anything still red or outside scope

## Phase 5 — Multi-failure ordering

When several checks fail:

1. **Format** (`ruff format --check`) — cheapest
2. **Import order** (`ruff check --select I`)
3. **Lint** (`ruff check .`)
4. **Typecheck** (`ty check`)
5. **Unit tests** (`pytest -m "not integration"`)
6. **Integration tests / examples** (need a live NOVA instance; slow, often infra)

Fix one cluster at a time; re-run the local check before moving on.

## Reference files

- `references/gh-commands.md` — `gh` CLI for runs, logs, PR checks
- `references/nova-ci.md` — workflow → local command mapping and env notes

## Output format

End with a short status block:

```markdown
## CI status
- Branch: …
- PR: … (if any)

## Fixed
- [workflow/job]: cause → change → verified with …

## Still failing / needs you
- …

## Next step
Push and re-watch CI, or run: …
```
