# GitHub CLI — CI inspection

Requires `gh` authenticated (`gh auth status`). Run from repo root.

## Current branch PR summary

```bash
git branch --show-current
gh pr view --json number,url,headRefName,baseRefName,statusCheckRollup,mergeable,mergeStateStatus
```

`statusCheckRollup` entries:

- `CheckRun`: GitHub Actions (`conclusion`, `name`, `detailsUrl`, `workflowName`)
- `StatusContext`: External statuses (`state`, `context`, `targetUrl`)

## No PR yet

```bash
gh run list --branch "$(git branch --show-current)" --limit 20
```

## Failed job logs

From the run list, copy the run ID:

```bash
gh run view 27340993761 --log-failed
```

Truncated output — pipe to a file for large logs:

```bash
gh run view <id> --log-failed 2>&1 | tail -100
```

## Filter by workflow file

```bash
gh run list --branch feat/my-branch --workflow nova-dev.yaml --limit 5
gh run list --branch feat/my-branch --workflow nova-run-examples.yaml --limit 3
gh run list --branch feat/my-branch --workflow yamllint.yaml --limit 3
gh run list --branch feat/my-branch --workflow uv-audit.yaml --limit 3
```

## Watch after push

```bash
gh run watch
gh run watch <run-id>
```

## Compare with base branch CI

If failures look unrelated to your changes:

```bash
git fetch origin main
git merge origin/main   # or rebase — user preference
```

Then re-run local checks before pushing.
