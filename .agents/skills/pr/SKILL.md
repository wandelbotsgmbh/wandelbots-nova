---
name: pr
description: >-
  Open or update one GitHub pull request for the current work: a
  conventional-commits title and a description that always includes why and
  benefits, and never a second PR for the same head. Use when the user asks to
  open a PR, create a pull request, update or refresh the PR description, ship
  the branch, or "make a PR". On the default branch, create a new branch first.
  If this head already has a PR, edit it — do not create another. Do not open a
  PR unless the user asked.
---

# Open / update a pull request

This folder is the whole skill — enough to open a PR in any repo. Do not require
the repo's `AGENTS.md` to operate. If you already know repo extras
(ticket-as-scope, extra body sections, branch pattern, a PR template), apply them
**on top** of this skill — never instead of Why and Benefits.

Read [references/description.md](references/description.md) before writing title
or body.

Drive GitHub through the local authenticated `gh` CLI. Prefix `gh` with
`GH_PAGER=cat`. Never merge. Never force-push unless the user asked. Always end
the reply with the PR URL on its own last line.

## When to open (and when not to)

Only run this skill when the user asked to open, create, update, or refresh a PR.

| Situation | Action |
|---|---|
| User did **not** ask to open or update a PR | Do nothing. Do not create "while you're here". |
| Default branch, clean tree, no unique commits | Stop. Do not open an empty PR. |
| Default branch, has work to ship | **Always** create a new branch first (step 1). Never commit or push on the default branch. |
| Feature branch, unique commits, **no** PR for this head | Push (if needed) and **create** one PR (step 3). |
| Feature branch, a PR **already exists for this head** | **Update** that PR (step 4). Never `gh pr create`. |
| `gh pr view` / list shows a PR whose `headRefName` ≠ current branch | Stop. Do not edit the wrong PR. |
| Work is unfinished / user said WIP | Open or convert to **draft**. Title may use a `[WIP]` prefix. |
| Diff contains secrets, credentials, or `.env` files | Stop and tell the user. Do not open. |
| Two unrelated problems on the same branch | Flag the mixed scope. Do not silently describe them as one change. Do not split the branch unless asked. |

One head → one PR. Updating means **rewriting** title and body for the whole
branch, not appending "also…".

## 0. Context (always, in parallel)

```bash
git status -sb
git branch --show-current
git log --oneline -8
git diff && git diff --cached
git rev-parse --abbrev-ref --symbolic-full-name @{u} 2>/dev/null || true
GH_PAGER=cat gh repo view --json defaultBranchRef --jq '.defaultBranchRef.name'
command -v gh && GH_PAGER=cat gh auth status
```

Default branch: the `gh` value, else `main`, else `master`.

If `gh` is missing or unauthenticated: still author title + body to a temp file,
then give the user the exact `git push` and `gh pr create --body-file` commands.
Do not invent a web-API fallback.

Decide:

- `type`: `feat` | `fix` | `docs` | `chore` | `refactor` | `perf` | `test` |
  `build` | `ci` (or another [Conventional Commits](https://www.conventionalcommits.org/)
  type that fits). Use `!` for a breaking change.
- `scope`: a ticket id if the user, branch, or commits have one (`ABC-123`);
  otherwise the area the diff belongs to. Omit `(scope)` when none is honest.
- `summary`: short imperative phrase (the what, not the why).

Title, under 72 characters:

```text
<type>(<scope>): <imperative summary>
```

If the branch already has a conventional subject that still describes the
**whole** branch, reuse it. Do not title the PR after the last commit alone.

## 1. Branch

### On the default branch

Create a new branch before any commit, push, or `gh pr`:

```bash
git checkout -b <type>/<scope>-<slug>
```

If the repo documents a branch pattern, use that instead. `<slug>` is 2–4
lowercase words from the change. Uncommitted work comes along.

If the work is not yet committed and the user asked for a PR, commit it on the
**new** branch (conventional subject; body = why). Commit only files that belong
to this change; ask if the dirty tree is mixed.

### Already on a feature branch

Stay on it. Do not fork another branch. Do not create a second PR.

## 2. Existing PR?

```bash
GH_PAGER=cat gh pr list --head "$(git branch --show-current)" --json number,title,url,body,headRefName
```

- Empty → create (step 3).
- One row, `headRefName` matches current branch → update (step 4).
- Anything else → stop.

## 3. Create (no PR yet)

```bash
git push -u origin HEAD
```

Draft the body from [references/description.md](references/description.md). Write
it to a temp markdown file (avoids shell escaping). If
`.github/PULL_REQUEST_TEMPLATE*` (or `docs/pull_request_template.md`) exists,
fill **that** structure and still include Why and Benefits.

```bash
GH_PAGER=cat gh pr create --base "<default-branch>" --head "$(git branch --show-current)" \
  --title "<type>(<scope>): <imperative summary>" \
  --body-file <tmp.md>
```

Add `--draft` when the work is unfinished. Delete the temp file after. If a hook
or CI gate blocks create, fix the reported failure and retry. Do not skip hooks.

## 4. Update (PR already exists)

Re-read `git log <default-branch>...HEAD` and the full diff since the branch
diverged — not just the latest commit. If the branch has unpushed commits,
`git push` first (no `--force` unless asked). Rewrite **title and body** so they
describe the branch as it is now.

```bash
GH_PAGER=cat gh pr edit <number> --title "<conventional title>" --body-file <tmp.md>
```

Only pass `--title` when the current title is wrong. Do not append "also
updated…". Replace the description.

## 5. Report

Say whether the PR was **created** or **updated**, plus one line on what changed.
Wait for review. Do not merge, do not request reviewers, do not comment, unless
the user asked.

**Always end the reply with the PR URL on its own last line.** No extra sentence
after it.

```bash
GH_PAGER=cat gh pr view --json url --jq '.url'
```
