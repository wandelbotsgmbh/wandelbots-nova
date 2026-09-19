# PR title and body

Self-contained writing rules. The reviewer should not need the ticket, the chat,
or the diff to understand **what** changed, **why**, and **what is better** after
merge.

## Title

- One problem. One [Conventional Commits](https://www.conventionalcommits.org/)
  subject. Imperative, under 72 characters, no trailing period.
- `<type>(<scope>): <summary>` — `type` is `feat`, `fix`, `docs`, `chore`,
  `refactor`, `perf`, `test`, `build`, `ci`, or another honest type. Add `!`
  after type or scope for a breaking change.
- **Scope:** a ticket id when one is known (`feat(ABC-123): …`). Otherwise the
  area the change belongs to (`feat(web): …`). Omit scope if none is honest. Do
  not put a ticket in trailing parentheses after an area scope.
- The title is the what in one line. Do not stuff why into the title.
- The title describes the **branch**, not the last commit. If an existing
  conventional subject on the branch still covers the whole diff, reuse it
  verbatim.

## Length and tone

- Write for a reviewer who has not been in the conversation. Concise and
  complete.
- Small fix: ~50–100 words. Single feature: ~150–250. Multi-area: ~300–400.
  Breaking change or migration: as long as the migration note needs. Do not pad.
  Cut LLM-length drafts.
- Lead with what changed and why, then benefits, then only the how the diff will
  not say. Tradeoffs and limits go **near the top**, not in a footnote.
- No "this PR is about…". Start summary bullets with an active verb.
- Match the repo's tone. Fill a repo PR template when one exists rather than
  replacing it; still include every **Always include** section that the template
  lacks.

## Always include

| Section | Answers |
|---|---|
| **Summary** | What changed (1–3 bullets). Match language to the size of the change. |
| **Why** | The problem or motivation. Mandatory. One short paragraph. |
| **Benefits** | What is better for a user or operator after this lands. Mandatory. Not a restatement of the diff. |
| **How** | Approach and any unconventional choice. Do not narrate every file. |
| **Test plan** | Exact commands or click paths, environment, edge cases actually run. "Tested locally" is not a plan. |
| **UI evidence** | For user-facing work: pass/fail plus a screenshot or other proof. Omit if not user-facing. |

Also include when they apply (omit when they do not):

- **Breaking change / migration** — what callers must do, and whether it is
  reversible.
- **Notes for reviewers** — where to focus, open questions, follow-ups
  deliberately left out.
- **Issue link** — GitHub issues: `Closes #123` (or `Fixes`) only when this PR
  should close them. Other trackers (Linear, Jira, …): mention the id; do not
  invent a `Closes ABC-123` unless that tracker actually honors it.

## Template

Use this when the repo has no PR template. If it has one, fill that template and
merge in any missing rows from **Always include**.

```markdown
## Summary
- <primary change>
- <second area only if the PR really touches it>

## Why
<What was missing, broken, or postponed. One short paragraph.>

## Benefits
- <user or operator outcome>
- <second outcome if real>

## How
<Only what the diff will not tell: approach, tradeoff, leftover risk.>

## Test plan
- [ ] <command or click path>
- [ ] <edge case you actually checked>
```

Add **UI evidence** when the change is visible in a product UI.

## Ground in the diff

- Describe the branch from `git diff <default-branch>...HEAD` and the file list.
  Never fabricate changes, tests, screenshots, or verification you did not run.
- Call out new public APIs, config, and version bumps. Skip a file-by-file dump.
- If something is unconventional versus existing patterns, say so in **How** in
  one sentence.

## Common misses

- Why without benefits, or benefits that just repeat "we added X".
- Implementation dump before the problem.
- A second issue smuggled into the same PR (one problem per PR).
- Updating an existing PR by appending "also…" instead of rewriting the body so
  it still describes the whole branch.
- Titling after the last commit when the branch now does more (or less).
- Empty template headings left in. Delete unused sections.
