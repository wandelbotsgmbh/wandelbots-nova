<!--
Title must follow Conventional Commits and is validated by CI:
  chore|feat|fix[(scope)][!]: Description
It feeds release-please and the changelog, so write it for the whole branch.

Fill in Summary, Why, Benefits and Test plan. Delete sections that do not apply.
Contribution guide: https://github.com/wandelbotsgmbh/wandelbots-nova#how-to-contribute
-->

## Summary

<!-- What changed, in 1-3 bullets. Start each with an active verb. -->

-

## Why

<!-- Required. The problem, gap or motivation. One short paragraph. -->

## Benefits

<!-- Required. What is better for a user, operator or maintainer after this lands.
     Not a restatement of the diff. -->

-

## How

<!-- Only what the diff does not show: approach, tradeoffs, leftover risk.
     Delete for trivial changes. -->

## Test plan

<!-- Exact commands or click paths you actually ran. "Tested locally" is not a plan. -->

- [ ] `uv run ruff format --check . && uv run ruff check --select I && uv run ruff check .`
- [ ] `uv run ty check`
- [ ] `PYTHONPATH=. LOG_LEVEL=WARNING uv run pytest -rs -v -m "not integration"`
- [ ] Docs updated (`README.md`, `docs/programs.md`, examples) if behavior changed
- [ ]

## Breaking change / migration

<!-- Delete if none. What must callers do, and is it reversible? -->

## Notes for reviewers

<!-- Delete if none. Where to focus, open questions, follow-ups left out on purpose. -->
