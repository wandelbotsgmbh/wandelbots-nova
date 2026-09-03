# Architecture Decision Records

An ADR records **a decision and why we made it over the alternatives** — not how
the code works (that lives in the code, its docstrings, and `docs/`). Lead with
the decision; foreground the rejected options and their rationale. Keep it lean.

Once **Accepted**, an ADR is immutable: don't edit it as the design evolves —
write a new ADR that **supersedes** it.

## Format

Michael Nygard's ADR shape (Status / Context / Decision / Consequences), leaned
decision-first with an explicit **Alternatives considered** section and a
Y-statement. Informed by **MADR 4.x** (<https://adr.github.io/madr/>) and the
**Semantic-Anchors** ADR guidance
(<https://github.com/LLM-Coding/Semantic-Anchors>). The same convention is used
by `wandelbotsgmbh/trajectory-format`, so records read the same across repos.

Use a Pugh-style scored table inside *Alternatives considered* only when a
decision genuinely turns on weighing several criteria; most turn on a single
decisive force.

## Writing a new ADR

1. Copy [`TEMPLATE.md`](./TEMPLATE.md) to `NNN-short-title.md` (next number).
2. Fill in **Decision → Forces → Alternatives considered → Consequences → References**.
3. Open it as `Status: Proposed`; flip to `Accepted` on sign-off. An ADR written
   after the fact for something already shipped may open as `Accepted`, citing
   the PR that implemented it under *References*.
4. To change an accepted decision, write a new ADR with `Supersedes: ADR-NNN`;
   leave the old one intact.

## Index

Status legend: 🕒 Proposed · ✅ Accepted · ⚠ Superseded / deprecated

### Execution

- [001](./001-merge-movement-controllers-into-trajectory-cursor.md) — `move_forward` becomes a `TrajectoryCursor` adapter ✅
