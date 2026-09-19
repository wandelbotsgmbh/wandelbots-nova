# ADR NNN: <the decision, as an outcome — "Do X", not "The X system">

**Status**: Proposed | Accepted | Rejected | Deprecated | Superseded by ADR-NNN
**Date**: YYYY-MM-DD
**Authors**:
**Supersedes**: ADR-NNN  *(omit if none)*

<!--
An ADR records a DECISION and its rationale at a point in time — not how the
system works (that lives in the code and docs/). Lead with the decision.
Foreground the alternatives you rejected and why; that is the part the code
never captures and the part that stops a decision being re-litigated. Keep it
lean. Do NOT update it as the design evolves — supersede it with a new ADR.

Format: Michael Nygard's ADR (Status / Context / Decision / Consequences),
leaned toward decision-first with an explicit alternatives section and a
Y-statement. Informed by MADR 4.x (adr.github.io/madr) and the Semantic-Anchors
ADR/Pugh guidance. Use a Pugh-style scored table inside "Alternatives
considered" only when a decision genuinely turns on weighing several criteria.
-->

## Decision

*State the call plainly, first — "We will …" — the what, not the how. One or
two short paragraphs. A one-line Y-statement often crystallises it: "In the
context of `<use case>`, facing `<concern>`, we chose `<option>`, to achieve
`<quality>`, accepting `<downside>`."*

## Forces

*Only the things that made this a genuine question — constraints, prior pain,
requirements, what was learned from the surfaces involved. Terse. Not a system
overview.*

## Alternatives considered

*The heart of the record: each option that was on the table and the one-line
reason it lost, ending with the chosen one. If a choice was validated against
something real — hardware, a baseline schema, an existing consumer — say so.*

- **`<option>`** — `<why rejected>`
- **`<option>`** — `<why rejected>`
- **`<chosen>`** — `<why it wins>`

## Consequences

*What becomes true as a result — both the wins and the costs/risks knowingly
accepted, plus anything deliberately deferred to a follow-up.*

## References

*PRs, issues, superseded/related ADRs, prior art, the code or hardware surface
that grounded the decision. One line each.*
