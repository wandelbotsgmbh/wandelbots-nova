# AGENTS.md

## Coding agent guidelines

Behavioral guidelines to reduce common mistakes when automated coding assistants edit this repository. Use together with the project-specific sections below.

**Tradeoff:** These guidelines bias toward caution over speed. For trivial tasks, use judgment.

### 1. Think before coding

**Do not assume. Do not hide confusion. Surface tradeoffs.**

Before implementing:

- State your assumptions explicitly. If uncertain, ask.
- If multiple interpretations exist, present them—do not pick silently.
- If a simpler approach exists, say so. Push back when warranted.
- If something is unclear, stop. Name what is confusing. Ask.

### 2. Simplicity first

**Minimum code that solves the problem. Nothing speculative.**

- No features beyond what was asked.
- No abstractions for single-use code.
- No flexibility or configurability that was not requested.
- No error handling for impossible scenarios.
- If you write 200 lines and it could be 50, rewrite it.

Ask: “Would a senior engineer say this is overcomplicated?” If yes, simplify.

### 3. Surgical changes

**Touch only what you must. Clean up only your own mess.**

When editing existing code:

- Do not “improve” adjacent code, comments, or formatting.
- Do not refactor things that are not broken.
- Match existing style, even if you would do it differently.
- If you notice unrelated dead code, mention it—do not delete it unless asked.

When your changes create orphans:

- Remove imports, variables, and functions that **your** changes made unused.
- Do not remove pre-existing dead code unless asked.

**Test:** Every changed line should trace directly to the user’s request.

### 4. Goal-driven execution

**Define success criteria. Loop until verified.**

Turn tasks into verifiable goals:

- “Add validation” → write tests for invalid inputs, then make them pass.
- “Fix the bug” → reproduce with a test (or clear steps), then make it pass.
- “Refactor X” → ensure tests pass before and after.

For multi-step tasks, state a brief plan:

```text
1. [Step] → verify: [check]
2. [Step] → verify: [check]
3. [Step] → verify: [check]
```

Strong success criteria allow independent iteration. Weak criteria (“make it work”) invite endless clarification.

---

## NOVA API (v2)

The SDK targets the **released** NOVA API v2. Treat the installed `wandelbots_api_client`
(`nova.api` re-exports `wandelbots_api_client.v2_pydantic`) as the source of truth — it matches
the released API, whereas an internal/nightly spec may contain fields that are not released yet.

**For anything involving the NOVA API — model field names, motion planning requests
(`plan_trajectory`, `MotionGroupSetup`, `LimitSet`, `LimitsOverride`), robot limits, or
torque/velocity/acceleration/jerk limits — use the `nova-api-v2` skill**
(`.agents/skills/nova-api-v2/SKILL.md`). It documents how to verify fields against the installed
client, the planning data flow, and the key limits/torque rules (e.g. `torque` is a
motion-group-global per-joint limit with no per-segment override, and controller-reported torque
is already forwarded to the planner via `global_limits`).
