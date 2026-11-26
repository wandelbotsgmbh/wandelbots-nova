# novapolicy developer docs

For people working on novapolicy itself. How to use the package is in the
[README](../README.md) and the guides in [docs/](../docs/); this folder holds why the code is the
way it is, what it promises, and what was measured on the way.

## What is here

| | |
| --- | --- |
| [architecture.md](architecture.md) | How the package is built: mapping and transport, the send pipeline, execution modes, where each concern lives, what a provider costs |
| [contracts.md](contracts.md) | What every part promises (M for mapping, T for timing), each with the tests that hold it |
| [adr/](adr/README.md) | Decision records: context, decision, alternatives measured, consequences |
| [notes/](notes/) | Measurements, rationale, limitations and internals moved out of the customer docs, one file per guide |

The notes, by the guide they came from:

| Guide | Notes |
| --- | --- |
| [README](../README.md) | [notes/readme.md](notes/readme.md) |
| [docs/executor.md](../docs/executor.md) | [notes/executor.md](notes/executor.md) |
| [docs/jogging.md](../docs/jogging.md) | [notes/jogging.md](notes/jogging.md) |
| [docs/rerun.md](../docs/rerun.md) | [notes/rerun.md](notes/rerun.md) |
| [docs/migration.md](../docs/migration.md) | [notes/migration.md](notes/migration.md) |
| [docs/schema.md](../docs/schema.md) | [notes/schema.md](notes/schema.md) |
| [docs/adding-a-provider.md](../docs/adding-a-provider.md) | [notes/adding-a-provider.md](notes/adding-a-provider.md) |
| [docs/reinforcement-learning.md](../docs/reinforcement-learning.md) | [notes/reinforcement-learning.md](notes/reinforcement-learning.md) |
| [docs/gr00t.md](../docs/gr00t.md) | [notes/gr00t.md](notes/gr00t.md) |
| [providers/lerobot/README.md](../providers/lerobot/README.md) | [notes/lerobot.md](notes/lerobot.md) |
| [examples/](../examples/) | [notes/examples.md](notes/examples.md) |

## Where new material goes

- A change in how the package is used: the guide in `docs/` (or the README), with a verified
  snippet. No measurements, rationale, limitations or ADR and contract references there.
- A decision with alternatives or measurements behind it: a new record in `adr/`, listed in
  [adr/README.md](adr/README.md).
- A new promise the code keeps: a contract in [contracts.md](contracts.md), with its tests.
- A measurement or an internal detail that explains a guide: the matching file in `notes/`.

The rule is also in the repository's `AGENTS.md`, for coding agents.
