# Notes moved from docs/migration.md

Material moved out of the customer guide [docs/migration.md](../../docs/migration.md):
measurements, ADR and contract references, and repository-internal notes (test patch targets, CI
paths). The text is kept as it was, with links fixed for this folder.

## Buffering and ease-in are pipeline modules: measurements

**A jogging script that relied on the default buffer now sends each live target alone:** measured
at a ~90 Hz loop, 0.45–0.84° from the target instead of 0.015°, with arm peaks up to twice the
commanded speed. Add `LiveBuffer(window_ms=500)` in front of the defaults to keep the old
behaviour; the joint and TCP jogging examples (`jogging_joints.py`, `jogging_tcp.py`) did until
`LiveBuffer` was removed on 2026-11-24 (ADR 0013); they use `ServoHorizon` now. For a
one-step policy in `PolicyExecutor` the buffer measured worse than the default lead path
([ADR 0009](../adr/0009-opt-in-pre-send-modules.md)).

- **Multi-step and TCP chunks are accepted.** The old buffered mode refused them; the module
  passes a multi-step chunk through unchanged (and clears its window, warning once), so it is
  placed by T1–T3 like any other chunk.

## The built-ins are pipeline modules

Smoothing, the bridge, the endpoint ramp, the joint clamp, hold padding and dropping late
waypoints are modules of the send pipeline now (ADR 0010).

## Sequential chunks end in a 300 ms hold: measurement

Before, a chunk of 15 steps or more at
30 Hz got no hold, and an arm still moving at its last step came to rest up to 0.5° past it.

## Session failures and invalid actions: ADR and contract references

- **`PolicyExecutor` connects the policy and runs `validate_schema` before the start move**
  (`start_joint_position`), not after it: a schema the policy refuses now fails before the robot
  moves (ADR 0014).
- **A failed IO write or computed action of a chunk without motion ends the run with its
  error** (contract T7), also on the run's last tick, and the sessions stop instead of draining.
- **Action values are checked when queued** (contract M12).

## Chunk modules run in a fixed order: measurement

Such a list used
to run, wrongly: with `[Bridge, TimestepTimeline]` a queue's steps ran up to 832 ms late.

## The package follows its architecture: internal notes

The modules moved into one folder per part of the [architecture](../architecture.md#where-things-live).

- **Patch targets follow them too**: tests that stub the transport patch
  `novapolicy.transport.lifecycle.ActionChunkSession` and
  `novapolicy.transport.lifecycle.EstopMonitor`.
- Type-checker and CI paths: `novapolicy/lerobot/**` is now `novapolicy/providers/lerobot/**`.

## A slimmer transport core: test patch targets

- Tests that stub the transport patch `novapolicy.transport.lifecycle.ActionChunkSession` and
  `novapolicy.transport.lifecycle.EstopMonitor` instead of the names in `novapolicy.executor` or
  `novapolicy.jogging.jogger`.

## The NATS policy provider is removed: architecture link

A policy service reached over NATS is a provider of its
own ([adding a provider](../architecture.md#adding-a-policy-provider);
`examples/http_policy.py` is one written that way in 26 lines).

## `Timestamps` has a setting: ADR reference

`Timestamps(grid_ms=...)` puts one-step joint targets on NOVA's cycle grid (opt-in, ADR 0021;
see [executor.md](../../docs/executor.md#one-step-joint-targets-on-the-cycle-grid-timestampsgrid_ms)).

## Live targets and jogger chunks land on their grid exactly: measurement and ADR reference

- **A `LiveBuffer` horizon ends at or before the newest target.** It used to reach up to one step
  past it, holding the newest value, and the next target rewrote that waypoint under the
  controller (up to 46 mm in a fast teleoperation run).
- **New, opt-in: `ServoHorizon`** follows live targets with a spring-damper and sends its braking
  path as a 400 ms horizon (ADR 0022, [jogging.md](../../docs/jogging.md#following-a-hand-servohorizon)).
