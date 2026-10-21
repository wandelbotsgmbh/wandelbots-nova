# ADR 002: Execute command routines with an SDK-native `CommandRoutineCursor`

**Status**: Proposed
**Date**: 2026-10-13
**Authors**:
**Supersedes**: ADR-001 (in part: single protocol owner, layering rule)

## Decision

We will execute NOVA `CommandRoutine`s entirely in the SDK through a new
`CommandRoutineCursor` in `nova.command_routines`, built from the ground up for
routines. It keeps the concepts of `TrajectoryCursor` (intent mailbox, operation
futures, trajectory state machine, `cntrl` as the `executeTrajectory` client) and
reuses its module-level building blocks (`Intent`, `OperationHandler` and operation
types, `init_movement_gen`, `TrajectoryExecutionMachine`), but owns its own protocol
loop. It neither wraps nor changes `TrajectoryCursor`, which stays the cursor for
action lists until actions are retired.

A routine is compiled directly from its API commands into planner requests
(`MotionCommand`, `PlanCollisionFreeRequest`), never through `nova.actions`. Planning
yields a `PlannedCommandRoutine`: the joint trajectory plus a precomputed schedule of
**stops** (robot stands still; the cursor runs that point's commands in routine
order), **overlay outputs** (`set_outputs`), **markers** and the **pause-on-IO
activation**. The cursor interprets that schedule natively against an **execution
position** (location plus the commands already run there), which makes forward and
backward travel and resuming from any command well-defined.

> In the context of running dataset command routines from programs and from the
> routine service that is built on this SDK, facing a released API with no routine
> execution endpoint and an action model slated for deprecation, we chose a
> routine-native `CommandRoutineCursor` that reuses `TrajectoryCursor`'s building
> blocks but owns its protocol loop, to achieve exact `at` semantics, lossless
> routine events, bidirectional interactive control and resume-from-command,
> accepting a second `executeTrajectory` loop until `TrajectoryCursor` retires.

Commitments that define the engine:

- **Layers.** Pure `compile` (routine → planning segments and positioned commands)
  and `schedule` (triggers → locations, execution-position logic); async
  `plan_command_routine`; the cursor with `attach_command_routine` /
  `execute_command_routine` / `stream_execute_command_routine`.
  `ctx.dataset.command_routines[...]` offers `plan` / `attach` / `execute` /
  `plan_and_execute` shortcuts.
- **Location contract.** Motion *i* spans `[i, i+1]`; triggers resolve against the
  *preceding* motion, as the API defines; collision-free segments are rescaled to one
  location unit.
- **Stops.** Before the first motion, at the end, and after each motion followed by
  an untriggered wait; planning splits there, so the robot stands still exactly on the
  target. A wait with `at` stops on the path at its resolved location.
- **IO.** `wait_for_io` is evaluated client-side at stops (composite expressions,
  timeouts). `pause_on_io` uses the server's single-condition `pause_on_io`; the cursor
  tells an IO pause from a user pause and resumes it with the inverted condition. It
  activates exactly at a stop, otherwise at the preceding start.
- **Waits.** `wait_for_time` is a client-side sleep at a stop; the trajectory carries
  motion only.
- **Fidelity.** A command's `motion_settings`, else the routine default; neither means
  no override. Live motion group setup; start joints from the routine unless
  overridden. Unsupported motions raise before anything moves.
- **Forward travel.** `forward`, `forward_to_next_command`, `forward_to_command(k)`.
  Each start carries only the outputs still ahead of the execution position; stop
  commands run in order; markers and command events are emitted losslessly.
- **Backward travel is motion only.** `backward`, `backward_to_previous_command`,
  `backward_to_command(k)`: starts carry no outputs; no commands, markers or command
  events run; stops only halt the robot when targeted; the active pause monitor stays
  armed. The next forward travel re-runs everything ahead of the new position.
- **Resume.** `attach_command_routine(..., start_at_command=k)` initializes the
  movement at command k's location. The robot must already be at the trajectory's
  joints there (else it raises with the expected joints), or opt into
  `approach="collision_free"` to plan and run a collision-free approach first. Outputs
  from skipped commands are not replayed; pause monitors from skipped commands are
  active.
- **Failures.** A failed client-side command fails its operation but keeps the session
  (retry with `forward()`); controller movement errors end it.
- **Service boundary.** Only the planning, kinematics, trajectory caching/execution,
  state stream and controller/bus IO APIs, with IO behind an injected
  `CommandRoutineIO`. No routine service, `ProgramContext`, NATS or global signals.

## Forces

- **No server-side routine execution.** The released API v2
  (`wandelbots_api_client==26.7.0`) only persists routines in datasets; planning takes
  `MotionCommand`s, execution takes trajectories.
- **The routine service is built on the SDK.** The SDK cannot delegate to it; it must
  provide an engine the service can embed, including restart from a command.
- **Actions will be deprecated.** `TrajectoryCursor` is shaped by them (action
  metadata, action-typed motion events, motion-count validation, action stepping).
  Compiling routines to actions would also lose settings fidelity (SDK
  `MotionSettings` defaults to a 50 mm/s TCP limit; the API means "no override").
- **Wrapping `TrajectoryCursor` falls short.** No lossless crossing hook (its state
  iterator is bounded and drops), no per-start hook to shape outputs by direction or
  position, and `PAUSED_ON_IO` completes an operation exactly like `ended`.
- **Trigger anchors disagree.** API `AtTrigger` anchors to the preceding motion;
  `WriteAction.at` anchors to the following one.
- **Overlay outputs are direction- and history-blind.** The server sets them on
  backward travel too, re-fires an output at the resume location (ADR 001), and fires
  outputs that share a boundary with a wait before the wait.
- **Server IO primitives are narrow.** `StartOnIO` / `PauseOnIO` take one IO and
  comparator, no negation, composition or timeout, and only change on a start.

## Alternatives considered

- **Compile routines into action lists and run `MotionGroup.plan` / `execute`** —
  ties the core to the deprecated model, needs trigger re-anchoring and epsilon tricks,
  loses settings fidelity, cannot stop mid-path or resume by command. Rejected.
- **A `CommandRoutineCursor` that wraps `TrajectoryCursor`** — keeps one loop, but
  routine semantics stay outside the protocol: lossy markers, state sniffing for IO
  pauses, no per-start overlay for direction or resume, and the future core is bound to
  an action-era class. Rejected.
- **Teach `TrajectoryCursor` routines** — grows an action-era class and widens the
  blast radius of every `execute()`. Rejected.
- **Extract a shared protocol core from `TrajectoryCursor` first** — keeps one loop,
  but refactors the protocol path of every `execute()` now, for a class that retires
  with actions. Rejected.
- **Delegate to a routine service** — it is built on the SDK; circular. Rejected.
- **Routine-native `CommandRoutineCursor` reusing `TrajectoryCursor`'s building blocks
  with its own loop** — routine semantics, direction and resume live where the protocol
  is driven; the existing cursor and its suites stay untouched; the duplicate loop
  disappears when actions retire. Chosen.

## Consequences

- **Two `executeTrajectory` loops coexist** (`TrajectoryCursor`,
  `CommandRoutineCursor`). This knowingly reopens ADR 001's fork risk. Mitigations:
  the state machine, `Intent`, operation lifecycle and initialize handshake are
  single-sourced; ADR 001's protocol contract (overlay on every start, stop beats a
  pending intent, first-dispatch gate, acknowledgements, terminal states) is tested
  against both cursors; each loop points to its twin; protocol fixes land in both
  until `TrajectoryCursor` retires.
- **`TrajectoryCursor` and `MotionGroup.execute` are unaffected.**
- **One routine engine, in the SDK.** Programs and the routine service run the same
  code; `PlannedCommandRoutine` is serializable, so the service can cache plans and
  resume after a restart from a stored command index.
- **`at` is exact.** Outputs and markers land inside their preceding motion; waits
  with `at` stop on the path; routine events are lossless because the cursor's own
  monitor sees every state.
- **Outputs fire exactly once going forward and never going backward** — for routines
  this improves on ADR 001's accepted re-fire at the resume location.
- **Routine order is exact at stops**, at the cost of standstill: an untriggered wait
  ends blending at its boundary.
- **Resuming does not restore IO state.** Outputs from skipped commands are left as
  they are; the caller owns any state the skipped part would have set.
- **A collision-free approach can fail to find a path**; the resume then raises before
  the routine moves.
- **IO waits add client latency** (polling plus a round trip) versus server-side
  `start_on_io` — accepted for composition and timeouts; revisit with
  `stream_io_values` if needed.
- **Pause monitoring can start earlier than authored** when its command is not at a
  stop — conservative and logged.
- **Trigger constructors and resolution math move to `nova/utils/path_triggers.py`**;
  `nova.actions` stops importing `nova.command_routines` (removes an import cycle).
- **Assumption pinned by an integration test:** `forward_to` stops a blended path at
  the requested location in both directions.
- **Deferred:** splines and direction-constrained Cartesian PTP, cross-dataset pose
  references, multi-motion-group routines, moving action lists onto this engine and
  retiring `TrajectoryCursor`'s loop.

## References

- [ADR 001](./001-merge-movement-controllers-into-trajectory-cursor.md) — single
  protocol owner and layering rule superseded here in part; its protocol contract
  reused as tests.
- `nova/cell/movement_controller/trajectory_cursor.py` — source of the reused building
  blocks, unchanged.
- `nova/cell/movement_controller/trajectory_state_machine.py` — the shared state
  machine.
- `nova/cell/trajectory_executor.py` — the `attach` session pattern.
- `nova/command_routines/` — routine builders (existing) and the engine.
- `nova/actions/path_trigger.py`, `nova/actions/path_trigger_resolver.py` — the
  trigger vocabulary and math being shared.
- `wandelbots_api_client==26.7.0` — `CommandRoutine`, `PlanTrajectoryRequest`,
  `InitializeMovementRequest`, `StartMovementRequest`.
- PR [#488](https://github.com/wandelbotsgmbh/wandelbots-nova/issues/488) (NDX-844) —
  command routine builders.
