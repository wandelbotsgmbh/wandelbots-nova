# ADR 001: `move_forward` becomes a `TrajectoryCursor` adapter

**Status**: Accepted
**Date**: 2026-08-18
**Authors**:
**Supersedes**: —

## Decision

We will have **one implementation of the `executeTrajectory` protocol**, owned by
`TrajectoryCursor`. `move_forward` stays the default `MovementController` and
keeps its name and signature, but becomes a thin adapter: it configures a cursor
for one-shot execution, calls `forward()`, and returns `cursor.cntrl`.

`MovementController` / `MovementControllerContext` remain the public plug-in
seam, so no caller of `execute()` or `plan_and_execute()` changes.

> In the context of two movement controllers implementing the same
> `executeTrajectory` protocol, facing a loop that had forked five ways with bug
> fixes landing on one fork only, we chose to reduce `move_forward` to a thin
> adapter over `TrajectoryCursor`, to achieve a single protocol implementation
> behind an unchanged plug-in seam, accepting that the default execution path
> now runs substantially more code than the 172-line original.

## Forces

- **The loop had forked five ways.** `main`'s `move_forward`, `TrajectoryCursor`,
  and three in-flight branches (`feat/path-triggers-distance-offset`,
  `feat/motion-guards`, `fix/trajectory-completion-standstill-watchdog`) each
  carried their own copy of *initialize → monitor → start → consume responses →
  stop at standstill*.
- **A feature branch had already rewritten `move_forward` into a worse cursor.**
  `feat/path-triggers-distance-offset` added a command queue, a response
  consumer, and pause/resume callbacks — structurally `Intent` + `_request_loop`
  + `_response_consumer`, minus the operation futures, the state-transition
  safety, and the tests.
- **Bug fixes were diverging.** The stall watchdog fixed a dropped-terminal-event
  hang in `move_forward` only; the cursor ran the same loop with the same
  exposure, unfixed.
- **They were already equivalent.** Compared message by message,
  `cursor.forward()` from location 0 with no target *is* `move_forward`;
  everything else the cursor offers is a superset.
- **A silent data-loss path blocked downstream work.** `Intent.to_commands()`
  built `StartMovementRequest` without `set_outputs`, so a routine's `io_write`
  actions were dropped whenever it executed through a cursor — the single most
  valuable overlay kind, and a prerequisite for the CommandRoutine effort.

## Alternatives considered

- **Keep both and sync fixes by hand** — the status quo that produced five forks
  and a watchdog fix on one side only. Rejected.
- **Delete `move_forward`; make callers drive a cursor** — breaks every caller of
  `execute()` / `plan_and_execute()`. The seam is precisely what makes the
  migration invisible. Rejected.
- **Fold the cursor into `move_forward`** — inverts the containment: the cursor
  is the superset (bidirectional travel, targets, pause/resume, playback speed).
  Rejected.
- **Move the adapter into `MotionGroup._execute`** — dissolves
  `MovementController` as a plug-in seam and hard-codes one execution policy.
  Rejected.
- **Keep `move_forward` as the default controller, reimplemented as a thin
  adapter over the cursor** — one protocol implementation, unchanged public
  surface, and every future feature has exactly one place to land. Chosen.

## Consequences

- **Features stop forking the loop.** Path triggers become `set_outputs` payload;
  async actions and guards become consumers of the cursor's existing
  `pause()` / `forward()` and state stream; the stall watchdog gets fixed once.
- **`set_outputs` is a cursor-level default, not a per-call argument.** Confirmed
  on hardware that the controller treats each `StartMovementRequest` as an
  *override* of the attached overlay: a resume that omits `set_outputs` clears
  the remaining outputs, and an output at exactly the resume location fires
  again. Every start must therefore carry the full list. Boundary re-firing is
  accepted rather than worked around — harmless for level-triggered outputs, not
  for edge-sensitive consumers downstream.
- **Starting immediately is adapter policy, not a cursor capability.** No
  `autostart` flag; the adapter calls `cursor.forward()` itself. This keeps
  "run to completion" out of an object whose purpose is interactive control.
- **A stop always wins over a queued or partially dispatched command.** A pending
  intent is deliberately *dropped* when the cursor stops; flushing it was
  implemented and reverted because it let an explicit `detach()` cancel the
  caller's future and then still command the robot to move.
- **Two latent cursor defects were surfaced and fixed** while establishing
  parity: the cursor reported a successful traversal for movement it never
  commanded, and a stop landing between two commands of one intent still sent the
  movement command. Both predated this work and affected interactive users.
- **The blast radius of a cursor bug is now every `execute()` call.** Mitigated by
  an equivalence test suite and by landing the adapter as its own revertible
  change, but knowingly accepted.
- **Layering rule, enforced from here on:** the cursor carries *server-side*
  overlay (`set_outputs`, `start_on_io`, `pause_on_io`) as **opaque payload** it
  never interprets, and must not import `WriteAction`, `to_set_io()` or path
  triggers. *Client-side* overlay — markers, async actions, guards, motion
  events — belongs in a layer **above** the cursor, driving it through its public
  API. Deferred: building that layer, and moving motion-event emission out of the
  cursor into it.

## References

- PR [#472](https://github.com/wandelbotsgmbh/wandelbots-nova/pull/472) — cursor
  parity: IO overlay on every start, false-success completion, stop-vs-dispatch.
- PR [#475](https://github.com/wandelbotsgmbh/wandelbots-nova/pull/475) —
  `move_forward` reduced to the adapter.
- `nova/cell/movement_controller/move_forward.py` — the adapter.
- `nova/cell/movement_controller/trajectory_cursor.py` — the protocol owner.
- `tests/cell/test_trajectory_cursor_parity.py` — the regression suite pinning
  the overlay contract and both defects.
- Hardware verification of `set_outputs` override + boundary re-fire: virtual
  KUKA `kuka-kr6_r700_sixx`, 2026-08-05.
