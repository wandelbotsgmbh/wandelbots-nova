# ADR 002 — A controller-side IO pause is a suspended execution, not completion

Date: 2026-09-03. Status: accepted.

## Context

`StartMovementRequest.pause_on_io` lets the controller pause a trajectory on path while an IO
condition holds (controller IO or bus IO / Profinet). The state stream reports this as
`TrajectoryDetails.state = PAUSED_ON_IO`, level-based, for as long as the pause holds.

Until this decision the SDK's execution state machine handled `PAUSED_ON_IO` in the same
branch as `END_OF_TRAJECTORY`. Every `execute()` / `plan_and_execute()` armed with
`pause_on_io` therefore *returned normally mid-trajectory* the moment the signal was set, the
execution websocket was closed, and nothing could resume the motion. `TrajectoryCursor.forward()`
resolved as a successful traversal at the pause location.

Measured on NOVA 26.7.0 on 2026-09-03 with virtual KUKA KR6 / UR10e controllers and a virtual
Profinet bus-IO service (full method and numbers in the local research note
`docs/architecture/incoming/pause-on-signal-evaluation.md`, a gitignored directory):

- the controller never resumes an IO pause by itself, not even after the condition clears;
- a new `StartMovementRequest` on the same websocket resumes it within ~100 ms, re-arms the
  pause and re-attaches the IO overlay; a start while the condition still holds is accepted
  but leaves the robot paused;
- with the condition already true at the start the controller reports `PAUSED_ON_IO` at the
  start location without ever running; between init and the first start it reports the
  parked `PAUSED_BY_USER`;
- after a resume the controller keeps re-publishing `PAUSED_ON_IO` for a few control cycles
  before `RUNNING` appears;
- the deceleration profile equals the one of a `PauseMovementRequest`; only the detection
  point differs (controller loop vs. SDK polling).

## Decision

1. `TrajectoryPausedOnIO` moves the execution machine to `pausing`/`paused`, like
   `TrajectoryPausedByUser`; the machine records `pause_reason` (`USER` | `IO`). A `RUNNING`
   frame observed in `paused` is an observed resume and returns to `executing`.
2. The cursor completes a *commanded* movement operation on an IO pause with
   `OperationResult.paused_on_io = True` (`final_location` = pause location) — including one
   that never moved, since there is no parked look-alike for `PAUSED_ON_IO` — and stays
   attached; `detach_on_standstill` fires on `ended` only. An operation started while the
   machine still holds a terminal state ignores frames that re-publish that state until a
   frame with another state has been seen: `PAUSED_ON_IO` after an IO pause, and
   `END_OF_TRAJECTORY` after an intermediate `forward_to` stop when the movement has room to
   move (the controller reports every commanded stop as END and re-publishes it level-based;
   without the guard the next `forward()` of a multi-group session resolved at the
   intermediate location — a pre-existing defect the same rule fixes).
3. `move_forward` (the `execute()` path) drives the execution to the end through pauses:
   when the operation resolves `paused_on_io`, it awaits
   `MovementControllerContext.wait_for_pause_on_io_release` and calls `forward()` again.
   `MotionGroup._execute` provides that waiter from `nova.cell.io_condition.IOConditionWatcher`,
   which polls the IO (both origins, tolerant of the controller-IO endpoint's transient 429)
   and returns once the condition no longer holds. A context without a waiter keeps the old
   early-return behaviour, with a warning.
4. Resume is automatic when the signal clears (the PLC "hold" contract). Pausing is per
   motion group: the same condition may be attached to several groups to pause them all.
   `GroupArgs.pause_on_io` exposes it for synchronized multi-group execution.

## Consequences

- `execute()` blocks through IO pauses and returns at the target; programs need no code to
  handle the pause itself.
- Cursor callers must check `OperationResult.paused_on_io`: `final_location` no longer implies
  the target was reached.
- The SDK depends on the controller not auto-resuming. If a future controller resumes on its
  own, the machine's observed-resume transition keeps the state consistent and the redundant
  start the SDK sends while running is a no-op retarget.
- Program-wide pausing (between motions) is a follow-up; `IOConditionWatcher` is the primitive
  to gate non-motion steps with the same signal.
