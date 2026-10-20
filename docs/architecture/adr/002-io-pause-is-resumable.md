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
4. Resume is automatic when the condition clears. Pausing is per motion group: the same
   condition may be attached to several groups to pause them all. `GroupArgs.pause_on_io`
   exposes it for synchronized multi-group execution.
5. The intended wiring is a **motion-enable signal**: the IO reads `True` while the robot is
   allowed to move and the robot pauses as soon as it reads `False`. This is fail-safe with
   respect to the signal path — a broken wire or a lost fieldbus connection reads `False`,
   never `True` — whereas a "pause while high" signal would silently stop working when the
   bus drops. `nova.cell.io_condition.motion_enable_signal(io, origin)` builds that condition
   (`PauseOnIO(io == False)`); a signal that is already low at the start keeps the robot at
   the start of its trajectory until it is raised (measured: `PAUSED_ON_IO` without motion).

## Consequences

- **Bus loss (measured 2026-09-04):** if the bus-IO *service* disappears while a motion is
  armed with a `BUS_IO` condition, the controller stops evaluating the condition and the robot
  keeps running; it reports the pause only once the bus is back. The SDK closes that gap for
  one-shot execution: `move_forward` watches the bus-IO state on the NATS subject
  `nova.v2.cells.{cell}.bus-ios.status` (the service publishes an empty state when it goes away
  and `CONNECTED` when it returns) and sends a `PauseMovementRequest` itself when the bus is not
  connected — the same on-path ramp as the controller pause — then resumes through the normal
  release path once the bus is back and the signal allows motion (`E10`: stopped ~1 s into the
  service removal, resumed ~60 ms after the enable signal returned). This holds only while the
  SDK process and its NATS connection are alive; the controller-side fix (evaluate an
  unreadable IO as "pause") is reported to the RAE/bus-IO team.
- **No polling of the API.** Controller IOs are observed on the `stream_io_values` websocket
  (the controller-IO REST endpoint returns 429 to a write while a read is in flight), bus IOs
  and the bus state over NATS with a single initial read after subscribing. When a source cannot
  be observed, `execute()` fails with `IOConditionUnavailable` instead of degrading to polling.

- `execute()` blocks through IO pauses and returns at the target; programs need no code to
  handle the pause itself.
- Cursor callers must check `OperationResult.paused_on_io`: `final_location` no longer implies
  the target was reached.
- The SDK depends on the controller not auto-resuming. If a future controller resumes on its
  own, the machine's observed-resume transition keeps the state consistent and the redundant
  start the SDK sends while running is a no-op retarget.
- Program-wide pausing (between motions) is a follow-up; `IOConditionWatcher` is the primitive
  to gate non-motion steps with the same signal.
