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

## Addendum (2026-10-01): selectable resume strategy

robotics/wbr!2384 with service-manager !3081 adds `PauseOnIO.auto_resume`: the controller
resumes an IO pause by itself once the condition clears. Neither MR is merged and no instance
runs it yet, and the earlier objection to self-restarting motion (RB-3908) is still open. So
the resume is a selectable strategy on `ExecutionPolicy`, and the default stays the SDK resume
until the two have been compared on a cell:

- `pause_resume=sdk` (default, `NOVA_PAUSE_RESUME=sdk`): this ADR as written. The SDK watches
  the signal and sends the resume start, with supervision of ignored starts.
- `pause_resume=controller`: every start carries `auto_resume=True`. Per wbr!2384 the
  controller then brakes on path while still reporting `RUNNING`, holds the robot as
  `WAIT_FOR_IO` (not `PAUSED_ON_IO`), and resumes with `RUNNING`. The machine reads
  `WAIT_FOR_IO` after motion as an IO pause (`paused`, reason IO), and the existing
  `paused(IO) ∧ RUNNING → executing` edge follows the resume. The cursor keeps the movement
  operation pending through the hold, and the one-shot driver sends no start.
- `missed_auto_resume` decides what happens when the signal was released but the controller
  still holds the robot after `resume_detect_s`:
  - `fail` (default, and always under `strict`) raises `ResumeNotTakenUp`, so an upstream
    defect stays visible.
  - `start` sends one start from the SDK within `resume_window_s` (the same rules as an
    ignored resume), so a production cell keeps moving.

Kept under both strategies: SDK-side signal observation (push only) and the bus-loss guard.
Per wbr!2384 the controller keeps using the last cached bus value when the bus-IO service goes
away. The guard's pause is a user pause, which drops the controller's `pause_on_io` and pending
`set_outputs` and is resumed only by a start. Synchronized multi-group sessions always use
`sdk`: one group resuming alone would break the shared time parameterization.

`controller` needs an API client that has the field; without one it fails at the start instead
of silently sending a terminal pause.
