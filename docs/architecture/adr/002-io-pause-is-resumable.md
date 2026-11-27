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
   which observes the IO without polling (see Consequences) and returns once the condition no
   longer holds. After every resume the driver also watches for the condition to hold again: a
   start that reaches the controller while it holds is accepted but leaves the robot paused at
   the same location, frames the stale-terminal guard of point 2 drops, so the operation alone
   would never resolve. The driver then waits for the next release and starts again. The loop
   lives in `nova/cell/movement_controller/io_pause_driver.py`.
4. Resume is automatic when the condition clears — the default. `resume_on_io_release=False`
   on `execute()` / `plan_and_execute()` keeps the earlier meaning: `pause_on_io` is a **stop
   condition** (move until a contact sensor fires) and the call returns where the robot stopped.
   Apps used it that way before this decision (the bundled review skill's DEV-003 / SAF-011 /
   IO-008 describe it); resuming would drive such a move on into the part once the sensor
   reading drops.
5. Pausing is per motion group. A synchronized multi-group motion pauses as a whole on **one**
   signal: `TrajectoryExecutor.execute()` / `attach()` take a single `pause_on_io` for every
   group, and `execute()` resumes all groups together through the start barrier, which
   re-synchronizes them; they drift apart only by their own braking ramps. A controller-IO
   condition is rejected when the groups span controllers, since it would name a different
   signal on each.
6. The intended wiring is a **motion-enable signal**: the IO reads `True` while the robot is
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
  unreadable IO as "pause") is reported to the RAE/bus-IO team. The guard is watched like the
  movement itself: if it cannot watch (no NATS client, a failing subscription), the SDK pauses
  the robot and ends the execution with that error at once. A bus-IO condition without a usable
  NATS client fails before the robot moves. Under a stop condition a lost bus pauses the robot
  and raises `IOConditionUnavailable` rather than returning as if the condition fired.
- **No polling of the API.** Controller IOs are observed on the `stream_io_values` websocket
  (the controller-IO REST endpoint returns 429 to a write while a read is in flight), bus IOs
  and the bus state over NATS with a single initial read after subscribing. When a source cannot
  be observed, `execute()` fails with `IOConditionUnavailable` instead of degrading to polling.
  The controller-IO value stream is reopened when it closes; only opens in a row that deliver no
  value count towards giving up, so a long pause survives routine socket closes.

- `execute()` blocks through IO pauses and returns at the target; programs need no code to
  handle the pause itself. An `asyncio.timeout` around it also bounds the pause.
- Breaking for callers that relied on `execute()` returning at an IO pause: they must pass
  `resume_on_io_release=False`.
- Cursor callers must check `OperationResult.paused_on_io`: `final_location` no longer implies
  the target was reached.
- The SDK depends on the controller not auto-resuming. If a future controller resumes on its
  own, the machine's observed-resume transition keeps the state consistent and the redundant
  start the SDK sends while running is a no-op retarget.
- Program-wide pausing (between motions) is a follow-up; `IOConditionWatcher` is the primitive
  to gate non-motion steps with the same signal.
