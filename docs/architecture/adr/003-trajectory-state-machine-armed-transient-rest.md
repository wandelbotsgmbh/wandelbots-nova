# ADR 003 — The trajectory state machine reads frames by regime: armed, transient, rest

Date: 2026-09-17. Status: accepted. Amends ADR 002 §1 (`002-io-pause-is-resumable.md`, on
branch `feat/pause-on-io-resume` at the time of writing; its §1–2 land on `main` with this
decision, §3–5 follow with that branch).

## Context

On 2026-09-16 a one-shot `execute()` hung although the robot ran its trajectory to the end.
The per-frame trace (250 Hz state stream, trajectory `21a04c31`) showed two controller
artefacts at motion start that the SDK's `TrajectoryExecutionMachine` read as a pause:

1. `standstill` dropped to `false` one control cycle before `execute.state` changed from
   `PAUSED_BY_USER` to `RUNNING`. The machine went `executing → pausing`.
2. In `pausing` the machine only asked "standstill?" and never looked at the discriminator, so
   13 `RUNNING` frames were ignored. Then `standstill` flickered back to `true` for eleven
   `RUNNING` frames while the location advanced (an upstream defect, reported to RAE), and
   `pausing → paused` fired. The cursor resolved the FORWARD operation *as paused* at
   location 0.0065.
3. `paused` had no branch in `process_motion_state`; only a `start` leaves it, and the cursor
   sends `start` only while an operation is in progress. `detach_on_standstill` fires on
   `ended` only. The robot finished (`END_OF_TRAJECTORY`, location 7.0), the cursor tracked
   nothing, `execute()` never returned and nothing was logged.

Also known from ADR 002: between `InitializeMovementRequest` and the first motion the
controller publishes `PAUSED_BY_USER` at standstill (the *parked* shape), and after a resume it
re-publishes the terminal state it was in (`PAUSED_ON_IO`, `END_OF_TRAJECTORY`) for a few
cycles before `RUNNING`. The machine handled the parked shape by bouncing `executing ⇄ paused`
on every frame, with the cursor refusing to complete and re-sending `start` (22 times per
execution in the trace); ADR 002 handled the re-published terminal with a cursor-side
stale-frame guard.

## Decision

1. **A new `armed` state models "start issued, robot not yet moving".** `start` enters
   `armed`, the first `RUNNING` frame enters `executing`. While armed, the parked
   `PAUSED_BY_USER` at standstill, `WAIT_FOR_IO`, bare frames and the *stale terminal* the
   start was issued out of (named by the owner: `arm(stale_terminal=…)`) change nothing. A
   `PAUSED_BY_USER` without standstill is the robot leaving the parked state and is
   remembered; a parked frame after that, or after the owner called `request_pause()`, is a
   real pause. `PAUSED_ON_IO` that is not stale is a real pause (no parked look-alike exists
   for it; ADR 002 §2). `END_OF_TRAJECTORY` at standstill that is not stale completes — a
   zero-length trajectory may never show motion.
2. **Transient states follow the discriminator.** In `pausing` and `ending` the robot is still
   moving, so a `RUNNING` frame is not a contradiction: the pause or end never settled, or the
   controller re-armed. The machine returns to `executing`. Waiting for standstill would hang
   or, at the next standstill flicker, conclude the wrong thing. `pausing` + `END_OF_TRAJECTORY`
   completes the trajectory (a pause requested just before the end).
3. **Rest states enforce the discriminator.** In `paused` and `ended` the operation is resolved
   and standstill was confirmed; the owner sends `start` before processing any frame of a new
   movement. A `RUNNING` frame arriving at rest therefore means the robot moves without this
   owner having started it:
   - `paused` with `pause_reason = IO` → `executing` (observed resume; ADR 002 §1 kept for IO
     pauses, since a controller that clears an IO pause by itself is conceivable);
   - `paused` with `pause_reason = USER`, and `ended` → `error`, with `failure_reason` and
     `failed_frame` set. **This amends ADR 002 §1**, which followed the wire for every pause:
     nobody but this cursor may resume a user pause or restart a finished trajectory.
   Other discriminators at rest are tolerated (logged): a stop, another client initialising,
   a bus-IO service coming back.
4. **Errors surface.** `TrajectoryCursor` raises `UnexpectedTrajectoryState`
   (`nova.exceptions`, an `ErrorDuringMovement`) from its state monitor when the machine is in
   `error`, failing the current operation and the protocol loop, so `execute()` raises with
   the offending frame instead of hanging. In one-shot mode (`move_forward`) a `paused` with
   `pause_reason = USER` that the cursor did not request raises the same way — there is no
   resume path there.
5. **IO pauses on `main`** (absorbed from ADR 002 §1–2): `PAUSED_ON_IO` is a pause, the
   operation completes with `OperationResult.paused_on_io = True` and the cursor stays attached.
   `move_forward` cannot observe the IO yet and ends the execution with a warning (the
   observable behaviour of the previous `→ ended` mapping, made explicit). The resume
   machinery of ADR 002 §3–5 (`io_condition`, the `move_forward` supervisor, the bus-loss
   guard) follows separately.

## Alternatives considered

- **Debounce `standstill`** (require N consecutive standstill frames before concluding a
  transient state). Rejected: it adds N control cycles of latency to every completion, still
  trusts a flag that is wrong upstream, and does nothing about the stale `PAUSED_BY_USER` at
  motion start. The discriminator already says what the robot is doing.
- **Fail on `RUNNING` in transient states too.** Rejected: `ending` is also entered from an
  intermediate `forward_to` stop and `pausing` from an IO pause, and a `backward()` issued
  while decelerating legitimately produces `RUNNING` there. Returning to `executing` is
  correct in every reading; failing would be wrong in some.
- **Always follow the wire at rest** (ADR 002 §1 as written). Rejected for user pauses and
  finished trajectories: it would have masked this very defect — the FORWARD operation would
  still have resolved at location 0.0065 while the trajectory silently ran to the end — and a
  robot moving without a start from its controller process is worth an exception.
- **Flags inside `executing` instead of an `armed` state.** Rejected: the parked phase would be
  invisible in traces and the rules would sit inside the `PAUSED_BY_USER` branch of the
  executing handler; an explicit state names the phase, resets its memory on every start and
  gives the stale-terminal rule one home.

*In the context of* reading a 250 Hz level-based state stream whose two fields do not flip in
the same step, *facing* a hang that produced no error, *we decided* to give the start phase its
own state, let transient states trust the discriminator and rest states enforce it, *accepting*
that an external pause before motion still looks like the parked frame.

## Consequences

- The cursor's `may_complete_as_paused` guard and ADR 002's stale-terminal cursor guard are
  gone; `armed` owns both rules. The cursor still decides *which* terminal is stale
  (`_stale_terminal_state_for`: `PAUSED_ON_IO` after an IO pause, `END_OF_TRAJECTORY` after an
  intermediate stop with room to move) and passes it to `arm`.
- `TrajectoryCursor.pause()` tells the machine (`request_pause()`), so a pause issued before
  the robot moved concludes on the parked frame.
- A trace of a healthy execution reads `armed → executing → ending → ended`; the 22-frame
  `executing ⇄ paused` ping-pong is gone.
- An external pause issued *before* motion starts is still indistinguishable from the parked
  frame: the FORWARD operation never completes. That hang existed before; a timeout in `armed`
  is the place to address it.
- Tests that feed `PAUSED_BY_USER` at standstill straight after `start` and expect `paused`
  encode the pre-ADR reading; they now call `request_pause()` or feed a `RUNNING` frame first.

## References

- Frame-by-frame trace of the 2026-09-16 hang (trajectory `21a04c31`, F001–F1918):
  https://claude.ai/artifact/MVGXsf84PxHjFGV3Tzo58x
- `nova/cell/movement_controller/README.md` — wire behaviour, decision table, diagrams.
- `tests/cell/test_trajectory_state_machine.py::TestObservedStartLag` — the capture replayed.
- `docs/architecture/incoming/pause-on-signal-evaluation.md` — the IO-pause measurements ADR 002
  and this decision rely on.
