# `move_forward` → `TrajectoryCursor` — transition plan

Status: **Phases A and B implemented; C–E open.** This document argues that the two
movement controllers should become **one** implementation, and sequences the transition.

It is a companion to the `command-routine.md` analysis (an unpublished working document on
the incoming NOVA CommandRoutine format), which flagged the missing `set_outputs` on the
cursor as a P0 blocker; references to its sections are kept for whoever holds that draft.
This document stands on its own for everything needed to continue the work here.

Date: 2026-08-04 (implementation notes added 2026-08-05)
SDK baseline analysed: `main` @ `3e9e513`
Branches analysed: `feat/NDX-155-simple-custom-actions` @ `8112b15`,
`feat/path-triggers-distance-offset` @ `5560279`, `feat/motion-guards` @ `878be25`,
`fix/trajectory-completion-standstill-watchdog` @ `9e7e4f4`

---

## 0. Why now — the protocol loop has forked five ways

`move_forward` and `TrajectoryCursor.cntrl` are both async generators implementing the
*same* `executeTrajectory` websocket protocol: initialize → start state monitor →
`StartMovementRequest` → consume responses → terminate on standstill. Every feature that
needs to *interrupt* that loop has forked it rather than shared it:

| Where | Shape | What it added |
|---|---|---|
| `main` `move_forward.py` (172 ln) | linear generator | baseline: init, one Start, monitor, error consumer |
| `main` `trajectory_cursor.py` (1314 ln) | intent slot + command loop | forward/backward/pause/target, playback speed, events |
| `feat/path-triggers-distance-offset` `move_forward.py` (~260 ln) | **+ command queue, + response consumer, + pause/resume callbacks** | async actions; re-invents the cursor's `_request_loop` and `_response_consumer` |
| `feat/motion-guards` `guarded_move_forward.py` (385 ln) | third copy | guard evaluation + stop/resume |
| `fix/trajectory-completion-standstill-watchdog` `move_forward.py` (+82 ln) + `stall_watchdog.py` (273 ln) | fourth copy | completes trajectories when the terminal event is dropped — **applied to `move_forward` only; the cursor still has the bug** |

Two observations make the merge urgent rather than merely tidy:

1. **`feat/path-triggers-distance-offset` rewrote `move_forward` into a worse cursor.** It
   adds an `asyncio.Queue` of commands, a `PauseMovementRequest`/`StartMovementRequest`
   pair driven by executor callbacks, and a response consumer that tolerates pause acks.
   That is structurally `Intent` + `_request_loop` + `_response_consumer` — minus the
   operation futures, the state-transition safety, and the tests.
2. **Bug fixes are already diverging.** The stall watchdog fixes a real
   dropped-terminal-event hang. Part of that fix lands in the shared
   `trajectory_state_machine.py` (+25 ln) and so would benefit both, but the watchdog
   wiring itself is in `move_forward` (+82 ln) and `stall_watchdog.py`. The cursor runs
   the same `async for motion_group_state in …` loop with the same dependency on the FSM
   reaching a terminal state, and stays exposed.

The claim in the task is exact: **`cursor.forward()` from location 0 with no target *is*
`move_forward`.** Everything else the cursor offers is a superset.

---

## 1. The equivalence, wire message by wire message

| Step | `move_forward` (main) | `TrajectoryCursor` |
|---|---|---|
| Init | `InitializeMovementRequest(trajectory, Location(0))`, `move_forward.py:64-67` | `init_movement_gen(motion_id, stream, self._current_location)`, `:1282-1306` — identical, parameterised location |
| Init failure | raises `InitMovementFailed`, `:86` | raises `InitMovementFailed`, `:1314` — identical |
| Monitor ordering | monitor started **before** Start, 5 s ready timeout, `:91-104` | monitor started before `_request_loop`, 5 s ready timeout, `:1020-1032` — identical intent |
| Start | `StartMovementRequest(FORWARD, set_outputs, start_on_io, pause_on_io)`, `:107-112` | `Intent.to_commands()` → `StartMovementRequest(direction, target_location, start_on_io, pause_on_io)`, `:403-414` — **no `set_outputs`** |
| State handling | `TrajectoryExecutionMachine`, `:44` | `TrajectoryExecutionMachine`, `:1109` — same FSM |
| Completion | returns when `machine.is_ended`, `:46-50` | `_complete_operation()` on `is_ended`/`is_paused`; loop exits only if `detach_on_standstill`, `:1122-1126` |
| Movement error | `ErrorDuringMovement`, `:152` | bare `Exception` inside a `TaskGroup` → surfaces as `BaseExceptionGroup`, `:1167` |
| Termination | generator returns → websocket closes | `detach()` → `_stop_event` → `_request_loop` breaks → `cntrl` returns |
| Motion events | none | `motion_started` blinker signal + `MotionEvent`, 5 Hz ticker |

Everything in the left column exists in the right column, or is a small enumerable delta.

---

## 2. Delta list — what must change before the cursor is a drop-in

| # | Delta | Anchor | Priority |
|---|---|---|---|
| D1 | `Intent` carries no `set_outputs`; a routine's `io_write`s are silently dropped under the cursor | `trajectory_cursor.py:369-415` | **P0** |
| D2 | Movement errors surface as `BaseExceptionGroup`, not `ErrorDuringMovement` | `trajectory_cursor.py:1165-1169`, `:1040-1042` | **P0** |
| D3 | No auto-start: the cursor requires an explicit `forward()` after `cntrl` is wired up | new `autostart` ctor arg | **P0** |
| D4 | Ctor takes a live `AsyncIterator`; `MovementControllerContext` supplies a *factory* (`motion_group_state_stream_gen`) | `trajectory_cursor.py:495`, `container.py:167` | **P0** (trivial) |
| D5 | With `detach_on_standstill=False` the monitor never exits on an infinite state stream → the websocket stays open forever | `trajectory_cursor.py:1122-1126` | **P0** for parity |
| D6 | `_motion_event_updater` ticks at 5 Hz unconditionally; every `execute()` would start emitting blinker/NATS traffic | `trajectory_cursor.py:1194-1214` | P1 |
| D7 | State monitor `continue`s while no operation is in progress, so states arriving before the first `forward()` are dropped — and a fast/finite stream can exhaust and `detach()` before autostart ever sets its intent | `trajectory_cursor.py:1104-1107`, `:1131-1135` | **P0** (raised from P1 — see D12) |
| D8 | Stall watchdog / dropped-terminal-event fix exists only on `move_forward` | `fix/trajectory-completion-standstill-watchdog` | P1 |
| D9 | `actions=None` location-only mode must keep working; `_raw_actions` (non-motion actions) must become addressable | `trajectory_cursor.py:524-534` | P1 |
| D10 | The cursor is unreachable from `MotionGroup.execute()` — only tests build an adapter factory | `test_trajectory_cursor_integration.py:177-200` | P1 |
| D11 | The cursor requires `joint_trajectory`; `MovementControllerContext` has no such field, and the existing `move_forward` tests construct the context without one | `trajectory_cursor.py:496`, `container.py:162-167` | **P0** |
| D12 | **`_request_loop` discards a pending intent on stop.** It checks `_stop_event` before consuming `_pending_intent`, so an intent queued before a detach is dropped and the command never goes out. **Measured** — see below. *Resolution: dropping is the correct, safe behaviour; the actual defect is D16, which reported the dropped movement as a success.* | `trajectory_cursor.py:1047-1062` | **P0** |
| D16 | **The cursor reports success for movement it never commanded.** Operation completion is driven purely by the state stream, with no correlation to whether a `StartMovementRequest` was sent or acked. **Measured**: `await cursor.forward()` returned `OperationResult(final_location=3.0, error=None)` having emitted only an `InitializeMovementRequest` | `trajectory_cursor.py:1104-1126` | **P0** |
| D13 | **`actions=[]` is a supported execution path and the cursor rejects it.** An empty list is not `None`, so the ctor's end-location check demands a trajectory ending at `0.0` and raises `ValueError`. `execute(joint_trajectory=…, actions=[], tcp=…)` for a preplanned trajectory is exercised today | `trajectory_cursor.py:524-544`, `test_motion_group_movement.py:77` | **P0** |
| D14 | **`_in_queue` fills with nobody draining it in adapter mode.** The monitor enqueues every execute-bearing state for `__aiter__`, but `MotionGroup._execute` keeps its own `stream_state()` subscription and never iterates the cursor. Reframed by §3: the queue is the overlay layer's input, so the fix is a bounded/faithful tee, not a switch | `trajectory_cursor.py:548-550`, `:1114-1117`, `motion_group.py:1006-1031` | P1 |
| D15 | **Stream-lifecycle semantics differ, beyond exception type (D2).** `move_forward` treats a state-monitor task that finished *for any reason* as success — `if state_monitor in done: return` never inspects the task result, so a state-stream exception is swallowed. The cursor's `TaskGroup` propagates it. Response-stream EOF, state-stream EOF without a terminal state, and a missing `StartMovementResponse` ack are each handled differently by the two implementations | `move_forward.py:126-152`, `trajectory_cursor.py:1148-1189` | **P0** |

D1–D5, D11–D16 are the drop-in blockers; D6–D10 are quality-of-transition. D12 and D16 are
pre-existing cursor defects that the autostart work merely *surfaced* — see below.

### D1 in detail — `set_outputs` re-send under override semantics

**Confirmed on hardware** (virtual KUKA `kuka-kr6_r700_sixx`, NOVA @ `172.31.10.174`,
2026-08-05). Both properties were measured, not assumed:

| Experiment | Result |
|---|---|
| Start with `[A@1.0]`, pause before 1.0, resume with `[B@2.0]`, run to end | `A=False`, `B=True` → **the second Start replaced the first list; A never fired** |
| Start with `target_location=1.0` + `[A@1.0]` (stops exactly on the overlay), reset A externally, resume from 1.0 with `[A@1.0]` | `A=True` → **re-fires at the boundary** |

So: **a new `StartMovementRequest` overrides the previously attached overlay**, and an output
at *exactly* the resume location does fire again. The released client's field doc is
consistent with both:

> `set_outputs`: Attaches a list of output commands to the trajectory. The outputs are set
> to the specified values right after the specified location was reached. **If the
> specified location is located before the start location (forward direction: value is
> smaller, backward direction: value is bigger), the output is not set.**

Three consequences, and they are what actually shape the implementation:

1. **Every `StartMovementRequest` must carry the full list — always.** Under override
   semantics a resume that omits `set_outputs` does not "keep the existing overlay", it
   **silently clears it** for the rest of the trajectory. Measured above: `A` was dropped by
   a resume that simply did not re-send it. This makes `set_outputs` a **constructor-level
   default on the cursor**, applied automatically to every emitted Start, rather than a
   per-call argument — a per-call argument is a footgun where the first resume that forgets
   it disables all remaining IO.
   - Concrete casualty for §5.4: `feat/path-triggers-distance-offset`'s resume sends a bare
     `StartMovementRequest(direction=DIRECTION_FORWARD)`. That branch therefore **drops every
     remaining IO write on the first blocking async action.** This is now a measured defect,
     not a suspicion, and another reason not to port its `move_forward` rewrite.
2. **Re-triggering at the boundary is accepted, not worked around.** No client-side "already
   fired" bookkeeping. It is the natural consequence of override semantics.
   - Caveat to document for users rather than engineer around: harmless for level-triggered
     outputs, **not** harmless for anything edge-sensitive downstream (e.g. a PLC counting
     pulses). Worth one line in the `io_write` docstring.
3. `command-routine.md` Q5 is answered for `set_io`: outputs re-fire on backward travel
   through the same location, and the server — not the SDK — owns the rule.

This also pins an ordering constraint that the path-triggers branch introduced:
`to_set_io(trigger_overrides)` must be evaluated **after** planning (time triggers need the
planned time profile), so `set_outputs` reaches the cursor as constructor data computed by
`MotionGroup._execute` — not as something the cursor derives itself. Which is exactly the
layering rule of §3.

### D2 in detail

`_response_consumer` raising inside `asyncio.TaskGroup` converts any error into a
`BaseExceptionGroup`, which `cntrl` re-raises as-is. `move_forward` callers catch
`ErrorDuringMovement`. Fix: raise `ErrorDuringMovement` from the consumer, and unwrap
single-exception groups at the `cntrl` boundary so the caller-visible type is unchanged.

### D11 in detail — do not make `joint_trajectory` required

`tests/cell/test_process_motion_group_state.py:67-76` builds a `MovementControllerContext`
with only `combined_actions`, `motion_id`, `start_on_io` and the state-stream factory.
Adding a **required** `joint_trajectory` to the context breaks those tests at construction
— which would forfeit the Phase C gate (§4) before it is even run.

The cursor uses `joint_trajectory` in exactly three places: the ctor's end-location
sanity check (`:536-544`), the `end_location` property (`:570-573`), and the
`forward_to_next_action` upper bound (`:853`). None of them is on the path of a one-shot
autostart run with no `target_location`. So:

- `MovementControllerContext.joint_trajectory` is **optional** (`| None = None`);
- `TrajectoryCursor` accepts `joint_trajectory=None` and raises a clear error only if a
  location-bounded operation (`forward_to`, `forward_to_next_action`,
  `get_movement_options`) is attempted without it.

That keeps both the existing unit tests and the plug-in seam untouched.

### D12, D16 and D15 in detail — autostart surfaced two real cursor defects

Trying to place `autostart` raised the question of whether it belongs *in* the cursor or in
the adapter. Probing it produced a sharper answer than expected: **autostart is fine in the
adapter; the cursor has two pre-existing bugs that autostart merely exposed.**

Measured against `main` @ `3e9e513`, with `detach_on_standstill=True`, a cursor whose
`forward()` is called before `cntrl` — i.e. the adapter shape:

| State stream | Requests emitted |
|---|---|
| paced, then blocks (live websocket) | `InitializeMovementRequest`, `StartMovementRequest` ✅ |
| finite, then EOF (existing unit-test fixtures) | `InitializeMovementRequest` only ❌ |

**D12 — the intent is discarded on stop.** `_request_loop` (`:1047-1062`) does:

```python
await self._intent_event.wait()
if self._stop_event.is_set():
    break                     # ← a pending intent is thrown away here
intent = self._pending_intent
```

The monitor's `finally: self.detach()` (`:1131-1135`) sets both `_stop_event` and
`_intent_event`. If the state stream ends before `_request_loop` gets its first turn, the
already-queued `forward()` is dropped and no `StartMovementRequest` is ever sent.

**Resolution — dropping the intent is correct; do not "fix" it.** Flushing the pending
intent instead was implemented, measured, and **reverted**: it means an explicit
`cursor.forward(); cursor.detach()` cancels the caller's future *and then still commands the
robot to move*. Verified on the implementation branch — `future.cancelled() == True` while a
`StartMovementRequest` reached the wire. A stop must always win over a queued command; on the
teardown path there is also nothing left to monitor the movement. The genuine defect behind
the original symptom is D16.

**D16 — the cursor then reports that it succeeded.** In the same scenario,
`await cursor.forward()` resolved to
`OperationResult(operation_type=FORWARD, final_location=3.0, error=None)` and
`cursor._current_location` advanced to `3.0` — the end of the trajectory — with only an
`InitializeMovementRequest` on the wire and no `StartMovementResponse` ever received. The
operation lifecycle is driven entirely by the state stream (`:1104-1126`) with no
correlation to whether the command was sent, so **the cursor can report a completed
traversal for movement it never commanded.** A caller that awaits `forward()` and then
advances its program would do so with the robot still at the start.

This is exactly the failure mode D15 describes from the other side: `move_forward` at least
requires a `StartMovementResponse` before it accepts completion (`move_forward.py:116-121`).
The cursor did not.

**Resolution — split the concern along the user's own rule:**

- The **fix goes inside the cursor** (P0, benefits every existing cursor user, nothing to do
  with `move_forward`): gate operation completion on the operation having actually been
  commanded, and fail — rather than silently abandon — an operation whose state stream ends
  first. A dropped intent then surfaces as `ErrorDuringMovement` instead of a phantom
  success.
- The **autostart *policy* stays in the adapter.** No `autostart` constructor flag: the
  adapter calls `cursor.forward()` before returning `cursor.cntrl`. "Start immediately" is a
  `move_forward` policy, not a cursor capability, and keeping it out preserves §3's layering.

D7 remains a genuine but lesser issue: the monitor's `if current_op is None: continue`
(`:1104-1107`) skips the state tee and the location update, not just completion handling.
The fix is to always process and tee, and gate only `set_running()` / `_complete_operation()`
on having an operation — note that this interacts with the FSM kick at `:1096-1102`, which
needs re-examination at the same time.

---

## 3. Target architecture — three layers, not one class

The question "should IO, markers and events live in the cursor?" is the right one to ask
before any of this lands. Today the cursor already carries motion events, `start_on_io` /
`pause_on_io` pass-through, a `_raw_actions` field it does not use, and — under D1 — would
gain `set_outputs`. The branches queue up async actions, guards and markers behind that.
That is a god object forming in slow motion, and 1314 lines is already past comfortable.

**Answer: separate them — but along the seam that the transport actually dictates, which is
not "motion vs. non-motion".** There are two kinds of overlay and they belong in different
places:

| | **Server-side overlay** | **Client-side overlay** |
|---|---|---|
| Examples | `set_outputs`, `start_on_io`, `pause_on_io`, `distance_offset` triggers | markers, async actions, guards, motion events |
| Mechanism | *fields on `StartMovementRequest`* | *reactions to location changes* + commands back |
| Executed by | the controller, at the resolved location | the SDK, at whatever rate states arrive |
| Precision | exact | best-effort, bounded by state rate |
| Can it live outside the cursor? | **No** — it has to be in the message | **Yes** — needs only a location stream and pause/forward |

Server-side overlays *must* travel with the request, so the cursor has to carry them. But it
should carry them as **opaque payload**: a pre-built `list[SetIO]` it attaches to every Start
and never interprets. The rule that keeps this honest is an import rule —

> **The cursor must not import `WriteAction`, `to_set_io()`, path triggers, or any overlay
> concept.** It accepts already-resolved API models. Today it imports `CombinedActions`
> (`trajectory_cursor.py:59`) to compute `.motions`; that is the existing smell, and the
> direction of travel is to remove it, not to add more.

Client-side overlays need exactly two things, and the cursor **already exposes both**:

- a location/state stream — `__aiter__` / `_in_queue` (`:1265-1279`)
- movement control — `forward()` / `pause()` returning awaitable futures

So they belong in a layer *above* the cursor that drives it through its public API:

```
TrajectoryCursor        protocol + location + movement commands.
                        Carries opaque server-side overlay payloads.
                        Knows nothing about IO, markers, actions-as-events.

TrajectoryOverlay       consumes the cursor's state stream, fires client-side
  (new, Phase D)        overlays, drives cursor.pause()/forward().
                        Owns markers, async actions, guards, MotionEvent emission.

move_forward(context)   adapter: builds a cursor (+ an overlay only if needed).
```

Three things fall out of this that are worth more than the tidiness:

1. **The latency objection answers itself.** Routing an overlay through a layer boundary
   costs a queue hop. But anything needing *precise* placement is already server-side for
   exactly that reason (that is why `distance_offset` exists). Client-side overlays are
   inherently best-effort at state-stream rate, so the extra hop is noise. The layer boundary
   is free precisely where it is allowed to exist.
2. **D14 is reframed rather than fixed by a flag.** `_in_queue` is not dead weight to be
   switched off — it is the overlay layer's input. It is only unused in the bare
   `move_forward` case, where the adapter creates no overlay, and there it needs a bound
   rather than a kill switch. It also has to become a *faithful* tee: today it only receives
   states with `execute` set, and only once an operation is running (D7), which is not good
   enough for a guard that must veto *before* movement starts.
3. **D9 dissolves.** Making `_raw_actions` addressable stops being a cursor feature; the
   overlay layer owns non-motion actions, and the cursor's `_raw_actions` field can
   eventually be deleted rather than grown.

This also lines up with `command-routine.md` §4.1: `compile_routine()` produces actions plus
overlay commands; the actions go to `plan()` and then the cursor, the client-side overlay
commands go to the overlay layer. Neither layer needs to know about teaching schemas.

**Staging.** Do *not* build the overlay layer in Phases A–C — those are pure unification and
add no overlay logic. It becomes the shape of Phase D, replacing "bolt the branch features
onto the cursor". In the meantime the rule stands on its own: **no new client-side overlay
logic goes into `TrajectoryCursor`.** That single rule is what stops `feat/motion-guards`'
`_check_and_trigger_async_actions` sketch from being merged as-is.

Motion events (D6) are the one awkward case: they are a client-side overlay by this
definition, but they already exist in the cursor and the tuner depends on them
(`tuner.py:140-144`). Recommendation: leave them where they are for now, gate them with
`emit_motion_events`, and move them out with the rest in Phase D — do not let "it is already
there" become the argument for adding the next one.

### The adapter

```python
def move_forward(context: MovementControllerContext) -> MovementControllerFunction:
    actions = list(context.combined_actions.items)
    cursor = TrajectoryCursor(
        motion_id=context.motion_id,
        motion_group_state_stream=context.motion_group_state_stream_gen(),
        joint_trajectory=context.joint_trajectory,
        actions=actions or None,  # D13: [] must not mean "zero-length trajectory"
        set_outputs=context.combined_actions.to_set_io(),  # opaque payload
        start_on_io=context.start_on_io,
        pause_on_io=context.pause_on_io,
        initial_location=0.0,
        detach_on_standstill=True,
        emit_motion_events=False,
    )
    cursor.forward()  # autostart is adapter policy, not a cursor flag (D12)
    return cursor.cntrl
```

`set_outputs`, `start_on_io`, `pause_on_io` (as **constructor defaults** applied to every
emitted Start — see D1) and `emit_motion_events` are **new constructor parameters** added by
Phases A and B; none exists today (`trajectory_cursor.py:492-500`). There is no `autostart`
flag — that is adapter policy — and no `queue_states` flag, since the queue is the overlay
layer's input and needs a bound rather than an off switch (D14).

The `cursor.forward()` future is deliberately not awaited here. It must still be consumed —
attach a done-callback that logs, or the adapter will produce "Future exception was never
retrieved" warnings on failure paths.

The trigger-override argument to `to_set_io()` arrives later, with §5 step 4 — the adapter
calls the parameterless form until then, so Phase C stays independently landable.

Consequences:

- `MovementController` / `MovementControllerContext` stay the public plug-in seam. No
  caller of `execute()` / `plan_and_execute()` changes. **That is what makes the transition
  seamless.**
- `MovementControllerContext` grows an **optional** `joint_trajectory` — the cursor needs it
  for `end_location`, `move_forward` never did, and existing tests construct the context
  without it (D11). It is available at the single production construction site,
  `motion_group.py:993-1001`.
- Features stop forking the loop: path triggers become `set_outputs` payload, async actions
  and guards become overlay-layer consumers of the cursor's public API, the stall watchdog
  gets fixed once.

**Not proposed:** deleting `move_forward`. Keeping it as the default controller name is
what buys a zero-churn transition. `command-routine.md` §4.2's position — the cursor is a
transport object, teaching-domain compilation stays in `nova/teaching/` — is unaffected, and
the overlay layer is the missing piece between the two.

---

## 4. Phases

Each phase is independently landable and independently verifiable.

> **Status: Phases A and B are implemented** on `agents/merge-move-forward-into-cursor`
> (`nova/cell/movement_controller/trajectory_cursor.py`, plus
> `tests/cell/test_trajectory_cursor_parity.py` and one new integration test).
> Verified: 363 unit tests and 14 integration tests green against a live cell, `ruff` and
> `ty` clean. `move_forward` is untouched — Phases C–E remain open.

### Phase A — close the parity deltas in the cursor (D1, D2, D4, D5, D11, D13, D15) — **DONE**

1. `Intent.set_outputs: list[api.models.SetIO] | None`, attached to every
   `StartMovementRequest` in `to_commands()`. Because a new Start **overrides** the previous
   overlay (D1), these are **constructor-level defaults** applied automatically to every
   emitted Start — not per-call arguments that a resume can forget and thereby clear the
   overlay. Same for `start_on_io` / `pause_on_io`.
2. `ErrorDuringMovement` from `_response_consumer`; unwrap single-exception
   `BaseExceptionGroup` at `cntrl`. Then, separately, **decide and document** the
   state-stream EOF, state-stream exception, response-stream EOF and missing-ack
   semantics (D15) rather than copying `move_forward`'s accidental behaviour.
3. Accept `AsyncIterator | Callable[[], AsyncIterator]` for the state stream.
4. Make `joint_trajectory` optional on the cursor; fail loudly only on location-bounded
   operations that need it (D11).
5. Treat `actions=[]` as "no action metadata" rather than "a zero-length trajectory"
   (D13).
6. Verify `detach_on_standstill=True` reliably terminates `cntrl`.

*Verify:* unit tests asserting the exact `StartMovementRequest` payload for
forward/backward/resume; a test that `set_outputs` survives pause→resume; an
`ErrorDuringMovement` propagation test; a `joint_trajectory=None` / `actions=[]` test.
Existing cursor tests stay green.

> Phase A alone closes `command-routine.md` §3.1 and is worth landing even if the rest of
> this plan is rejected.

### Phase B — fix the cursor defects autostart exposed (D3, D6, D7, D14, D16) — **DONE**

This phase is **cursor bug-fixing**, not feature work — the name "one-shot mode" was wrong.
All of it benefits existing cursor users; none of it is about `move_forward`.

1. **D12** — confirmed that discarding a pending intent on stop is **correct**: a stop must
   win over a queued command, or an explicit abort is followed by the robot moving. No
   change; the symptom is addressed by D16.
2. **D16** — gate operation completion on the operation having been commanded, so
   `forward()` cannot resolve successfully for movement that was never sent. `move_forward`
   already requires the `StartMovementResponse` (`move_forward.py:116-121`); the cursor must
   too. Also fail — rather than silently abandon — an in-progress operation whose state
   stream ends first, otherwise the gate turns a lost command into a hang.
3. **D7** — always process and tee incoming states; gate only `set_running()` /
   `_complete_operation()` on having an operation. This also makes `_in_queue` the faithful
   tee that §3's overlay layer needs.
4. **D3/D6** — add `emit_motion_events: bool = True`. **No `autostart` flag** and no
   `queue_states` flag: starting immediately is adapter policy (§3), and the queue is the
   overlay layer's input rather than dead weight.
5. **D14** — bound the state queue (drop-oldest) so a cursor whose iterator is never
   consumed cannot grow memory for the length of a trajectory.

*Verify:* a queued command must never reach the wire after a stop (`forward()` then
`detach()` sends nothing and the future is cancelled), and a command that was dropped must
surface as `ErrorDuringMovement` rather than as a traversal to the end of the trajectory.

**Implementation note (D16).** The completion guard belongs on *"was the command sent"*, not
*"was it acknowledged"*. Gating on the acknowledgement loses a race against a fast state
stream — the terminal state can arrive before the ack is processed, wrongly failing a
legitimate movement. The cursor therefore marks the operation commanded in `_request_loop`
immediately **before** yielding the request (after the `yield` is too late: a generator
suspends there until the consumer pulls again). This is applied to `PauseMovementRequest` as
well as `StartMovementRequest` — a pause is an operation too, and gating it on its ack alone
left `pause()` unresolved whenever the ack was delayed or lost.

**Implementation note (test fixtures).** Mocked state streams must not run ahead of the
request loop: a stream that completes instantly tears the cursor down before it has had a
turn, which is the artefact that originally looked like D12. The parity fixtures emit an
idle state immediately (a real motion-group stream is always live) and withhold trajectory
progress until the `StartMovementRequest` has actually been observed.

### Phase C — `move_forward` becomes an adapter (D10)

Rewrite `move_forward` as §3. Add the optional `joint_trajectory` to
`MovementControllerContext`. Expose the live cursor to callers of `execute()` — proposal: an
optional `on_cursor: Callable[[TrajectoryCursor], None]` on `MovementControllerContext`,
which also lets the integration tests drop their hand-rolled adapter factory.

*Verify:* **the existing `move_forward` test files must pass unchanged** —
`tests/cell/test_process_motion_group_state.py`,
`tests/nova/cell/motion_group/test_pause_on_io.py`,
`tests/nova/cell/motion_group/test_motion_group_task_cancelation.py`, plus
`tests/nova/cell/motion_group/test_motion_group_movement.py` (the `actions=[]` preplanned
path, D13). Those tests are the behavioural contract; not editing them is the definition of
"seamless" — with the one documented exception from D15, where `move_forward`'s current
behaviour is a bug and the harness pins the *chosen* semantics instead. Then the
integration suite against a live cell.

### Phase D — build the overlay layer, then fold in the branch features once each

This is where `TrajectoryOverlay` (§3) gets built, and where the branch features land *on
top of* the cursor rather than inside it. See §5 for order. Nothing in this phase touches the
protocol loop again, and nothing adds client-side overlay logic to `TrajectoryCursor`.

Motion events move out of the cursor here too (D6), with the tuner switched over to the
overlay layer in the same change.

### Phase E — cleanup

Delete the standalone protocol code, fold `stall_watchdog` into the cursor's monitor, and
update `nova/cell/movement_controller/README.md` (which currently frames
`TrajectoryExecutionMachine` as "shared across movement controllers" — after this work
there is one). `docs/programs.md` does not mention `movement_controller`, so it needs no
change.

---

## 5. Branch landing order

The four in-flight branches each carry a fork of the loop, so **order is load-bearing**:
land the merge first, then rebase features onto the single implementation. Doing it the
other way round means doing the merge four times.

1. **Phase A + B** — no branch dependencies.
2. **`fix/trajectory-completion-standstill-watchdog`** — rebase `stall_watchdog` onto the
   cursor's `_motion_group_state_monitor` instead of `move_forward`. Fixes the cursor's
   identical latent exposure for free. Independent value; no CommandRoutine risk.
3. **Phase C** — the adapter. Gate: the three unchanged test files above.
4. **`feat/path-triggers-distance-offset`** — split it. The valuable, self-contained parts
   are `path_trigger.py`, `path_trigger_resolver.py`, `to_set_io(trigger_overrides)` and
   `_resolve_set_io_overrides()`; they feed Phase A's `set_outputs` directly and land
   cleanly. **Drop the branch's `move_forward` rewrite entirely** — it is superseded.
   `command-routine.md` §3.4 lists this branch as a P0 prerequisite for `at` triggers, so
   this is the step that unblocks the CommandRoutine work.
   - Caveat: `path_trigger_api_compat.py` monkey-patches `distance_offset` onto
     `api.models.SetIO`. Verified against the installed client — `SetIO` has exactly
     `{io, location, io_origin}` — so the shim is still required. Quarantine it and give it
     a removal trigger (the client release that ships the field); do not spread it further.
5. **`feat/NDX-155-simple-custom-actions` async actions** — `AsyncActionExecutor` becomes a
   `TrajectoryOverlay` participant: its `_pause_callback` / `_resume_callback` collapse into
   direct `cursor.pause()` / `cursor.forward()` calls from the overlay layer. The executor's
   whole reason for existing on that branch is that `move_forward` had no pause/resume; the
   cursor has had it all along, with operation futures to await. Expect the executor to
   shrink. `feat/motion-guards` sketched this wiring *inside* the cursor
   (`_check_and_trigger_async_actions`) and left the pause/resume path unconnected — that is
   the shape §3 rules out; take the trigger logic, not the placement.
6. **`feat/motion-guards`** — delete `guarded_move_forward.py`; guards become overlay-layer
   consumers of the cursor's state stream, not cursor internals. Note that guards need states
   *before* movement starts, which is what the D7 tee fix in Phase B enables.

---

## 6. Decisions

### Decided

- **D1 — `StartMovementRequest` overrides the previously attached overlay. Confirmed on
  hardware** (§2, D1): a resume that omits `set_outputs` clears it, and an output at exactly
  the resume location re-fires. Consequently every Start carries the full list, `set_outputs`
  is a cursor **constructor default** rather than a per-call argument, and boundary re-firing
  is accepted rather than engineered around.
- **Layering — client-side overlays live above the cursor, not in it.** Server-side overlays
  (`set_outputs`, `start_on_io`, `pause_on_io`) travel as opaque payload on the request
  because they must; markers, async actions, guards and motion events move to a
  `TrajectoryOverlay` layer that drives the cursor through its public API. §3 has the
  reasoning and the import rule that enforces it. Built in Phase D; the *rule* applies
  immediately.
- **Autostart is adapter policy, not a cursor flag.** The cursor gets no `autostart`
  parameter; `move_forward` calls `cursor.forward()` before returning `cursor.cntrl`. The
  cursor bugs this uncovered (D12, D16) are fixed *inside* the cursor, in Phase B.
- **Keep the name `move_forward`**, indefinitely. It is the default `MovementController`;
  renaming buys nothing and breaks callers.
- **`MovementControllerContext` gains an *optional* `joint_trajectory`** rather than moving
  the adapter into `MotionGroup._execute` — keeps the plug-in seam intact, one line at the
  single production construction site, and does not break existing test fixtures (D11).

### Still open

1. **Does `execute()` emit motion events by default?** Recommendation: **no**
   (`emit_motion_events=False` in the adapter). Switching on 5 Hz blinker/NATS emission for
   every production `execute()` is a behavioural and performance change unrelated to this
   merge. Tuning and teaching paths opt in.
2. **Backward travel for client-side overlays.** `set_io` is answered by the server rule
   (D1). Still open from `command-routine.md` Q5: do async actions re-fire on backward
   travel? do markers re-emit? Recommendation: **forward-only** in v1, with a warning logged
   on a backward crossing, until there is a concrete use case. Owner: Phase D.
3. **~~Confirm the server's `set_outputs` behaviour~~ — DONE.** Override semantics and
   boundary re-firing both confirmed on hardware 2026-08-05 (§2, D1). No longer open.
4. **Should `MotionGroup._execute` stop running its own `monitor_motion_group_state()` and
   iterate the cursor instead** (`async for state in cursor`)? It would collapse two
   `stream_state()` subscriptions into one. Recommendation: **not in this transition** — it
   changes which states callers see. Revisit once the Phase B tee fix makes the cursor's
   stream faithful, since that removes the main objection.
5. **What does `detach()` mean once `on_cursor` exposes the cursor to callers?** It cancels
   the operation future and lets `cntrl` return normally (`trajectory_cursor.py:935-948`);
   it sends no pause or abort, so the robot may still be moving. Cancelling the `execute()`
   task instead propagates `CancelledError` and relies on websocket closure. Recommendation:
   document `detach()` as "relinquish control", and if callers need a guaranteed stop, add
   an explicit abort that pauses first — do not let `detach()` acquire that meaning by
   accident.
6. **Ownership and lifecycle between cursor and overlay** (Phase D): if the cursor detaches
   while the overlay is mid-blocking-action, who cancels what? And the overlay's queue needs
   a bounded/drop-oldest policy so a slow overlay cannot grow memory without limit.

---

## 7. Test strategy — the equivalence harness

The highest-value artefact of this work is a test that pins the claim:

```
given the same mocked response stream and the same motion-group state sequence,
  move_forward(context)                                          and
  cursor = TrajectoryCursor(...); cursor.forward(); cursor.cntrl
yield the same sequence of request messages and raise the same exceptions.
```

`tests/cell/test_process_motion_group_state.py` already builds exactly this fixture shape
(a `MovementControllerContext` over a fixed state sequence), so the harness is cheap.
Parameterise it over:

- happy path
- init failure
- movement error
- `pause_on_io`, `start_on_io`
- cancellation mid-flight
- **finite/fast state stream** — measured to lose the `StartMovementRequest` on `main`
  today (D12) and to resolve the operation as a successful full traversal anyway (D16)
- **`actions=[]`** — the D13 preplanned path
- **`set_outputs` present on every Start**, including after a pause→resume and on a
  backward start — the D1 override contract
- **state-stream EOF without a terminal state**, **state-stream exception**, and
  **response-stream EOF** — the D15 cases, where the harness pins the *decided* semantics
  rather than `move_forward`'s current ones
- dropped terminal event (after step 2 of §5)

Once that harness is green, Phase C is mechanical — and the harness stays in the suite
afterwards as the regression guard against a sixth fork of the loop.

---

## 8. Risks

| Risk | Mitigation |
|---|---|
| The cursor is ~7× the code of `move_forward`; making it the default path widens the blast radius of any cursor bug to every `execute()` | The equivalence harness (§7); land Phase C separately so it can be reverted alone |
| Three background tasks + a ticker per execution vs. two today | `emit_motion_events=False` removes the ticker; measure before/after on a long trajectory |
| `_response_consumer`'s ack mis-attribution logic was reasoned about for interactive use, not one-shot runs | Covered by the harness; the one-shot case is strictly simpler (single operation, no stale acks) |
| Four branches must be rebased; each rebase can reintroduce a fork | §5 order, plus a review rule: no new file may send `InitializeMovementRequest` |
| `path_trigger_api_compat` monkey-patches a generated model | Quarantine + removal trigger (§5.4) |
| Divergent bug fixes keep landing on `move_forward` while this is in flight | Land §5 step 2 early — it is the proof that fixes belong in one place |
| "Equivalence" is the wrong goal where `move_forward` is subtly wrong (D15: swallowed state-monitor exceptions) | Decide each stream-lifecycle case explicitly in Phase A and pin it in the harness; call out any intentional behaviour change in the changelog |
| The equivalence harness passes on mocks but the pause/resume `set_outputs` design is wrong on real hardware (D1) | **Retired** — override + boundary re-fire confirmed on hardware 2026-08-05 (§2, D1) |
| The overlay layer is designed but never built, leaving the branch features homeless and the pressure to put them back in the cursor | Phase D is the *only* place the branch features land; the §3 import rule blocks the shortcut in review in the meantime |
| Splitting cursor and overlay adds a lifecycle/ownership surface that did not exist before | §6 open 6; keep the overlay's contract to exactly two cursor APIs (state stream, `pause()`/`forward()`) so the surface stays small |
| D16 (false success) may already be biting production silently — a caller awaiting `forward()` can proceed with the robot unmoved | **Fixed** in Phase B, independently of the merge |
| A stop racing a queued command could command movement after the caller's abort | Pinned by `test_queued_intent_is_not_sent_after_detach`; a stop always wins over a pending intent |
