# End-of-execution detection via MotionGroupState — research briefing

**Status:** open problem, partially mitigated in two places.
**Written:** 2026-08-12, from measurements on the ARG3 virtual cell and the production IPC.
**Purpose:** seed a research session on how execution and completion detection *should* be
architected. This document is the input; it is not a decision.

---

## 1. Why this exists

Running 12 KUKA controllers (14 motion groups) in parallel, executions intermittently never
complete: `execute()` blocks forever after the robot has physically stopped. The suspicion was that
the NOVA Python SDK is the culprit, so a second app (`arg3-api`) was built that performs the same
motions through `wandelbots-api-client` only, with no SDK, as a control group.

Measurements since then have narrowed the problem to one specific mechanism, described in §3. The
question now is **not** "is it broken" but **what is the right architecture for consuming
MotionGroupState to decide that a motion has ended**, given how the controller publishes state.

Two apps, same cell, same motions, deliberately pinned to the same `wandelbots-api-client` and
`websockets` versions so a behavioural difference can only come from the SDK layer:

| | app | package | approach |
|---|---|---|---|
| SDK | `audi-arg3-sdk` | `audi_arg3` | NOVA Python SDK + NOVAx |
| API | `audi-arg3-api` | `audi_arg3_api` | `wandelbots-api-client` only |

---

## 2. How execution actually works

The essential asymmetry, and the root of the whole problem:

**The execution websocket never reports that a motion finished.**

`trajectory_execution_api.execute_trajectory` is a websocket driven by a request generator. You send
`InitializeMovementRequest` (trajectory inline as `TrajectoryData`, or by id), await its
acknowledgement, then send `StartMovementRequest`. Every response is an *acknowledgement*:

```
ExecuteTrajectoryResponse = InitializeMovementResponse
                          | StartMovementResponse
                          | PauseMovementResponse
                          | PlaybackSpeedResponse
                          | MovementErrorResponse
```

There is no "completed" message. The API documentation is explicit about the consequence:

> To monitor the state of the movement, listen to the state stream (`streamMotionGroupState`). The
> state is published via nats as well. Field `execute` in the `MotionGroupState` indicates whether a
> movement is ongoing and carries execution details.

So completion is **observed**, never **delivered**. Every architecture below is a different way of
observing it.

Two related traps found the hard way, both worth keeping in any design:

- **Acknowledgements carry failures.** A successful-looking `INITIALIZE_RECEIVED` came back with
  `add_trajectory_error: "Provided TCP ID '1' is not configured on controller"`. Because the ack was
  awaited but not inspected, `StartMovementRequest` was rejected with "not initialized yet", nothing
  moved, and it presented as an unexplained completion timeout.
- **`StartMovementRequest` can gate server-side.** It carries `start_on_io`, `pause_on_io` and
  `set_outputs`. NOVA can therefore start/pause a motion on an IO condition without the client being
  in the loop at all. Neither app uses this today; see architecture D.

---

## 3. The core defect: the terminal state exists for one controller step

Measured on the virtual cell, streaming `stream_motion_group_state` at **maximum rate** across one
~4.5 s motion (1119 trajectory points):

| frames observed | count |
|---|---|
| total | **3101** |
| `TrajectoryRunning` | 1121 |
| no `execute` block | 1979 |
| **`TrajectoryEnded`** | **1** |

`TrajectoryEnded` appears in **exactly one frame**. Repeating the same run throttled to
`response_rate=200` never observed it at all.

The arithmetic behind that, confirmed on the production robots:

- `cycle_time` = **4 ms** on both `KUKA_KR270_R2700` and `KUKA_KR240_R2900`. NOVA updates state at the
  controller's step rate.
- The terminal state therefore exists for roughly **one 4 ms step**.
- Streaming at step rate (~280 frames/s) you normally see it — once.
- **Anything that costs you that single frame loses it permanently.** Fourteen concurrent state
  streams on one event loop, a coalesced or dropped websocket frame, GC, network jitter, a slow
  consumer callback — any of these, once, and the frame is gone. There is no repeat and no re-query
  that reconstructs it.

This is the mechanism behind "executions sometimes never complete", and it explains the observed
shape: intermittent, load-correlated, unreproducible on a single robot.

**On production, at 200 ms sampling, `arg3-api` observed the terminal frame in 0 of 24 consecutive
cycles.** Every cycle completed through the fallback described in §6.

---

## 4. The state-rate dimension

`stream_motion_group_state(cell, controller, motion_group, response_rate=...)`. From the generated
client's own parameter documentation:

> Update rate for the response message in milliseconds (ms). **Default is 200 ms.** We recommend to
> use the step rate of the controller or a multiple of the step rate as NOVA updates the state in the
> controller's step rate as well. Minimal response rate is the step rate of controller.

Consequences to weigh in any design:

- **The API's default rate cannot reliably detect completion.** A 4 ms state inside a 200 ms sampling
  window is caught roughly 2% of the time. Any design that samples must not depend on catching a
  single-step state.
- **The SDK streams at step rate** (`stream_state()` with `response_rate=None`), which is why it
  mostly works — and why it fails intermittently rather than always.
- **Step-rate streaming does not scale linearly.** 14 motion groups × ~280 frames/s ≈ 3900
  frames/s, every one parsed into a pydantic `MotionGroupState`, on one event loop. `arg3-api`
  throttles to 200 ms deliberately (`STATE_RESPONSE_RATE_MS`) precisely because full-rate streaming
  for 14 motion groups is itself a suspect — but that throttling makes the terminal frame
  unobservable, so it must not be the completion signal.

There is a genuine tension here: **the rate you need to catch the terminal state is the rate that
creates the load you are trying to avoid.** Resolving that tension is the point of the research.

An unexplored input: NATS carries `nova.v2.cells.{cell}.controllers.{name}.state`. That is
**per controller**, not per motion group — 12 subscriptions instead of 14 websockets for this cell,
with fan-out handled by the broker rather than by N client-side streams. Its rate, delivery
guarantees and whether it carries the same `execute.details` are unverified.

---

## 5. The SDK's execution state machine

`nova/cell/movement_controller/trajectory_state_machine.py` — `TrajectoryExecutionMachine`, a
`python-statemachine` FSM fed one `MotionGroupState` at a time via `process_motion_state()`.

```
┌──────┐  start   ┌───────────┐
│ idle │─────────→│ executing │←───────────────────┐
└──────┘          └─────┬─────┘                    │
                        │                          │
           ┌────────────┼────────────┐          resume
        ended+ss     ended       paused+ss         │
           │         (no ss)        │              │
           ▼            │           ▼              │
     ┌───────────┐      │     ┌─────────┐          │
     │  ended    │      │     │ paused  │──────────┘
     └───────────┘      │     └─────────┘
           ▲            ▼
           │      ┌──────────┐
           └──ss──│  ending  │        ss = standstill
                  └──────────┘
```

The completion path requires **two** observations:

1. a frame whose `execute.details.state` is `TrajectoryEnded` — this is the single-step state from §3.
   With standstill already true it goes straight to `ended`; otherwise it enters `ending`;
2. then a frame with `standstill` true to leave `ending` for `ended`.

Miss observation 1 and the machine stays in `executing` forever. No later frame can rescue it,
because the discriminator it is waiting for has already gone.

**The load-bearing assumption, as shipped in the pinned SDK** (`process_motion_state`, the
`if not has_execute:` branch):

```python
# No execute info — skip.  The API guarantees that once execute is
# set it will remain present in subsequent states, so a bare
# standstill (without execute) is not a reliable completion signal.
return StateUpdate(has_execute=False, state_changed=False, ...)
```

Frames without an `execute` block are discarded outright. **That assumption is wrong**, and the
watchdog branch says so explicitly (§6): the controller drops the `execute` block the instant the
robot settles, so for a run whose `TrajectoryEnded` frame was lost, the bare standstill frames that
follow are the only completion signal that will ever arrive — and the shipped machine throws them
away. In the §3 measurement, 1979 of 3101 frames were exactly these discarded frames.

---

## 6. The two existing mitigations

### 6a. SDK branch `fix/trajectory-completion-standstill-watchdog`

Local checkout: `/Users/q/0code/nova-python-sdk`, also on `origin`. Head commit:

> `313a523b` fix: complete trajectories on standstill when terminal event is dropped

Diff vs the pinned branch (`feat/NDX-155-simple-custom-actions`): 10 files, +563/−36, including a new
`nova/cell/movement_controller/stall_watchdog.py` (127 lines), a new `MovementStalled` exception, and
tests with a captured `frames_swallow.json` fixture. Related upstream: PR #426.

It makes two distinct changes.

**Change A — honour bare standstill in the state machine.** The `if not has_execute:` branch no
longer discards the frame. When the machine is already in `ending` or `pausing`, a bare standstill
completes the transition, on the reasoning that the discriminator was already seen on the way into
that state. Its comment replaces the old assumption:

> The controller drops the trajectory `execute` block the instant the robot settles
> (robotics/wbr RAEv2_ProtoRobotState), so a bare standstill is the only completion signal we may
> ever receive.

This fixes the case where `TrajectoryEnded` *was* seen but the following standstill frame carried no
`execute` block. It does **not** help when `TrajectoryEnded` itself was never observed — the machine
is then still in `executing`, not `ending`.

**Change B — a standstill watchdog** (`run_standstill_watchdog`), for exactly that uncovered case.
A `StandstillObservation` is written by the controller's state monitor and polled by the watchdog:

- ignores standstill that is **legitimate by design** — `TrajectoryWaitForIO`,
  `TrajectoryPausedOnIO`, `TrajectoryPausedByUser`;
- after `stall_timeout_s` at an unexplained standstill, if the joints are within
  `at_target_tolerance` of the **planned final joint position**, it returns → motion complete;
- past `max_stall_s` still not at target, it raises `MovementStalled` — a hard ceiling so `execute()`
  can never hang forever, covering an ambiguous stop-short (e.g. a real protective stop) that is
  otherwise indistinguishable from a legitimate in-path dwell.

Defaults (`nova/actions/container.py`): `stall_timeout_s = 2.0`, `max_stall_s = 30.0`,
`at_target_tolerance = 1e-2`. Poll interval `min(0.1, max(stall_timeout_s/4, 0.01))`.

Design properties worth preserving in any alternative: it **never fabricates completion** (at-target
is verified against the planned target, not assumed) and **never cancels in-flight motion** (it only
acts once the robot is already at standstill). Known gap, called out in its own docstring:
`TrajectoryCursor` shares the same state machine and therefore the same latent hang; wiring the
watchdog in there is deliberately deferred.

### 6b. What `arg3-api` does today

`audi_arg3_api/motion.py`, `_watch_for_completion` — deliberately minimal, and weaker than 6a:

- primary: `TrajectoryEnded` **and** `standstill` → complete;
- fallback: **`execute` block cleared while at standstill, after movement was observed** → complete;
- backstop: `MOTION_TIMEOUT_S` (default 300 s) → fail.

On production this ran 24 consecutive cycles at 4.60–4.80 s, **every one of them completing via the
fallback**, never the primary. That is the §3 finding reproduced on real hardware.

Its weakness, and a specific thing to evaluate: unlike 6a it does **not** verify the robot is at the
planned target, and it does not classify legitimate waits. A motion that stops short and has its
`execute` block cleared would be reported as a successful completion. `arg3-api` gets away with it
today only because its motion has no IO waits or dwells mid-path.

---

## 7. Open questions

1. **Is the single-step terminal state intended?** Is `TrajectoryEnded` guaranteed to be published
   for at least one step, or can it be skipped entirely by the controller? If it can be skipped, no
   consumer-side rate is ever sufficient and Change B (or equivalent) is mandatory, not a backstop.
2. **When exactly is the `execute` block dropped**, relative to the terminal state and to standstill?
   Change A's comment cites `robotics/wbr RAEv2_ProtoRobotState`. The ordering guarantee here decides
   whether "bare standstill" is a sound completion signal or a race.
3. **Does NATS `…controllers.{name}.state` carry the same `execute.details`**, at what rate, and with
   what delivery semantics? At-most-once (as documented for bus IO) would make it strictly worse for
   a single-step state; a retained/last-value semantic would make it strictly better.
4. **What does step-rate streaming for 14 motion groups actually cost** — client CPU, event-loop
   latency, gateway load? This is the number that decides whether "just stream at step rate" is a
   real option. Untested: nothing has yet run both apps side by side under load.
5. **Can completion be made pull-based?** Is there any endpoint that answers "is trajectory X still
   executing" authoritatively, so completion could be polled at a low rate instead of caught in
   flight?
6. **Should the client be in the loop at all?** With `start_on_io` / `pause_on_io` /
   `set_outputs`, the cycle could be gated entirely server-side.

---

## 8. Candidate architectures to evaluate

Not exhaustive, and not mutually exclusive.

| | approach | catches single-step state? | load | notes |
|---|---|---|---|---|
| **A** | step-rate websocket per motion group + FSM (SDK today) | usually | highest — 14 streams, ~3900 frames/s | the intermittent failure mode of §3 |
| **B** | A + standstill watchdog (branch 6a) | not needed | highest | proven design; verifies at-target; hard ceiling |
| **C** | throttled stream + at-target confirmation | no, by design | low | needs the target-verification of B to be safe; closest to `arg3-api` today, minus its weakness |
| **D** | server-side IO gating (`start_on_io`, `set_outputs`) | n/a | lowest | robot signals its own completion over IO; changes the app's contract, diverges from the SDK app |
| **E** | NATS controller-state subscription | unknown — §7.3 | 12 subs, broker fan-out | needs the §7.3 answers before it can be judged |
| **F** | low-rate poll of `get_current_motion_group_state` + at-target | no | 14 × 1/s REST | simplest to reason about; needs §7.5 |

Evaluation criteria worth agreeing on up front: does it ever **fabricate** a completion; does it ever
**hang**; does it distinguish a legitimate dwell from a stall; what does it cost at 14 motion groups;
and how far does it drift from the SDK app (which matters only while the two are being compared).

---

## 9. Reproducing the evidence

Everything below is read-only except the deliberate motion runs.

```bash
# virtual cell (safe): provision 12 controllers mirroring production topology
cd /Users/q/1prj/audi/arg3-api
.venv/bin/python scripts/provision_virtual_cell.py --host http://<virtual-host>   # --teardown to undo

# frame census over one motion: count TrajectoryRunning / TrajectoryEnded / no-execute
# stream with response_rate=None (step rate) vs 200 and compare
# see git log for the exact probe scripts used
```

Key source references:

- `audi_arg3_api/motion.py` — `_watch_for_completion`, `_is_ended`, `execute_trajectory`,
  `_initialize_error`
- `audi_arg3_api/config.py` — `STATE_RESPONSE_RATE_MS`, `MOTION_TIMEOUT_S`
- SDK, pinned: `nova/cell/movement_controller/trajectory_state_machine.py`,
  `nova/cell/movement_controller/move_forward.py`
- SDK, branch `fix/trajectory-completion-standstill-watchdog`:
  `nova/cell/movement_controller/stall_watchdog.py`, `nova/actions/container.py`,
  `tests/cell/test_movement_controller_stall.py`, `tests/cell/fixtures/frames_swallow.json`
- `arg3-sdk/audi_arg3/move_forward.py` — an instrumented copy of the SDK controller with phase
  logging and a diagnostic standstill watchdog, built for observing this exact bug
- Production log showing 24/24 fallback completions: `~/Downloads/ir360r02.log`

---

## 10. Constraints and safety

- **A production IPC has physical robots attached.** The cell is served by more than one IPC and
  which is live changes, so no address is recorded here on purpose -- take it from `.env` /
  `nova config view` and confirm with `GET {basePath}/api/health` before doing anything.
  **Know which kind of instance you are on before you act:** the same call is harmless on a virtual
  instance and moves 150 kg of tooling on a production one. Read-only API calls are fine;
  anything that sets a mode or moves a robot needs someone at the cell. Robots move +100 mm in Z and
  back, gated on the `start` bus IO.
- **Virtual instances are typically reached over WAN.** Safe to move, but latency there is not
  representative of the LAN/IPC setup — slow round trips are not evidence of a load problem. Its
  virtual controllers have only one motion group each (12, not 14) and no TCP `"1"`.
- **Payload matters for planning.** `MotionGroupSetup.payload` feeds the planner's dynamics; the tools
  on this cell are 68–151 kg. Planning unloaded caused `Sollmotormoment` (motor torque) faults in
  production. Any experiment that plans its own trajectories must resolve the payload — see
  `resolve_payload()`.
- After a torque fault a controller needs **manual intervention at the panel**; it will not recover on
  its own, and re-running just re-faults.
- `ir350r01` was reported disconnected during the last read (`connected:false`).
