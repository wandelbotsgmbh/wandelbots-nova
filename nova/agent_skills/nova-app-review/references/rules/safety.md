# Safety (SAF)

## SAF-001 · Return to a defined safe pose on every exit path
`severity: error` · `scope: robotics` · `detect: static+review`

**Rule.** Every job that moves the robot must declare its safe/home pose **before** the first motion
and return to it in a `finally` (or an equivalent central wrapper). The chosen pose must be
published to the rest of the system so that homing and home-checks use the same pose.

**Why.** A job that raises mid-sequence leaves the robot inside a fixture or above a conveyor. The
next job, or the operator, starts from an unknown pose.

**Detect.**
- Functions that call `plan_and_execute` / `execute` / `move` with no `try/finally` containing a
  motion to a home pose.
- Home pose chosen *after* the first motion, or chosen dynamically from runtime data.
- Home pose not stored anywhere shared (homing routine would use a different pose).

```python
# BAD
async def run(ctx):
    await mg.plan_and_execute(pick_actions, tcp=TCP)
    await gripper.close()  # raises -> robot left at the part
    await mg.plan_and_execute(place_actions, tcp=TCP)


# GOOD
async def run(ctx):
    home = dataset.home_pick.pose
    state.current_home_pose = home  # homing + home-check use the same pose
    try:
        await mg.plan_and_execute(pick_actions, tcp=TCP)
        await gripper.close()
        await mg.plan_and_execute(place_actions, tcp=TCP)
    finally:
        if not state.fault_active:
            await mg.plan_and_execute([joint_ptp(home)], tcp=TCP)
```

**Exceptions.** On a hard fault the `finally` motion must **not** run (see SAF-003); gate it on
"no active fault".

---

## SAF-002 · Re-validate preconditions immediately before motion
`severity: error` · `scope: robotics` · `detect: static+review`

**Rule.** Use a two-stage gate: (1) select the job from a polled snapshot, (2) re-read **live** IO
inside the job and re-check all preconditions immediately before the first motion. Precondition
lists must be declared once and reused by both stages.

**Why.** Snapshots are stale by at least one poll interval plus queue latency. The PLC can withdraw
"part ready" or a safety door can open in between.

**Detect.**
- Job reads cached snapshot values for go/no-go decisions without a fresh read.
- Precondition logic duplicated between dispatcher and job (drift risk).
- First motion happens before any `ensure_conditions()`-style call.

```python
PRECONDITIONS = [Cond("In.App.Tray1.PickRdy", True), Cond("In.Sys.FaultActive", False)]


async def run(ctx):
    await ensure_plc_conditions(PRECONDITIONS)  # live read, raises on mismatch
    await mg.plan_and_execute(actions, tcp=TCP)
```

NOVA: `nova.program.ProgramPreconditions` covers controller/dataset setup only, not PLC state.
Read live bus IO with `nova.utils.io.get_bus_io_value([...])` (one call, see IO-003).

---

## SAF-003 · Fault path stops motion first, with the shortest timeout
`severity: error` · `scope: robotics` · `detect: review`

**Rule.** On fault: (1) stop motion, bounded by the **shortest** timeout in the system, (2) cancel
the active motion task, (3) only then perform other cleanup. The stop must not wait behind other
awaits. Timeout hierarchy: `stop < plan < execute`.

**Observed.** stop 5 s, plan 20 s, execute 90 s.

**Detect.**
- Fault handler awaits logging, IO writes or plugin cleanup **before** stopping motion.
- Stop without `asyncio.timeout` / `asyncio.wait_for`.
- Stop and other cleanup combined in `gather()` where a failure of one cancels the other (use
  `return_exceptions=True` or sequence them).
- No cancellation of the in-flight execute task after stop.

NOVA: see `nova-sdk-mapping.md` (SAF-003). `MotionGroup.stop()` is **not functional** in the
current SDK; stop by cancelling the execute task or `TrajectoryCursor.pause()`.

---

## SAF-004 · Drain pending work on fault; never auto-resume stale jobs
`severity: error` · `scope: robotics` · `detect: review`

**Rule.** On fault, stop accepting jobs, drain the job queue and discard queued work. After recovery
the dispatcher re-evaluates from fresh inputs.

**Detect.** Fault handler does not clear the queue; queue has `maxsize` but no drain; jobs carry
snapshots captured before the fault and are executed after reset.

---

## SAF-005 · No automatic recovery from planning/reachability failures
`severity: error` · `scope: robotics` · `detect: static+review`

**Rule.** Planning failures (unreachable target, no IK solution, collision, singularity) must latch
a `manual_recovery_required` state. Leave it only after an explicit operator reset, ideally after
the robot was moved via teach pendant. Do not retry in a loop.

**Detect.**
- `except PlanTrajectoryFailed` / `NoInverseKinematicsSolutionFound` followed by retry,
  `continue`, or an alternative target.
- Planning errors treated identically to transient network errors.

```python
from nova.exceptions import PlanTrajectoryFailed

try:
    traj = await mg.plan(actions, tcp=TCP)
except PlanTrajectoryFailed as e:
    raise TrajectoryPlanningError(
        e.to_pretty_string(),
        fix_tip="Move the robot with the teach pendant to a valid pose, then send ResetFault.",
        manual_recovery_required=True,
    ) from e
```

---

## SAF-006 · Fault reset requires edge + cleared cause + interlocks
`severity: error` · `scope: robotics` · `detect: review`

**Rule.** Leave FAULT only when **all** of these hold: a rising edge on the reset input, the
original fault source is cleared, interlocks are OK, and any manual-recovery latch is released.

**Detect.** Reset on level; reset that ignores `fault_active`; reset path that does not consult the
interlock function; automatic transition `FAULT → READY` on a timer.

---

## SAF-007 · Start automatic cycle only from a verified safe state
`severity: error` · `scope: robotics` · `detect: review`

**Rule.** Enter automatic operation only on a start edge **and** automatic mode **and** robot in
home **and** no fault. Leave automatic immediately when mode or enable drops. Offer an explicit
"finish cycle, then stop" signal instead of stopping mid-motion when that is the safer choice.

---

## SAF-008 · Command inputs are edge-triggered, not level-triggered
`severity: error` · `scope: robotics` · `detect: static+review`

**Rule.** Start, reset, home and acknowledge inputs trigger on a rising edge with the previous value
latched per loop iteration. Delayed actions (e.g. "home after 3 s") must re-check conditions when
the delay expires and be cancellable by mode/fault changes.

**Detect.** `if signals.reset_fault:` without `and not self._last_reset`; edge memory updated in
some branches but not others; delayed tasks that do not re-validate.

NOVA: `nova.utils.io.wait_for_bus_io` passes `IOChange(old_value, new_value)` to `on_change`, so a
rising edge is `old_value is False and new_value is True`. The first call has `old_value=None`;
do not treat it as an edge.

---

## SAF-009 · Outputs go to safe defaults on start, fault and shutdown
`severity: error` · `scope: robotics` · `detect: review`

**Rule.** All robot→PLC handshake and status outputs have a defined default (normally `False`/0).
Write defaults at startup before signalling readiness, on fault entry, and as the last IO action on
shutdown.

**Why.** A stale `PickDone=True` left from a crashed process can make the PLC advance its sequence.

NOVA: write all defaults in one `nova.utils.io.set_bus_io_value({...})` call.

---

## SAF-010 · Simulation bypasses must be double-gated and loud
`severity: error` · `scope: robotics` · `detect: static`

**Rule.** Any code path that skips IO, preconditions or device checks must require **both** a
per-call/job opt-in **and** a global simulation flag for that subsystem, and must log a warning at
startup.

**Detect.** `if VIRTUAL:` or `ignore_checks` guarding a skip of preconditions with a single
condition; simulation flags defaulting to `True` in production configs.

```python
ignore_plc_checks = job.ignore_plc_checks and VIRTUAL_PLC  # GOOD: both required
```

---

## SAF-011 · Simulation must not silently change motion semantics
`severity: warning` · `scope: robotics` · `detect: static+review`

**Rule.** If simulation mode removes a motion feature (force stop-on-contact, pause-on-IO, collision
model), it must log that explicitly and tests must cover the real path separately.

**Observed.** A force-guided move dropped its `pause_on_io` stop condition in virtual mode, so the
virtual test passed on a path that would crash into the part on hardware.

---

## SAF-012 · Stubbed safety checks must be visible
`severity: warning` · `scope: robotics` · `detect: static`

**Rule.** A function named like `safety_ok`, `interlocks_ok` or `is_safe` that unconditionally
returns `True` must log a startup warning and carry a tracked TODO. Never let a stub look like a
real check.

**Detect.** `def .*(safe|interlock).*:\n\s*return True`.

---

## SAF-013 · Reduced-speed mode exists and is explicit
`severity: warning` · `scope: robotics` · `detect: static+review`

**Rule.** Provide a commissioning mode that scales all speed profiles down (e.g. ÷20). Log the
active scale at startup. Production must disable it explicitly; do not rely on a default.

NOVA: scale the named `MotionSettings` profiles (`tcp_velocity_limit`, `joint_velocity_limits`, …)
in one place; `MotionSettings` is immutable, so build scaled copies with `model_copy(update=...)`.

---

## SAF-014 · Home/pose checks use wrapped angles and per-joint tolerance
`severity: warning` · `scope: robotics` · `detect: static`

**Rule.** Compare joint positions with wrap-around (`(d + π) % 2π − π`) and allow per-joint
tolerance. Comparing Cartesian poses for "in home" fails near configuration flips.

NOVA: read joints with `await mg.joints()` (radians).
