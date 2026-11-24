# 0010 The built-ins are pipeline modules, and the list is everything

Status: accepted, 2026-11-22. Supersedes the "opt-in, none by default" part of 0009.

**Context.** After 0009 the send pipeline held only the opt-in `LiveBuffer` and `EaseIn`. Six
built-ins still shaped what was sent from four places, each with its own switch: the queue
clients smoothed their plans (`smoothing_passes`, `async_queue_smoothing`), the executor bridged,
ramped and clamped (`SequentialExecution(endpoint_ramp=...)`, `clamp_to_joint_limits`), and the
session padded and trimmed (`WaypointConfig.min_chunk_horizon_ms`, always). Nobody could see the
whole chain in one place, or leave one step out without a dedicated switch.

**Decision.** Each built-in is a module of the one list:

    [Smoothing] [Bridge] [EndpointRamp] [JointClamp] [LiveBuffer] [HoldPadding]
        -> Timestamps() -> [DropLate] -> [EaseIn] -> sender

- A third module kind, the **filter** (`keep`), runs after `Timestamps()` and before the waypoint
  modules. It may drop waypoints but never retime them, which is what dropping late steps (T3)
  needs. The final target of a run passes every filter.
- **The list is exactly what runs.** `pipeline=None` runs `default_pipeline()`, which reproduces
  the old behaviour for the joggers and for each executor mode (contracts T13); an explicit list
  replaces it. To add a module, extend the defaults: `[LiveBuffer(window_ms=500),
  *default_pipeline()]`.
- Settings move into their modules: `HoldPadding(min_horizon_ms)`, `JointClamp(margin_rad)`,
  `EndpointRamp(interpolation_steps)`, `Smoothing(passes)`, `Bridge(always_anchor)`,
  `DropLate(min_lead_ms)`.
- What a module needs travels with the chunk: `start_step` (where the policy's first step is
  after a bridge or a ramp, which a timestep queue anchors its timeline on) and
  `retained_prefix_steps` (the steps a queue already sent, which smoothing keeps exact). The
  session reads joint limits at its start when a `JointClamp` is in its pipeline.
- A timestep queue in continuous execution without its `TimestepTimeline` is refused at
  construction: every chunk would restart at "now" (since ADR 0011 the queue brings it, with its
  `Smoothing` and `Bridge`, as `default_modules`). Under sequential execution a timestep queue
  and a `LiveBuffer` are refused: it waits for standstill and observes again, so nothing replaces
  a plan while it runs, and either would be fed timings that never come (contracts T12).

**Verified.** A capture of what reaches NOVA per scenario (sequential with bridge and ramp,
continuous one-step and multi-step, joint limits, IO at the first step, a timestep queue, jog
targets and chunks) stayed byte-identical through every move but one: trimmed chunks no longer
rebase their times on the first kept step, so some waypoints of a timestep queue moved by +1 ms
onto `anchor + int(k·dt)`. LeRobot's async queue against a fake policy sends what it sent before
to 1e-6 rad, the float32 rounding its queue stored smoothed values in. On the virtual robot
(at `626dda5b`): jogging 0.835° without a buffer, 0.015° with `LiveBuffer(500)`, `EaseIn` and
dual-arm without errors, as in 0009; sequential execution 156 of 156 policy steps sent, IO on the
policy's first step; VLA over NATS continuous and sequential; a `QueuedPolicyClient` anchored its
timeline on its first bridged chunk.

**Consequences.** Every step between the policy and NOVA is one item in one list, visible,
replaceable and testable on its own; new shaping is a module, not a flag. Breaking: the six
switches are gone, and a caller's `pipeline=[LiveBuffer(...)]` now runs only the buffer (no
padding, no dropping of late steps) unless it extends the defaults (migration note). Smoothing a
LeRobot plan now happens on the resolved targets, which differs from before for relative actions
and rotation conversions. A `QueuedPolicyClient` step that was frozen and becomes replaceable
after a latency drop is blended from its unsmoothed value. Rerun's action plot and pre-send stop
conditions see a queue's plan unsmoothed. A LeRobot async queue run with an `n_action_steps` below
its plan length (the checkpoint's value, unless `n_action_steps=0` is passed as its README says) is
trimmed before it is smoothed, so its last kept steps are filtered against a replicated end
instead of the steps after them.

**Why each module does what it does** (recorded 2026-11-23 from the module docstrings).
`HoldPadding` gives a short chunk a horizon the controller can brake within; it skips one-step
chunks placed ahead, which the next step replaces before they are due and which a hold would make
the controller brake towards, and it ends a chunk in a hold, which leaves
`delay_until_braking_ms` no velocity to keep. `JointClamp`: see ADR 0015. `EndpointRamp` adds
points at the chunk's own spacing, so the robot gets real time to accelerate and brake.
`Smoothing` with two passes removes the waypoint-scale kinks where a queue blends overlapping
predictions, at the cost of rounding intended corners by about two steps; it goes first because
the bridge has to end on the smoothed first step. `Bridge` starts with a hold on the measured
position, so sending the request does not use up the first interval; `always_anchor` gives a
continuously refreshed queue one full step before its first target; rotations are unwrapped
first, since interpolating rotation vectors between `v` and `-v` (a tool pointing down, near π)
passes through the identity, a real half-turn of the wrist. `DropLate`: a chunk is stamped when
it is queued and sent a little later, so its first waypoints can already be due; when all are
late, the robot keeps following the previous chunk. Leave it out only to see late waypoints on
the wire.

**Sequential chunks end in a hold (2026-11-23).** `HoldPadding` pads up to a total length, so a
sequential chunk of 15 steps or more at 30 Hz got no hold at all. When the arm was still swinging
at the last waypoint (behind after a sharp bridge-to-policy turn), NOVA braked from that speed and
left it 0.3–0.5° from the final target: it brakes after the last waypoint and does not return.
Sequential execution therefore holds every chunk's final target `SEQUENTIAL_HOLD_MS` (300 ms) past
its last step (`HoldPadding(after_ms=...)`). On a virtual UR10e, a 200 ms hold with `EndpointRamp`
left at most 0.05° (without the ramp, 0.68°; 100 ms, 0.26°). Each sequential cycle gets up to
300 ms longer.
