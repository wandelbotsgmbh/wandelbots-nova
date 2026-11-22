# Migration

novapolicy is an experimental extra, so its API can still change between releases. This page
lists the changes that need an edit, or that change what an unchanged program does.

## Step rate: `control_freq`

Every client takes the policy's step rate as `control_freq=` in Hz: the training data's frame rate
(`fps` in a LeRobot dataset's `meta/info.json`) or a reinforcement-learning policy's control frequency. It is not
`ContinuousExecution(rate_hz=...)`, which sets how often the policy is asked.

| Before                                       | Now                                                |
| -------------------------------------------- | -------------------------------------------------- |
| `Gr00tPolicyClient(..., dt_ms=50.0)`         | `Gr00tPolicyClient(..., control_freq=20)`          |
| `LeRobotPolicyClient(..., fps=15)`           | `LeRobotPolicyClient(..., control_freq=15)`        |
| `LeRobotPolicyClient(..., playback_speed=x)` | `control_freq=` the dataset's `fps`                |
| `CallbackPolicyClient(fn)`                   | unchanged; `control_freq=` fills chunks without `dt_ms` |

- **The LeRobot default changed from 15 to 30.** A policy recorded at 15 fps that relied on the
  default would now play twice as fast: pass `control_freq=15`.
- The GR00T default is 30 Hz (33.3 ms per step, was 33 ms). A `dt_ms` the server reports still
  wins.
- `dt_ms=`, `fps=` and `playback_speed=` still work, with a `DeprecationWarning`.

## Relative TCP rotations are composed

`Action.tcp(..., mode="relative")` used to add the rotation offset to the current rotation vector
component by component. It now turns the tool about the TCP by the offset, composed as rotations.
Positions are unchanged. For small rotations away from a tool-down pose the result is nearly the
same; near a tool-down pose (rotation vectors close to π) the old sum pointed the tool in the wrong
direction. Nothing to change in code; a policy that relied on the old sum turns differently.

New options on the same action: `accumulate=True` for per-step deltas, and
`use_tool_coordinate_system=True` for offsets in the tool coordinate system (see
[schema.md](schema.md)).

## Pre-send stop conditions see the steps that run

At inference time, `StopContext.target_joints` and `target_ios` now hold only the
`n_action_steps` steps that will be sent, not the whole predicted chunk. A stop condition that
looked at later steps of the horizon no longer sees them.

## GR00T docs moved

`novapolicy/gr00t/README.md` is now [docs/gr00t.md](gr00t.md), together with the guide for setting
up a GR00T policy on a real robot.

## `PolicyExecutor(plant=...)` is removed

The executor no longer simulates a plant between a policy and the robot: it maps observations,
runs the policy and sends its actions to NOVA, and NOVA moves the robot. `novapolicy.plant`
(`VirtualPlant`) and the `plant=` argument are gone; no shipped plant existed. Drop the argument.

## The start move keeps the cell's collision setups

`PolicyExecutor(start_joint_position=...)` used to clear the motion group's collision setups for
the move to the start pose. That move is now planned with them, like any other move: a start pose
inside a collision zone fails to plan instead of being reached without collision checks.

## Continuous chunks are dated by their observation

In `ContinuousExecution`, a chunk the policy returns for the latest observation is now dated like
a one-step target: step `k` is due `WaypointConfig.max_expected_latency_ms + k·dt` after the
moment it was observed (on the server's clock, 100 ms by default), and steps already too close to
send are dropped. Before, step 0
was placed at server "now" minus the inference time, rounded to whole steps. Chunk policies
(GR00T, LeRobot without the async queue, NATS) therefore run about `max_expected_latency_ms`
later than before, and lose no head steps while the latency stays below it. Chunks on their own
timeline keep the old placement: RTC (`seam_backdate_steps`), the timestep queues
(`QueuedPolicyClient`, the LeRobot async queue) and bridged chunks.


## GR00T applies the schema's joint operators

`Gr00tPolicyClient` used to send joint positions to the server in radians even when the schema
declared `ops` (`Observation.joint_positions(..., ops=[Rad2Deg()])`), while the executor still
inverted those ops on the returned actions. It now applies them on the way in, as every other
client does. A GR00T schema without `ops` is unaffected.

## Sequential execution refuses setups it would run wrongly

`SequentialExecution` waits for standstill before it observes again, so nothing replaces a
running plan. `PolicyExecutor` now raises `ValueError` at construction for the setups that
depend on that, instead of running them with timings that never come:

| Before (ran, wrongly) | Now |
| --- | --- |
| `QueuedPolicyClient(...)` or `LeRobotPolicyClient(use_async_queue=True)` with `SequentialExecution()` | use `ContinuousExecution()`, or the wrapped policy without the queue |
| `live_buffer=...` with `SequentialExecution()` | use `ContinuousExecution()` |

A chunk with its own `first_timestamp_ms` is no longer bridged from the measured pose: the bridge
rebuilt the chunk without that timestamp. It is sent at the time the policy set.

## IO and computed actions run when the policy's first step is due

A chunk's IO (`ActionChunk.ios`) and its `Action.computed` hooks now run when the policy's first
step is due on the server's clock, in both modes and on every path, instead of at send (most
paths) or after the loop blocked until that moment (chunks with a bridge in front). The loop no
longer waits for them. A chunk replaced before it was sent runs neither (its motion never ran);
one still pending when the run stops is cancelled; a hook that raises ends the run. NOVA's action
chunks carry no IO, so the write reaches the controller one network delay after that moment.

## One placement rule for both modes

Step `k` of a chunk is now due `WaypointConfig.max_expected_latency_ms + k·dt` after a reference
in both modes: the observation in `ContinuousExecution`, the send in `SequentialExecution` (the
robot was at rest when it was observed). Sequential chunks used to start one `dt` after the send;
they now start one lead later (100 ms by default), and keep their hold padding. Chunks with their
own `first_timestamp_ms` are unchanged.

## GR00T IO comes from the chunk's first step

`Gr00tPolicyClient` used to take an IO action's value from the last step of the chunk; it now
takes the first, like the LeRobot client, since IO is written when that step is due. A policy
that toggles a gripper inside a chunk now has it follow the first step of each chunk.

## `PolicyExecutor(buffer_window_ms=...)` is now `live_buffer=LiveBuffer(...)`

| Before | Now |
| --- | --- |
| `PolicyExecutor(..., buffer_window_ms=80)` | `PolicyExecutor(..., live_buffer=LiveBuffer(window_ms=80))` |

The live-target buffer is one module now (`novapolicy.LiveBuffer`), shared with the joggers. For a
one-step policy it measured worse than the default lead path; see `docs/adr/0004`.

## Jogging: no live buffer and no ease-in unless passed

`jog_joints` and `jog_tcp` no longer buffer live targets by default, and `ease_in_s` and
`buffer_window_ms` are gone. Both are opt-in modules now, shared with `PolicyExecutor`:

| Before | Now |
| --- | --- |
| `jog_joints(mg)` (500 ms buffer for `set_target`) | `jog_joints(mg, live_buffer=LiveBuffer(window_ms=500))` |
| `jog_joints(mg, buffer_window_ms=0)` | `jog_joints(mg)` |
| `jog_joints(mg, ease_in_s=1.0)` | `jog_joints(mg, stages=[EaseIn(duration_s=1.0)])` |

**A script that relied on the default buffer now sends each live target alone:** measured at a
~90 Hz loop, 0.45–0.84° from the target instead of 0.015°, with arm peaks up to twice the
commanded speed. Pass `live_buffer=LiveBuffer(window_ms=500)` to keep the old behaviour; the
shipped live-target examples do. `set_chunk` is unchanged. `EaseIn` now blends by each
waypoint's timestamp and with a smootherstep.
