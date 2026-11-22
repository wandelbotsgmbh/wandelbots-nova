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
| a `LiveBuffer` in `pipeline=` with `SequentialExecution()` | use `ContinuousExecution()` |

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

## Buffering and ease-in are pipeline modules

`buffer_window_ms` (on `PolicyExecutor`, `jog_joints`, `jog_tcp`) and `ease_in_s` (on the
joggers) are gone. Both are modules of the send pipeline now, neither in the defaults; add them
by extending `default_pipeline()` (see the next section):

| Before | Now |
| --- | --- |
| `jog_joints(mg)` (500 ms buffer for `set_target`) | `jog_joints(mg, pipeline=[LiveBuffer(window_ms=500), *default_pipeline()])` |
| `jog_joints(mg, buffer_window_ms=0)` | `jog_joints(mg)` |
| `jog_joints(mg, ease_in_s=1.0)` | `jog_joints(mg, pipeline=[*default_pipeline(), EaseIn(duration_s=1.0)])` |
| `PolicyExecutor(..., buffer_window_ms=80)` | `PolicyExecutor(..., pipeline=[LiveBuffer(window_ms=80), *default_pipeline(execution=..., policy=...)])` |

**A jogging script that relied on the default buffer now sends each live target alone:** measured
at a ~90 Hz loop, 0.45–0.84° from the target instead of 0.015°, with arm peaks up to twice the
commanded speed. Add `LiveBuffer(window_ms=500)` in front of the defaults to keep the old behaviour; the shipped
live-target examples do. For a one-step policy in `PolicyExecutor` the buffer measured worse than
the default lead path (`docs/adr/0009`). `EaseIn` now blends by each waypoint's timestamp with a
smootherstep. With a buffer, a chunk's IO goes with its newest target instead of at once.

Also changed with the buffer in `PolicyExecutor`:

- **Multi-step and TCP chunks are accepted.** The old buffered mode refused them; the module
  passes a multi-step chunk through unchanged (and clears its window, warning once), so it is
  placed by T1–T3 like any other chunk.
- **A lone target before the window fills keeps the caller's padding**: it is sent as the caller
  queued it, with hold padding to the buffer horizon, until the buffer has two points to replay.
- **It needs `ContinuousExecution`**: a `LiveBuffer` under sequential execution is refused at
  construction.

## The built-ins are pipeline modules, and an explicit pipeline is the whole list

Smoothing, the bridge, the endpoint ramp, the joint clamp, hold padding and dropping late
waypoints are modules of the send pipeline now (ADR 0010). `pipeline=None` (the default) runs
`default_pipeline()`, which does what the built-ins did; **an explicit list is exactly what runs**.
A `pipeline=[LiveBuffer(...)]` or `pipeline=[EaseIn(...)]` from the previous release now runs only
that module: no padding and no dropping of late waypoints, and in `PolicyExecutor` no clamp and
no bridge. Extend the defaults instead. For `PolicyExecutor`, pass the same execution and policy:

```python
pipeline = [*default_pipeline(execution=execution, policy=policy), EaseIn(duration_s=1.0)]
PolicyExecutor(schema, policy, execution=execution, pipeline=pipeline)
```

| Before | Now |
| --- | --- |
| `WaypointConfig(min_chunk_horizon_ms=300)` | `HoldPadding(min_horizon_ms=300)` in place of `HoldPadding()` |
| `WaypointConfig(min_chunk_horizon_ms=0)` | the defaults without `HoldPadding` |
| `PolicyExecutor(..., clamp_to_joint_limits=False)` | the defaults without `JointClamp` |
| `SequentialExecution(endpoint_ramp=EndpointRamp(interpolation_steps=4))` | `EndpointRamp(interpolation_steps=4)` in place of `EndpointRamp()` |
| `SequentialExecution(endpoint_ramp=None)` | the defaults without `EndpointRamp` |
| `QueuedPolicyClient(policy, smoothing_passes=1)` | `Smoothing(passes=1)` in place of `Smoothing()` |
| `QueuedPolicyClient(policy, smoothing_passes=0)` | the defaults without `Smoothing` |
| `LeRobotPolicyClient(..., async_queue_smoothing=False)` | the defaults without `Smoothing` |

For example, a sequential run without the ramp:

```python
execution = SequentialExecution()
pipeline = [
    m
    for m in default_pipeline(execution=execution, policy=policy)
    if not isinstance(m, EndpointRamp)
]
```

Also changed:

- **A timestep queue without a `Bridge` is refused.** In continuous execution a
  `QueuedPolicyClient` or LeRobot's async queue anchors its timeline on its first bridged chunk;
  an explicit pipeline without `Bridge(always_anchor=True)` raises at construction.
- **The queue clients return their plan unsmoothed**, with `ActionChunk.retained_prefix_steps`
  set; the session smooths it. Rerun's action plot and pre-send stop conditions see the
  unsmoothed plan. LeRobot's plan is now smoothed after its targets are resolved to NOVA units:
  the same for absolute joints with unit scaling, different for relative actions and rotation
  conversions.
- **Dropped waypoints keep their exact times.** A chunk that loses its late steps no longer
  rebases the rest on the first kept one, so some waypoints of a timestep queue move by up to
  1 ms onto `anchor + int(k * dt)`, and so does the queue's timeline origin.
- **`ModuleContext`** gains `motion_group_id`, `mode` and `joint_limits`; **`PendingChunk`** gains
  `start_step` and `retained_prefix_steps`. A chunk module that puts steps in front of the policy's
  first step keeps `start_step` and `when_due` on it.
- **The chunk-level shaping functions are gone:** `smooth_action_chunk`, `interpolate_action_chunk_ramps`,
  `connect_action_chunk`, `create_bridge_chunk`, `chunk_duration_s` and their result types. The
  modules (`Smoothing`, `EndpointRamp`, `Bridge`) do the same per motion group; the per-group
  helpers are `novapolicy.chunking.smooth_steps`, `ramp_steps` and `bridge_steps`.
- **Removed shims:** `update_chunk(anchor_offset_steps=...)` (use `timestamp_offset_steps`),
  `update_chunk(server_dt_ms=...)` and `make_waypoints_request` (the session stamps chunks itself).
