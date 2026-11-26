# Migration

novapolicy is an experimental extra, so its API can still change between releases. This page
lists the changes that need an edit in your code, or that change what an unchanged program does.

How to use this page:

1. Find the area your code uses: policy clients, schema, execution, the send pipeline, jogging,
   or imports.
2. Apply the "Now" column of each table, or the change described.
3. If an upgrade fails with an error, look it up in [Troubleshooting](#troubleshooting).

## Policy clients

### Step rate: `control_freq`

Every client takes the policy's step rate as `control_freq=` in Hz: the training data's frame rate
(`fps` in a LeRobot dataset's `meta/info.json`) or a reinforcement-learning policy's control
frequency. It is not `ContinuousExecution(rate_hz=...)`, which sets how often the policy is asked.

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
- `dt_ms=`, `fps=` and `playback_speed=` are removed: pass `control_freq=`.

### LeRobot has no async queue of its own

`LeRobotPolicyClient(use_async_queue=..., async_queue_aggregation=..., async_queue_refill_threshold=...)`
are gone. Wrap the client in `QueuedPolicyClient(LeRobotPolicyClient(...), aggregation=...)` for
continuous execution, as any chunk-per-observation policy; `AsyncQueueAggregation` stays in
`novapolicy.providers.lerobot`. The queue now asks the server for a new chunk at every tick it is
called instead of refilling at a threshold in the background, and its frozen head follows the
measured inference time instead of a fixed three steps.

### GR00T real-time chunking (RTC) is removed

`Gr00tPolicyClient(rtc=...)`, `RTCConfig`, `client.rtc`, `reset_rtc()` and
`ActionChunk.seam_backdate_steps` are gone. Run GR00T sequentially, or continuously with the
pipeline's `Smoothing` and `Bridge` modules to connect chunks.

### GR00T applies the schema's joint operators

`Gr00tPolicyClient` used to send joint positions to the server in radians even when the schema
declared `ops` (`Observation.joint_positions(..., ops=[Rad2Deg()])`), while the executor still
inverted those ops on the returned actions. It now applies them on the way in, as every other
client does. A GR00T schema without `ops` is unaffected.

### GR00T IO comes from the chunk's first step

`Gr00tPolicyClient` used to take an IO action's value from the last step of the chunk; it now
takes the first, like the LeRobot client, since IO is written when that step is due. A policy
that toggles a gripper inside a chunk now has it follow the first step of each chunk.

### GR00T builds its state through the schema

`Gr00tPolicyClient` builds its observation with `PolicySchema.build_observation` and regroups it
per key (`observation_groups`), as every other client maps it. Two visible effects:

- **An `Observation.computed` hook gets the flat observation** (`{"arm_1": ..., "arm_2": ...}`),
  as with every other client, instead of GR00T's `(1, 1, D)` state arrays. Its numeric results
  still become state arrays under their own names.
- **An IO observation is 0.0 when the IO read returned nothing** (an empty `io_values`), as with
  every other client, instead of being left out.

`PolicySchema.velocity_and_pose_values` is gone; use `build_observation` and
`observation_groups`.

### A policy brings its own modules

- **`PolicyClient.requires_first_waypoint_bridge` is replaced by `PolicyClient.default_modules`**,
  the send-pipeline modules the policy needs in front in continuous execution. A timestep queue
  returns `timestep_queue_modules()`: `Smoothing`, `TimestepTimeline`, `Bridge(always_anchor=True)`.
  A custom queue client overrides `default_modules` instead of the flag.
- **The queue's timeline is the `TimestepTimeline` module** in each session, and the executor tells
  the policy which step is executing from `WaypointJoggingSession.executing_action_timestep`. An
  explicit pipeline for a queue policy must contain a `TimestepTimeline` (refused otherwise); a
  `Bridge` is no longer required.
- **`PendingChunk.action_timestep`** names the timestep of the caller's first step
  (`steps[start_step]`), also behind a bridge.

### The NATS policy provider is removed

`NatsPolicyClient` (`novapolicy.nats`), the `novapolicy-nats` extra, the NATS mock policy server
and the NATS dual-arm example are gone. A policy service reached over NATS is a provider of its
own: a `PolicyClient` whose `get_actions` builds the observation with the schema, sends it over
the service's subjects and decodes the reply with `schema.decode_actions`. See
[adding a provider](adding-a-provider.md); `examples/http_policy.py` is one written that way.

## Schema and actions

### A stale Omniverse prim pose ends the run

`OmniverseScene(...).prim(...)` takes `max_age_s` (default 1.0 s). When polling keeps failing and
the latest pose is older than that, `read()` raises `RuntimeError` and the run ends, as for a stale
camera frame. It used to return the last good pose for the rest of the run. Pass a larger
`max_age_s` for a scene that answers slowly.

### Relative TCP rotations are composed

`Action.tcp(..., mode="relative")` used to add the rotation offset to the current rotation vector
component by component. It now turns the tool about the TCP by the offset, composed as rotations.
Positions are unchanged. For small rotations away from a tool-down pose the result is nearly the
same; near a tool-down pose (rotation vectors close to π) the old sum pointed the tool in the wrong
direction. Nothing to change in code; a policy that relied on the old sum turns differently.

New options on the same action: `accumulate=True` for per-step deltas, and
`use_tool_coordinate_system=True` for offsets in the tool coordinate system (see
[schema.md](schema.md)).

### Pre-send stop conditions see the steps that run

At inference time, `StopContext.target_joints` and `target_ios` now hold only the
`n_action_steps` steps that will be sent, not the whole predicted chunk. A stop condition that
looked at later steps of the horizon no longer sees them.

## Execution

### `PolicyExecutor(plant=...)` is removed

The executor no longer simulates a plant between a policy and the robot: it maps observations,
runs the policy and sends its actions to NOVA, and NOVA moves the robot. `novapolicy.plant`
(`VirtualPlant`) and the `plant=` argument are gone; no shipped plant existed. Drop the argument.

### The start move keeps the cell's collision setups

`PolicyExecutor(start_joint_position=...)` used to clear the motion group's collision setups for
the move to the start pose. That move is now planned with them, like any other move: a start pose
inside a collision zone fails to plan instead of being reached without collision checks. The
joggers' start move (`jog_joints` and `jog_tcp` with `start_joint_position=`) keeps them too.

### Sequential execution refuses setups it would run wrongly

`SequentialExecution` waits for standstill before it observes again, so nothing replaces a
running plan. `PolicyExecutor` now raises `ValueError` at construction for the setups that
depend on that, instead of running them with timings that never come:

| Before (ran, wrongly) | Now |
| --- | --- |
| `QueuedPolicyClient(...)` or `LeRobotPolicyClient(use_async_queue=True)` with `SequentialExecution()` | use `ContinuousExecution()`, or the wrapped policy without the queue |

`use_async_queue=` is removed as well: wrap the client in `QueuedPolicyClient` instead (see
[LeRobot has no async queue of its own](#lerobot-has-no-async-queue-of-its-own)).

A chunk with its own `first_timestamp_ms` is no longer bridged from the measured pose: the bridge
rebuilt the chunk without that timestamp. It is sent at the time the policy set.

### One placement rule for both modes

Step `k` of a chunk is now due `ActionChunkConfig.max_expected_latency_ms + k·dt` after a reference
in both modes: the observation in `ContinuousExecution`, the send in `SequentialExecution` (the
robot was at rest when it was observed). Chunks with their own `first_timestamp_ms` are
unchanged.

- **Sequential chunks** used to start one `dt` after the send; they now start one lead later
  (100 ms by default), and keep their hold padding.
- **Continuous chunks** for the latest observation are dated by that observation, on the
  server's clock. Steps already too close to send are dropped. Before, step 0 was placed at
  server "now" minus the inference time, rounded to whole steps. Chunk policies (GR00T, LeRobot)
  therefore run about `max_expected_latency_ms` later than before, and lose no head steps while
  the latency stays below it. Chunks on their own timeline keep the old placement: the timestep
  queues (`QueuedPolicyClient`) and bridged chunks.

### IO and computed actions run when the policy's first step is due

A chunk's IO (`ActionChunk.ios`) and its `Action.computed` hooks now run when the policy's first
step is due on the server's clock, in both modes and on every path, instead of at send (most
paths) or after the loop blocked until that moment (chunks with a bridge in front). The loop no
longer waits for them. A chunk replaced before it was sent runs neither (its motion never ran);
one still pending when the run stops is cancelled; a hook that raises ends the run. NOVA's action
chunks carry no IO, so the write reaches the controller one network delay after that moment.

### Sequential chunks end in a 300 ms hold

`SequentialExecution`'s default pipeline holds every chunk's final target 300 ms past its last
step (`HoldPadding(after_ms=300)`, `SEQUENTIAL_HOLD_MS`). Before, long chunks got no hold, and an
arm still moving at its last step could come to rest past it. Each sequential cycle can take up
to 300 ms longer; pass your own `pipeline=[...]` with `HoldPadding()` for the old ending.

### Session failures end the run; invalid actions and refused schemas fail before motion

- **Any end of the action chunk stream while a session runs fails it**, and the next check
  (`PolicyExecutor`'s tick, the jogger's loop) raises: besides an `OSError` or `RuntimeError`,
  now also the API client's bare `Exception` on an error frame, websockets'
  `ConnectionClosedError`, an exception from a waypoint module or a stop condition, and a stream
  NOVA closes without an error. These used to end the sender silently while the policy kept
  running. Errors other than `OSError` and `RuntimeError` surface as a `RuntimeError` naming the
  motion group, with the original as its `__cause__`.
- **A state stream that ends after startup fails the session**, with a `RuntimeError`
  (`state stream ended for <motion group>`) or the stream's own error. The policy used to keep
  getting the last state. A stream that ends during a requested stop is still no failure.
- **`PolicyExecutor` connects the policy and runs `validate_schema` before the start move**
  (`start_joint_position`), not after it: a schema the policy refuses now fails before the robot
  moves. `connect` and `validate_schema` therefore run before the cameras, the sessions, IO and
  pose sources are started; the shipped providers need none of them there.
- **A failed IO write or computed action of a chunk without motion ends the run with its
  error**, also on the run's last tick, and the sessions stop instead of draining. The error used
  to be swallowed.
- **A failed IO write raises**, for a moving group's IO and in the joggers too: the session
  fails, and the run or the jogger's next check raises the write's error (its note names the IO).
  It used to be logged as a warning while the run went on without it.
- **The last chunks' effects are awaited.** The sessions drain before the run counts as
  completed, so an IO write or computed action due during the drain that raises ends the run with
  its error instead of a result.
- **Action values are checked when queued.** `WaypointJoggingSession.update_chunk`
  raises `ValueError` for a value that is not finite (NaN, infinity) or a step whose width is not
  the session's (the joint count once known, 6 for TCP poses), naming the motion group, the step
  and the expected width; such a chunk used to reach NOVA. The executor and `set_chunk` /
  `set_target` check every motion group before they queue any, so in `PolicyExecutor` such a
  chunk ends the run with that `ValueError` before any arm gets it. The new
  `WaypointJoggingSession.check_steps(steps)` runs the same check alone.

### Diagnostics removed

- **`on_stale`, `OnStale` and `hold_budget_s` are gone:** a stale camera frame or an inference
  past `inference_timeout_s` always ends the run with `RuntimeError` (the old default, `ABORT`).
- **No late-acknowledgement guard:** one-step waypoints that keep reaching the controller with
  less than 67 ms of their lead left no longer stop the run with `MotionError("over budget")`.
  Size `ActionChunkConfig.max_expected_latency_ms` for the network.
- **`WaypointJoggingSession.lead_left_ms`**, the per-session lead summary and the waypoint-tracking
  log lines are gone.

## Send pipeline

### `LiveBuffer` is removed

The live-target buffer is gone: the `LiveBuffer` module, and with it `buffer_window_ms`. It
replayed the last targets as a horizon, but NOVA follows each waypoint without looking ahead, so
the replayed horizon only delayed the robot by its window. Send live targets alone, or add
`ServoHorizon()` in front of the defaults for a smooth path that brakes onto the newest target:

| Before | Now |
| --- | --- |
| `pipeline=[LiveBuffer(window_ms=500), *default_pipeline()]` | `pipeline=None` (each live target alone), or `pipeline=[ServoHorizon(), *default_pipeline()]` |
| `jog_joints(mg)` (500 ms buffer for `set_target`) | `jog_joints(mg)`, or `jog_joints(mg, pipeline=[ServoHorizon(), *default_pipeline()])` |
| `jog_joints(mg, buffer_window_ms=0)` | `jog_joints(mg)` |
| `PolicyExecutor(..., buffer_window_ms=80)` | `PolicyExecutor(...)` without it |
| `novapolicy.jogging.live_buffer.LIVE_LEAD_MS` | `ActionChunkConfig.max_expected_latency_ms` (`jog_joints(..., config=...)`) |
| `novapolicy.jogging.live_buffer.TIMELINE_GRID_MS`, `trajectory_elapsed_ms` | `novapolicy.jogging.jogger.TIMELINE_GRID_MS`, `trajectory_elapsed_ms` |

A live target now goes out as one waypoint, due `ActionChunkConfig.max_expected_latency_ms` (100 ms by
default) after the jogger's loop time, the same lead a policy's chunks use.
The joint and TCP jogging examples (`jogging_joints.py`, `jogging_tcp.py`) use `ServoHorizon` for
their live targets. The executor no longer refuses anything about a buffer.

### Ease-in is a pipeline module

`ease_in_s` (on the joggers) is gone. Ease-in is a module of the send pipeline now, not in the
defaults; add it by extending `default_pipeline()`:

| Before | Now |
| --- | --- |
| `jog_joints(mg, ease_in_s=1.0)` | `jog_joints(mg, pipeline=[*default_pipeline(), EaseIn(duration_s=1.0)])` |

`EaseIn` now blends by each waypoint's timestamp with a smootherstep.

### The built-ins are pipeline modules, and an explicit pipeline is the whole list

Smoothing, the bridge, the endpoint ramp, the joint clamp, hold padding and dropping late
waypoints are modules of the send pipeline now. `pipeline=None` (the default) runs
`default_pipeline()`, which does what the built-ins did; **an explicit list is exactly what runs**.
A `pipeline=[EaseIn(...)]` from the previous release now runs only that module: no padding and no
dropping of late waypoints, and in `PolicyExecutor` no clamp and no bridge. Extend the defaults
instead. For `PolicyExecutor`, pass the same execution and policy:

```python
pipeline = [*default_pipeline(execution=execution, policy=policy), EaseIn(duration_s=1.0)]
PolicyExecutor(schema, policy, execution=execution, pipeline=pipeline)
```

| Before | Now |
| --- | --- |
| `ActionChunkConfig(min_chunk_horizon_ms=300)` | `HoldPadding(min_horizon_ms=300)` in place of `HoldPadding()` |
| `ActionChunkConfig(min_chunk_horizon_ms=0)` | the defaults without `HoldPadding` |
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

- **A timestep queue without its timeline is refused.** In continuous execution a
  `QueuedPolicyClient` keeps its chunks on one timeline
  (`TimestepTimeline`); an explicit pipeline without it raises at construction.
- **The queue clients return their plan unsmoothed**, with `ActionChunk.retained_prefix_steps`
  set; the session smooths it. Rerun's action plot and pre-send stop conditions see the
  unsmoothed plan. LeRobot's plan is now smoothed after its targets are resolved to NOVA units:
  the same for absolute joints with unit scaling, different for relative actions and rotation
  conversions.
- **Dropped waypoints keep their exact times.** A chunk that loses its late steps no longer
  rebases the rest on the first kept one, so some waypoints of a timestep queue move by up to
  1 ms onto `anchor + int(k * dt)`, and so does the queue's timeline origin.
- **`ModuleContext`** gains `motion_group_id`, `mode`, `joint_limits` and `scheduled_start`;
  **`PendingChunk`** gains `start_step` and `retained_prefix_steps`. A chunk module that puts
  steps in front of the policy's first step keeps `start_step` and `when_due` on it.
- **The chunk-level shaping functions are gone:** `smooth_action_chunk`, `interpolate_action_chunk_ramps`,
  `connect_action_chunk`, `create_bridge_chunk`, `chunk_duration_s` and their result types. The
  modules (`Smoothing`, `EndpointRamp`, `Bridge`) do the same per motion group; the per-group
  helpers are `smooth_steps`, `ramp_steps` and `bridge_steps`, next to their modules in
  `novapolicy.pipeline.modules` (`smoothing`, `endpoint_ramp`, `bridge`).
- **Removed shims:** `update_chunk(anchor_offset_steps=...)` (use `timestamp_offset_steps`),
  `update_chunk(server_dt_ms=...)` and `make_waypoints_request` (the session stamps chunks itself).
- **Removed deprecated rate parameters:** `LeRobotPolicyClient(fps=...)` and
  `Gr00tPolicyClient(dt_ms=...)` (pass `control_freq=`), `LeRobotPolicyClient(playback_speed=...)`.

### Chunk modules run in a fixed order, and a built-in module only once

`split_pipeline` now refuses chunk modules out of their stages' order (`ServoHorizon`,
`Smoothing`, `TimestepTimeline`, `Bridge`, `EndpointRamp`, `HoldPadding`; `JointClamp` and your
own modules anywhere) and a built-in module listed twice. Such a list used to run, wrongly.
Reorder the list as the error says. The executor also warns for the joggers' default list and for
`EndpointRamp` in continuous execution; both still run.

### `Timestamps` has a setting, and `SplitPipeline` a fourth field

`Timestamps(grid_ms=...)` puts one-step joint targets on NOVA's cycle grid (opt-in; see
[executor.md](executor.md#one-step-joint-targets-on-the-cycle-grid-timestampsgrid_ms)).
`Timestamps()` without it sends the same waypoints as before. Two API changes come with it:

- **`SplitPipeline` has a fourth field, `timestamps`**: the pipeline's `Timestamps` item, or a
  `Timestamps()` when the list has none. Code that unpacks `split_pipeline(...)` into three names
  breaks; unpack four or read the fields by name.
- **`Timestamps` is no longer frozen**: it keeps the grid's path between requests. Each session
  runs its own deep copy of the pipeline, so one instance in a list shared by several motion
  groups is still fine.

### Live targets and jogger chunks land on their grid exactly

Nothing to change in code; what is sent differs slightly:

- **A jogger chunk starts on the 10 ms grid.** Its anchor rounds each term instead of
  truncating their sum, which started some chunks 1 ms early. Step zero can move by 1 ms.
- **New, opt-in: `ServoHorizon`** follows live targets with a spring-damper and sends its braking
  path as a 400 ms horizon ([jogging.md](jogging.md#following-a-hand-servohorizon)).
  The default pipeline is unchanged.

## Sessions, jogging and Rerun

### One session lifecycle and one tracking path for executor and jogger

- **`WaypointJoggingSession.set_state_observer`** is replaced by `add_state_observer` /
  `remove_state_observer`: several observers run per state packet, and one that raises does not
  skip the others. `current_state_generated_at` is gone.
- **Rerun tracking** (commanded against measured) comes from the waypoints the session actually
  sent and is logged per state packet, in the executor too (it was once per tick).
  `PolicyRerunLogger(state_sample_interval_ms=...)`, `log_target_tracking` and
  `log_joint_tracking(trail=...)` are gone.
- **Teardown** drains every arm in parallel, before Rerun, the e-stop monitor, cameras, pose
  sources and IO streams stop; sessions are stopped even if one of those fails.

### A slimmer transport core

The session, the executor, the jogger and their shared lifecycle lost duplicate and test-only
members. Code that only uses `PolicyExecutor`, `jog_joints` and `jog_tcp` needs no change.

| Before | Now |
| --- | --- |
| `WaypointJoggingSession.scheduled_chunk_count` | `session.scheduled.chunk_count` |
| `WaypointJoggingSession.scheduled_until_server_ms` | `session.scheduled.until_server_ms` |
| `WaypointJoggingSession.scheduled_waypoint_timestamps` | `session.scheduled.waypoint_timestamps` |
| `WaypointJoggingSession.scheduled_action_timestep` | `session.scheduled.action_timestep` |
| `WaypointJoggingSession.scheduled_at_server_ms` | `session.scheduled.at_server_ms` |
| `WaypointJoggingSession.scheduled_timestamp_for_step(step)` | removed; a chunk's `when_due` step and `executing_action_timestep` keep the caller's timestamps |
| `WaypointJoggingSession.session_elapsed_ms` | `estimated_server_timestamp_ms` (the same value) |
| `WaypointJoggingSession.is_running` | `jogging_state == "RUNNING"` |
| `WaypointJoggingSession.standstill_ms` | removed; `is_at_standstill` |
| `JoggingTimeClock.acknowledged_elapsed_ms` | `estimated_server_timestamp_ms` (the same value) |
| `JoggingTimeClock.extract_from_state(state)` | `jogging_details(state)` (`novapolicy.transport.state`), then `.jogger_session_timestamp_ms` |
| `novapolicy.estop.check_sessions`, `check_estop`, `triggered_stop_condition` | `SessionLifecycle.check()` and `SessionLifecycle.stop_condition` |
| `SessionLifecycle(sessions, motion_groups)` | `SessionLifecycle(config=..., stop_conditions=..., pipeline=...)`, then `build({mg: (mode, tcp)})`; `start_monitoring()` starts its own `EstopMonitor` |
| `JointJogger(list_of_motion_groups, ...)` | `JointJogger({mg: "" for mg in ...}, ...)`; use `jog_joints` |

- `session.scheduled` is a frozen `Scheduled` record (`novapolicy.transport.session`),
  replaced on every send: read its fields together from one `session.scheduled`.
- `Placement` (`novapolicy.pipeline.placement`) has a `hold` field: whether `HoldPadding` may pad the chunk.
- A chunk without motion writes its IO and then runs its computed actions, one after the other
  in one task; they used to run side by side. A failing IO write now also skips the writes and
  computed actions after it (the run ends with the error either way). A moving group's IO still
  goes with its policy step.
- `PolicyExecutor.last_observation` is cleared when a run's observe-act loop starts, so a run
  that ends before its first observation reports `last_state=None`, not the previous run's.
- `set_target` and `set_chunk` check the value for every motion group before they send any. With
  several motion groups, a bare value is refused for not being a dict before its kind is checked,
  and a bare target of the wrong type says `Expected list[float] per motion group` (or `Pose`).

## Imports and package layout

### Waypoint names are now action chunk names

NOVA's API calls the stream it executes action chunks; the stream's names follow it. Same
behaviour; the names that mean single waypoints inside a chunk (`WaypointModule`,
`WaypointFilter`, `on_waypoints`, `TimedWaypoint`) stay.

| Before | Now |
| --- | --- |
| `WaypointJoggingSession` | `ActionChunkSession` |
| `novapolicy.transport.waypoints` | `novapolicy.transport.action_chunks` |
| `make_timed_waypoints_request` | `make_action_chunk_request` |
| Rerun `/policy/<mg>/waypoint_request/*`, `/policy/waypoint_requests` | `/policy/<mg>/action_chunk_request/*`, `/policy/action_chunk_requests` |

The entries above and below name the session as it was called before.

### `WaypointConfig` is now `ActionChunkConfig`

The configuration of NOVA's action chunk streaming is named after it. Same fields, same defaults:

| Before | Now |
| --- | --- |
| `from novapolicy import WaypointConfig` | `from novapolicy import ActionChunkConfig` |
| `PolicyExecutor(..., motion=WaypointConfig(...))` | `PolicyExecutor(..., motion=ActionChunkConfig(...))` |
| `jog_joints(..., config=WaypointConfig(...))` | `jog_joints(..., config=ActionChunkConfig(...))` |

### The package follows its architecture

The modules moved into one folder per part of the package: `mapping/`, `pipeline/` (with one file
per built-in module in `pipeline/modules/`), `transport/`, `jogging/` (the jogging API) and
`providers/`. Nothing else changed: no class or function was renamed and none behaves
differently. The live buffer was removed separately (see
[`LiveBuffer` is removed](#livebuffer-is-removed)).

- **`from novapolicy import ...` is unchanged**, `LeRobotPolicyClient` and `LeappPolicyClient`
  included.
- **`from novapolicy.jogging import ...` is unchanged** for every name it exported; the pipeline
  and transport names are re-exported there from their new homes. `novapolicy.pipeline` exports
  the same pipeline names (and `TimestepTimeline`).
- **Deep imports change** as in the table. `chunking` was split: each helper went to the module
  that uses it.
- **Logger names follow the module paths** (`novapolicy.transport.clock` instead of
  `novapolicy.jogging.clock`, ...): update logging configuration and `caplog` filters.
- **Mock and patch targets follow them too**: code that patches `WaypointJoggingSession` or
  `EstopMonitor` patches them in `novapolicy.transport.lifecycle`, not in `novapolicy.executor`
  or `novapolicy.jogging.jogger`.

| Before | Now |
| --- | --- |
| `novapolicy.schema` | `novapolicy.mapping.schema` |
| `novapolicy.ops` | `novapolicy.mapping.ops` |
| `novapolicy.poses` | `novapolicy.mapping.poses` |
| `novapolicy.pose_sources` | `novapolicy.mapping.pose_sources` |
| `novapolicy.joint_velocities` | `novapolicy.mapping.joint_velocities` |
| `novapolicy.state_stream` | `novapolicy.mapping.state_stream` |
| `novapolicy.joint_limits` | `novapolicy.mapping.joint_limits` |
| `novapolicy.training_range` | `novapolicy.mapping.training_range` |
| `novapolicy.chunking`: `TcpDelta`, `apply_relative_mode`, `compose_tcp_deltas` | `novapolicy.mapping.relative` |
| `novapolicy.chunking`: `unwrap_tcp_rotations` | `novapolicy.mapping.poses` |
| `novapolicy.chunking`: `trim_chunk`, `is_single_step`, `Placement`, `placement` | `novapolicy.pipeline.placement` |
| `novapolicy.chunking`: `smooth_steps` | `novapolicy.pipeline.modules.smoothing` |
| `novapolicy.chunking`: `bridge_steps` | `novapolicy.pipeline.modules.bridge` |
| `novapolicy.chunking`: `ramp_steps` | `novapolicy.pipeline.modules.endpoint_ramp` |
| `novapolicy.jogging.pipeline` (`ModuleContext`, `ChunkModule`, `WaypointModule`, `WaypointFilter`, `Timestamps`, `SplitPipeline`, `split_pipeline`, `CHUNK_STAGES`, `TimedWaypoint`) | `novapolicy.pipeline.base` |
| `novapolicy.jogging.pipeline.EaseIn` | `novapolicy.pipeline.modules.ease_in` |
| `novapolicy.jogging.modules.default_pipeline`, `timestep_queue_modules`, `SEQUENTIAL_HOLD_MS` | `novapolicy.pipeline.defaults` |
| `novapolicy.jogging.modules.Smoothing` | `novapolicy.pipeline.modules.smoothing` |
| `novapolicy.jogging.modules.TimestepTimeline` | `novapolicy.pipeline.modules.timeline` |
| `novapolicy.jogging.modules.Bridge` | `novapolicy.pipeline.modules.bridge` |
| `novapolicy.jogging.modules.EndpointRamp` | `novapolicy.pipeline.modules.endpoint_ramp` |
| `novapolicy.jogging.modules.JointClamp` | `novapolicy.pipeline.modules.joint_clamp` |
| `novapolicy.jogging.modules.HoldPadding` | `novapolicy.pipeline.modules.hold_padding` |
| `novapolicy.jogging.modules.DropLate`, `MIN_LEAD_MS` | `novapolicy.pipeline.modules.drop_late` |
| `novapolicy.jogging.live_buffer` | removed ([`LiveBuffer` is removed](#livebuffer-is-removed)) |
| `novapolicy.jogging.waypoint_session` | `novapolicy.transport.session` |
| `novapolicy.jogging.clock` | `novapolicy.transport.clock` |
| `novapolicy.jogging.waypoints` | `novapolicy.transport.waypoints` |
| `novapolicy.jogging.session` | `novapolicy.transport.state` |
| `novapolicy.jogging.lifecycle` | `novapolicy.transport.lifecycle` |
| `novapolicy.estop` | `novapolicy.transport.estop` |
| `novapolicy.io` | `novapolicy.transport.io` |
| `novapolicy.jogging.jogger` | unchanged |
| `novapolicy.policy_client` | `novapolicy.providers.base` |
| `novapolicy.queued_client` | `novapolicy.providers.queued` |
| `novapolicy.action_queue` | `novapolicy.providers.action_queue` |
| `novapolicy.gr00t` | `novapolicy.providers.gr00t` |
| `novapolicy.lerobot` | `novapolicy.providers.lerobot` |
| `novapolicy.leapp` | `novapolicy.providers.leapp` |

## Docs and examples

### GR00T docs moved

`novapolicy/gr00t/README.md` is now [docs/gr00t.md](gr00t.md), together with the guide for setting
up a GR00T policy on a real robot.

### Examples removed: episode tools and six jogging variants

- `examples/replay/replay_episode.py` (with its sample episode) and
  `examples/settings_from_episode.py` are gone. The pick-and-place example
  (`examples/execute_gr00t_single_arm_pick_and_place/`) keeps its measured settings in
  `episode_settings.json` and the loader it needs in `episode_settings.py`.
- The eight `examples/jogging/jogging_{single,dual}_{joint,tcp}[_chunked].py` scripts are now
  two, `jogging_joints.py` and `jogging_tcp.py`; `--dual` and `--chunked` select the variant.

## Troubleshooting

| symptom after upgrading | what to do |
| --- | --- |
| `TypeError` for `dt_ms=`, `fps=` or `playback_speed=` | pass `control_freq=` ([Step rate](#step-rate-control_freq)) |
| a LeRobot policy plays twice as fast | pass `control_freq=15` (or the dataset's `fps`); the default is now 30 |
| `TypeError` for `buffer_window_ms=` | remove it ([`LiveBuffer` is removed](#livebuffer-is-removed)) |
| `TypeError` for `ease_in_s=` | add `EaseIn` to the pipeline ([Ease-in](#ease-in-is-a-pipeline-module)) |
| `ImportError` for `WaypointJoggingSession` or `novapolicy.transport.waypoints` | renamed, see [Waypoint names are now action chunk names](#waypoint-names-are-now-action-chunk-names) |
| `ImportError` for `WaypointConfig` | renamed to `ActionChunkConfig`, see [above](#waypointconfig-is-now-actionchunkconfig) |
| `ImportError` for `LiveBuffer` or `LIVE_LEAD_MS` | see [`LiveBuffer` is removed](#livebuffer-is-removed) |
| `TypeError` for `use_async_queue=` on `LeRobotPolicyClient` | wrap it in `QueuedPolicyClient` ([LeRobot](#lerobot-has-no-async-queue-of-its-own)) |
| `TypeError` for `rtc=` on `Gr00tPolicyClient` | remove it ([RTC is removed](#gr00t-real-time-chunking-rtc-is-removed)) |
| `TypeError` for `plant=`, `min_chunk_horizon_ms=`, `clamp_to_joint_limits=`, `endpoint_ramp=` or `smoothing_passes=` | see the tables under [Send pipeline](#send-pipeline) and [Execution](#execution) |
| jogging live targets overshoot or are not smooth | add `ServoHorizon()` in front of the defaults |
| after adding one module, padding, clamping or late-step dropping stopped | an explicit pipeline is the whole list; extend `default_pipeline(...)` |
| `ValueError` at construction about `ContinuousExecution` | see [Sequential execution refuses setups](#sequential-execution-refuses-setups-it-would-run-wrongly) |
| `ValueError` at construction about a `TimestepTimeline` | build the pipeline from `default_pipeline(execution=..., policy=...)` |
| an error about module order or a module listed twice | reorder the list as the error says ([fixed order](#chunk-modules-run-in-a-fixed-order-and-a-built-in-module-only-once)) |
| `ValueError: too many values to unpack` from `split_pipeline` | unpack four fields, or read them by name |
| `ModuleNotFoundError` for a deep import | see the table in [Imports and package layout](#imports-and-package-layout) |
| the start move fails to plan | the start pose is in a collision zone; it is now checked |
| the run now fails before the start move | the policy refused the schema; fix the schema |
| `ValueError` naming a motion group and a step | an action had a non-finite value or the wrong width; fix the policy output |
| each sequential cycle takes longer | expected: chunks end in a 300 ms hold; use `HoldPadding()` for the old ending |
