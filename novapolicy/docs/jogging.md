# Jogging

Stream waypoints to the NOVA Jogging API directly — no policy, no schema, no
cameras. This is the simplest way to move a robot: open a session, send targets,
and the server handles velocity profiling, interpolation, limits, and servo
control internally.

> Building policy execution on top of this? See [executor.md](executor.md) for
> the `PolicyExecutor` loop and the timestamp protocol.

The `jog_joints()` and `jog_tcp()` functions provide a simple async context
manager for interactive jogging. Both accept an optional `start_joint_position`
that PTP-moves the robot to a known position before the session starts, so it
begins at a safe, predictable location.

## Joint jogging

```python
from novapolicy import jog_joints

HOME = [0, -1.57, 1.57, -1.57, -1.57, 0]

async with jog_joints(mg, start_joint_position=HOME) as jogger:
    async for state in jogger:
        # Single target (server interpolates from current position)
        jogger.set_target([0.0, -1.57, 1.57, -1.57, -1.57, 0.0])
```

## TCP jogging

```python
from novapolicy import jog_tcp
from nova.types import Pose

START = [1.17, -0.73, 1.75, -3.05, 0.87, 2.09]

async with jog_tcp(mg, tcp="Flange", start_joint_position=START) as jogger:
    async for state in jogger:
        jogger.set_target(Pose(500, 200, 300, 0, 3.14, 0))
```

## Chunked targets

Sending multi-step chunks enables the server to plan smooth trajectories
with proper velocity profiling:

```python
async with jog_joints(mg) as jogger:
    async for state in jogger:
        # 8 future targets at 33ms spacing
        chunk = [compute_target(t + i * 0.033) for i in range(8)]
        jogger.set_target(chunk, dt_ms=33.0)
```

Regenerate chunks at a stable cadence that leaves multiple future waypoints in
the queue. Replacing a long lookahead on every state callback repeatedly resets
the server profile; replacing too slowly lets the queue run dry. For finite
motion, ramp the target to zero velocity and acceleration. The chunked examples
show this pattern.

`dt_ms` is required for a chunk — it is the spacing between its steps.

### Where a chunk lands

Step zero is placed at `elapsed + jogger.LEAD_MS` (100 ms) on the absolute session
timeline, not at "now" resolved at send time. Content generated at trajectory time
`T` therefore always maps to the same timestamp, however often it is re-sent, so
overlapping chunks stitch together instead of the same trajectory point landing at
a slightly different timestamp in every chunk. The lead is constant on purpose:
anything time-varying in the time-to-position mapping moves already-commanded
points to new absolute times, and the robot executes each such shift as a lurch.

### Short chunks are padded

A chunk shorter than `WaypointConfig.min_chunk_horizon_ms` (default **500**) is
extended by repeating its final target, so the server has a horizon it can brake
within. The padding is sent but is not treated as commanded motion: it is excluded
from `scheduled_until_server_ms`, so anything waiting for the chunk to finish ends
at your last step rather than sitting at it for the rest of the window. Set the
config value to `0` to disable padding — a chunk whose last step is not a genuine
stopping point will then get a decelerate-to-standstill profile at its end.

### Late chunks are trimmed

A chunk is timestamped when you build it and sent a moment later. Leading
waypoints whose moment has already passed cannot be reached, and commanding them
makes the server jump to catch the trajectory; those are dropped at send time and
the still-reachable tail is sent. A chunk whose every waypoint has elapsed is
dropped. Surviving waypoints keep their original absolute timestamps, so trimming
never distorts the trajectory — see
[executor.md](executor.md#unreachable-waypoints-are-trimmed) for the effect on
step indices.

### Closing the session

Leaving the `async with` block waits for the waypoints already accepted by the
server to finish executing before tearing the session down, so a clean exit does
not truncate commanded motion part-way through. That wait is skipped when the loop
ended for a reason that wants the robot stopped now — a fault, an e-stop, or a
fired stop condition.

Rerun state visualization should not be configured faster than the controller
state source: duplicate states cost sampling and serialisation work without adding
trajectory fidelity, and approximately 30 Hz is the safe default for live robot
geometry. The Rerun writes themselves are handed to a worker thread rather than
paid on the control loop (see [rerun.md](rerun.md)).

## Live targets and the replay buffer (`buffer_window_ms`)

A live target says only where the target is *now*. A lone waypoint is a
**terminal** target: with no successor the server plans an
accelerate/decelerate-to-standstill profile, so a target replaced every tick makes
the robot stop and restart continuously.

`buffer_window_ms` (default **500**) is a ring buffer of recent live targets. The
jogger holds the last `buffer_window_ms` of them and replays them as a continuous
waypoint horizon, so the server always has somewhere to be going next. Nothing in
that horizon is invented — every waypoint in it is a target that was really
measured, replayed late rather than extrapolated forward.

The cost is latency: the robot trails the live target by a little over the window.
The window has to be a few hundred milliseconds, because the window *is* the
horizon and the server caps its speed at whatever it can brake to a stop within.
On the UR10e this was tuned against, a 150 ms window stalled a fifth of all
samples and 450 ms still stalled occasionally.

`buffer_window_ms=0` disables buffering: each target is sent alone, as measured,
with the halting motion described above. That is useful for stepping a robot to
discrete positions, not for tracking a moving one.

```python
async with jog_tcp(mg, tcp="Flange") as jogger:  # 500 ms window by default
    async for state in jogger:
        jogger.set_target(read_controller_pose())
```

`dt_ms` is **ignored** for live targets. Their spacing is not assumed from call
rate: each sample is stamped with the trajectory time it was produced at, and the
buffer is resampled onto a uniform grid (`WaypointConfig.single_step_dt_ms`) by
interpolating at those stamps. Handing the samples over with an averaged `dt`
instead replays each one at the wrong moment, which time-warps the trajectory.

`buffer_window_ms` applies to live targets only. A chunk carries its own horizon,
so there is nothing to buffer and no latency to pay — see
[Chunked targets](#chunked-targets). Pushing a chunk also clears the ring buffer;
alternating the two forms on one motion group means the live targets after each
chunk go out alone until the buffer refills, and the jogger warns once per motion
group when that happens.

## Timing targets (`jogger.elapsed`)

For time-parameterised motion (e.g. a sinusoid), drive it with `jogger.elapsed`
rather than your own `time.monotonic()` anchor. `elapsed` is seconds on the
**server's jogger-session clock**, measured from the first loop iteration that had
state, and it is what every target is placed on.

It must come from the server clock rather than a monotonic one: the robot executes
against the jogger session timer, and wall time drifts from it. Three properties
follow from that:

- **Sampled once per iteration.** Every target you derive from `elapsed` within
  one pass through the loop is stamped from the same instant.
- **Quantised** to a 10 ms grid (`jogger.TIMELINE_GRID_MS`), so successive chunks
  anchor on one absolute grid instead of each landing at a new phase.
- **Never runs backwards.** Server "now" is extrapolated between state samples, so
  a late sample can resolve below an earlier estimate; since content is a function
  of `elapsed`, a backward step would command the robot to reverse. Forward jumps
  are *not* limited — they are the clock correcting after the event loop was late
  reading the state stream, and rate-limiting them makes the timeline fall behind
  until every waypoint lands in the past.

There is no wait for the robot to report `RUNNING` before the clock starts.
Waiting for it deadlocks the live path: the timeline has to move before any target
spacing can be measured, and the robot has to move before it reports `RUNNING`. A
catch-up jump is not a concern either way, because targets are placed on an
absolute timeline and unreachable waypoints are dropped at send time.

```python
async with jog_joints(mg) as jogger:
    async for _ in jogger:
        t = jogger.elapsed
        if t >= 5.0:
            break
        target = list(HOME)
        target[0] += 0.2 * math.sin(2 * math.pi * 0.25 * t)
        jogger.set_target(target)
```

## Ease-in (`ease_in_s`)

A time-parameterised target with non-zero velocity at `t=0` (a sinusoid has
maximum velocity there) makes the robot lunge from a standstill to full speed on
the first move. Pass `ease_in_s` to `jog_joints`/`jog_tcp` to ramp motion up from
zero over the first N seconds: each target is blended from the robot's start
position toward the requested value, and is unchanged after the window. It is
**off by default** (`ease_in_s=0`).

The blend is a smoothstep (`f = e²(3 − 2e)` for `e = min(1, t / ease_in_s)`), not
a straight ramp. A straight ramp is continuous in position but not in velocity:
the blend rate drops from full to nothing the instant it completes, and the robot
executes that step in acceleration as a stumble a few hundred milliseconds later.
Smoothstep is zero-derivative at both ends, so entry and exit are both gradual.

```python
async with jog_joints(mg, ease_in_s=1.0) as jogger:  # gentle 1s ramp
    async for _ in jogger:
        ...
```

## Dual-arm

```python
from novapolicy import jog_joints, jog_tcp

# Joint jogging - two arms
async with jog_joints([mg1, mg2]) as jogger:
    async for states in jogger:
        jogger.set_target({mg1: target1, mg2: target2})

# TCP jogging - two arms with different TCPs
async with jog_tcp({mg1: "Flange", mg2: "Gripper"}) as jogger:
    async for states in jogger:
        jogger.set_target({mg1: pose1, mg2: pose2})
```

## Waypoint request types

The NOVA Jogging API accepts **timestamped waypoints** — either joint positions
or TCP poses:

| Mode | Request | Steps format | Use case |
|------|---------|--------------|----------|
| `"joint"` | `JointWaypointsRequest` | Joint radians `[j1, j2, ..., j6]` | Joint-space (default) |
| `"cartesian"` | `PoseWaypointsRequest` | TCP pose `[x, y, z, rx, ry, rz]` (mm + rad) | Cartesian-space |

`jog_joints` / `jog_tcp` pick the request type for you. Under a policy, the mode
is selected automatically based on whether the schema contains
`Observation.tcp(..., action=True)` entries.

## Error detection

The session monitors the NOVA jogging state stream for pause conditions.
Three of them are **blocking faults** — after consecutive ticks in one of these
states, a `MotionError` is raised:

| State | Meaning |
|-------|---------|
| `PAUSED_NEAR_JOINT_LIMIT` | Joint reached its limit |
| `PAUSED_NEAR_COLLISION` | Self-collision detected |
| `PAUSED_NEAR_SINGULARITY` | Kinematic singularity |

One pause is **recoverable** and never raises — the robot resumes on its own
once a fresh chunk arrives:

| State | Meaning |
|-------|---------|
| `PAUSED_BY_USER` | Waypoint buffer exhausted (send chunks faster) |
