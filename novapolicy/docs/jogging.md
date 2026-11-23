# Jogging

Stream waypoints to the NOVA Action Chunk Streaming API directly — no policy, no
schema, no cameras. The simplest way to move a robot: open a session, send targets,
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
        jogger.set_chunk(chunk, dt_ms=33.0)
```

Regenerate chunks at a stable cadence that leaves multiple future waypoints in
the queue. Replacing a long lookahead on every state callback repeatedly resets
the server profile; replacing too slowly lets the queue run dry. For finite
motion, ramp the target to zero velocity and acceleration. The chunked examples
show this pattern.

Chunks go to `set_chunk`, live targets to `set_target`. The two are separate
methods because they take different settings: a chunk needs `dt_ms` (the spacing
between its steps, required) and gets its horizon from `HoldPadding`, while a live
target needs no spacing and can get a horizon from a `LiveBuffer`. Handing a chunk to `set_target` raises `TypeError` rather
than silently ignoring the buffer, and vice versa.

For TCP jogging, note the asymmetry: `set_target` takes a `Pose`, while
`set_chunk` takes raw `[x, y, z, rx, ry, rz]` steps.

### Where a chunk lands

Step zero is placed at `elapsed + jogger.LEAD_MS` (100 ms) on the absolute session
timeline, not at "now" resolved at send time. Content generated at trajectory time
`T` therefore always maps to the same timestamp, however often it is re-sent, so
overlapping chunks stitch together instead of the same trajectory point landing at
a slightly different timestamp in every chunk. The lead is constant on purpose:
anything time-varying in the time-to-position mapping moves already-commanded
points to new absolute times, and the robot executes each such shift as a lurch.

### Short chunks are padded

A chunk shorter than `HoldPadding.min_horizon_ms` (default **500**, in the default
pipeline) is extended by repeating its final target, so the server has a horizon it can
brake within. The padding is sent but is not treated as commanded motion: it is excluded
from `scheduled.until_server_ms`, so anything waiting for the chunk to finish ends at your
last step rather than sitting at it for the rest of the window. Leave `HoldPadding` out of
the pipeline to disable padding: a chunk whose last step is not a genuine stopping point
then gets a decelerate-to-standstill profile at its end.

With `WaypointConfig.delay_until_braking_ms` set (NOVA 26.8.0-112 or newer), the
server keeps the robot's velocity that long after a chunk's last waypoint before it
brakes, and every new chunk restarts the countdown. Short chunks streamed at the
policy rate, down to a single waypoint, then run as one motion without padding:
leave `HoldPadding` out with it, because a padded chunk ends in a hold. About
200 ms covered 100-150 ms network stalls in teleoperation. Draining the session
sends the last target once more without the delay, so the robot stops on it
instead of running past it.

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

The robot stops when the jogging connection closes, wherever it happens to be on
the path. Two things follow.

First, leaving the `async with` block waits for the waypoints already accepted by
the server to finish executing before tearing the session down, so a clean exit
does not truncate commanded motion part-way through. That wait is skipped when the
loop ended for a reason that wants the robot stopped now — a fault, an e-stop, or
a fired stop condition — where closing the connection *is* the stop. It is also
abandoned after `_DRAIN_TIMEOUT_S` if the server stops reporting progress, which
is logged.

Second, **end your motion at rest**. Nothing in the session decelerates for you:
if the last commanded waypoint still carries full velocity, that is the velocity
the robot is at when the connection closes. Shape the trajectory so it arrives at
zero velocity instead. What to shape depends on the path:

| motion | ease | why |
|--------|------|-----|
| oscillation about a home pose | its **amplitude** | ramping the amplitude to zero returns to the home pose and stops there |
| a closed or fixed path (circle, line) | its **phase** | an amplitude envelope drags the TCP off the path — to a circle's centre, for one — while warping the phase keeps it exactly on the path and only slows it |

Use a curve that is zero in both the first and second derivative at the ends —
`smootherstep`, `6x⁵ − 15x⁴ + 10x³`. Plain `smoothstep` (`3x² − 2x³`) zeroes the
velocity but enters and leaves at peak acceleration, which the robot executes as a
stumble; that is the same defect as a linear ramp, one order up. This is also the
curve `EaseIn` uses. `jogging_tcp.py` shows the phase warp and
`jogging_joints.py --chunked` the amplitude envelope.

Rerun state visualization should not be configured faster than the controller
state source: duplicate states cost sampling and serialisation work without adding
trajectory fidelity, and approximately 30 Hz is the safe default for live robot
geometry. The Rerun writes themselves are handed to a worker thread rather than
paid on the control loop (see [rerun.md](rerun.md)).

## Live targets and the replay buffer (`LiveBuffer`)

A live target says only where the target is *now*. By default each one goes out as one
waypoint, and a lone waypoint is a **terminal** target: with no successor the server plans
an accelerate/decelerate-to-standstill profile, so a target replaced every tick makes the
robot brake toward every one of them.

`LiveBuffer(window_ms=500)` in front of the defaults holds the last `window_ms` of targets and
replays them as a continuous waypoint horizon on an even grid
(`WaypointConfig.single_step_dt_ms`), so the server always has somewhere to be going next.
Nothing in that horizon is invented: every waypoint is a target that was really measured,
replayed late rather than extrapolated forward. Measured on a virtual UR10e with a sine
target set at about 90 Hz: 0.015° rms against the target with 500 ms, 0.45–0.84° and arm
peaks up to twice the commanded speed without it.

The cost is latency: the robot trails the live target by a little over the window. The
window has to be a few hundred milliseconds, because the window *is* the horizon and the
server caps its speed at whatever it can brake to a stop within; on the UR10e a 150 ms
window stalled a fifth of all samples.

```python
pipeline = [LiveBuffer(window_ms=500), *default_pipeline()]
async with jog_tcp(mg, tcp="Flange", pipeline=pipeline) as jogger:
    async for state in jogger:
        jogger.set_target(read_controller_pose())
```

Spacing is not assumed from call rate: each sample is stamped with the trajectory time it
was produced at, and the buffer is resampled onto the grid by interpolating at those stamps.

The buffer applies to `set_target` only. Chunks go to `set_chunk` and carry their own
horizon — see [Chunked targets](#chunked-targets). Sending a chunk clears the buffer, and the
jogger warns once per motion group when that happens.

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
- **Quantised** to a 10 ms grid (`live_buffer.TIMELINE_GRID_MS`), so successive chunks
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

## The send pipeline (`pipeline=[...]`)

What `set_target` and `set_chunk` produce reaches NOVA through one ordered list of modules, the
same kind `PolicyExecutor` uses. Without `pipeline=` the joggers run `default_pipeline()`:
`HoldPadding()` (pads chunks), `Timestamps()`, `DropLate()` (drops waypoints whose moment has
passed). An explicit list is exactly what runs, so extend the defaults to add a module:

```python
from novapolicy import EaseIn, LiveBuffer, default_pipeline

async with jog_joints(
    mg, pipeline=[LiveBuffer(window_ms=500), *default_pipeline(), EaseIn(duration_s=1.0)]
) as jogger:
    ...
```

Modules before `Timestamps()` work on chunks (`LiveBuffer`, `HoldPadding`); filters after it
choose which timed waypoints go out without retiming them (`DropLate`); waypoint modules last may
change the values only (`EaseIn`). `Timestamps()` may be left out; it then sits after the last
chunk module. Each motion group runs its own copy. See `novapolicy.jogging.pipeline`,
`novapolicy.jogging.modules` and contract T13 in [contracts.md](contracts.md).

A time-parameterised target with non-zero velocity at `t=0` (a sinusoid has maximum velocity
there) makes the robot lunge from a standstill. `EaseIn(duration_s=1.0)` blends each waypoint from
where the robot stood toward its value over the first second, by its own timestamp, with a
smootherstep. Measured on a sine at about 90 Hz with a 500 ms buffer: peak speed in the first
second 54°/s instead of 85°/s.

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

# Chunks take the same per-motion-group dict form
async with jog_joints([mg1, mg2]) as jogger:
    async for states in jogger:
        jogger.set_chunk({mg1: chunk1, mg2: chunk2}, dt_ms=33.0)
```

## Waypoint kinds

The NOVA Action Chunk Streaming API accepts **timestamped waypoints** in a single
`ActionChunkRequest`. Each waypoint carries either joint positions or a TCP pose,
and the mode picks which:

| Mode | Waypoint kind | Steps format | Use case |
|------|---------------|--------------|----------|
| `"joint"` | `JointWaypoint` (`JOINTS`) | Joint radians `[j1, j2, ..., j6]` | Joint-space (default) |
| `"cartesian"` | `PoseWaypoint` (`POSE`) | TCP pose `[x, y, z, rx, ry, rz]` (mm + rad) | Cartesian-space |

`jog_joints` / `jog_tcp` pick the waypoint kind for you. Under a policy, the mode
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

One pause is **recoverable** and never raises:

| State | Meaning |
|-------|---------|
| `PAUSED_BY_USER` | A Pause/Stop request was honoured (the session clock is paused too) |

Note what is *not* in either table: there is no state for "the commanded
waypoints ran out". A drained queue keeps reporting `RUNNING` — the robot simply
brakes at the end of the last waypoint. Since NOVA 26.6 `PAUSED_BY_USER` follows
a Pause/Stop request only, and this SDK never sends one, so anything that needs
to know the robot came to rest measures it from the joint positions instead (see
[executor.md](executor.md)).

A NOVA instance without action chunk streaming raises `JoggingNotSupportedError` when the
session opens. The endpoint, `executeActionChunks`, exists on api-gateway 26.6 and newer; 26.5
reached the same feature through `executeWaypointJogging`, which 26.6 removed. Older gateways
answer the websocket upgrade with HTTP 404, which is reported as this error rather than as a
connection failure, so the fix (upgrade NOVA) is clear.

A session that loses either of its streams while it runs fails, and the next check raises
(`RuntimeError` naming the motion group, or the `OSError` itself): the action chunk stream ending
on any error (an error frame, a broken websocket, an exception in a waypoint module or stop
condition) or closed by the server, and the state stream ending, so a policy is never fed the
last state of a dead stream. Either ending during a requested stop is no failure.
