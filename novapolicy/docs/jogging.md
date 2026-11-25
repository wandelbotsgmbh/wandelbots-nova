# Jogging

Move a robot by streaming targets to NOVA, without a policy, schema or cameras. You open a
session, send joint or TCP targets, and NOVA handles velocity profiling, interpolation and
limits. Use it for scripted motion, teleoperation, or to test a cell before running a policy.

To run a trained policy instead, see [executor.md](executor.md).

## Quick start

`jog_joints()` and `jog_tcp()` return an async context manager. Iterate over it to get the robot
state about every 10 ms, and set a new target in each iteration.

### Joint jogging

```python
from novapolicy import jog_joints

HOME = [0, -1.57, 1.57, -1.57, -1.57, 0]

async with jog_joints(mg, start_joint_position=HOME) as jogger:
    async for state in jogger:
        jogger.set_target([0.0, -1.57, 1.57, -1.57, -1.57, 0.0])
```

### TCP jogging

```python
from nova.types import Pose
from novapolicy import jog_tcp

START = [1.17, -0.73, 1.75, -3.05, 0.87, 2.09]

async with jog_tcp(mg, tcp="Flange", start_joint_position=START) as jogger:
    async for state in jogger:
        jogger.set_target(Pose(500, 200, 300, 0, 3.14, 0))
```

`mg` is a motion group, for example `(await cell.controller("ur10e"))[0]`.
`start_joint_position` is optional. When given, the robot first moves point to point to that
position, so the session starts from a known pose. That move uses the cell's collision setups.

## Timing targets (`jogger.elapsed`)

For time-based motion such as a sine, compute the target from `jogger.elapsed`, not from your own
`time.monotonic()`. `elapsed` is seconds on the server's session clock since the first loop
iteration that had state. It is what every target is placed on, so motion computed from it
lines up with what the robot executes.

- It is sampled once per iteration: all targets you compute in one iteration share one time.
- It never runs backwards.
- It starts at `0.0` and does not wait for the robot to report `RUNNING`.

```python
import math

from novapolicy import jog_joints

async with jog_joints(mg) as jogger:
    async for _ in jogger:
        t = jogger.elapsed
        if t >= 5.0:
            break
        target = list(HOME)
        target[0] += 0.2 * math.sin(2 * math.pi * 0.25 * t)
        jogger.set_target(target)
```

## Live targets or chunks

There are two ways to send motion:

| method | what you send | use it when |
| --- | --- | --- |
| `set_target(target)` | one target: where the robot should be now | the target comes from a live source (a hand, a keyboard, a sensor) |
| `set_chunk(chunk, dt_ms=...)` | several future targets, `dt_ms` apart | you know where the robot should be over the next steps |

A live target is joint positions for `jog_joints` and a `Pose` for `jog_tcp`. A chunk is a list
of steps. For TCP jogging, each step is a raw `[x, y, z, rx, ry, rz]` list in mm and rad, not a
`Pose`.

Handing a chunk to `set_target`, or a single target to `set_chunk`, raises `TypeError`.

By default each live target goes out as a single waypoint, due `max_expected_latency_ms` after the
current `elapsed` (`ActionChunkConfig`, 100 ms by default). The robot steers toward it without knowing what
comes next, so a target that stops or reverses suddenly, such as a hand, can make it overshoot.
For a smooth path that brakes onto the target, add
[`ServoHorizon`](#following-a-hand-servohorizon) to the pipeline. To keep the robot moving between
targets, set `ActionChunkConfig.delay_until_braking_ms` (see
[Short chunks are padded](#short-chunks-are-padded)).

## Chunked targets

A chunk gives NOVA several future targets, so it can plan a smooth profile through them.

```python
async with jog_joints(mg) as jogger:
    async for state in jogger:
        t = jogger.elapsed
        # 8 future targets at 33 ms spacing
        chunk = [compute_target(t + i * 0.033) for i in range(8)]
        jogger.set_chunk(chunk, dt_ms=33.0)
```

`dt_ms` is the spacing between steps and is required.

Send chunks at a steady rate that leaves several future waypoints in the queue:

- If you replace a long chunk in every iteration, NOVA restarts its profile each time.
- If you send too rarely, the queue runs dry and the robot stops.

`examples/jogging/jogging_joints.py --chunked` sends an 8-step chunk at 33 ms spacing every
66 ms.

### Where a chunk lands

Step zero of a chunk is due `max_expected_latency_ms` after the current `elapsed` on the session
clock (`ActionChunkConfig`, 100 ms by default; raise it over a remote network, e.g. to 150). Content computed for trajectory time `T` always lands at the
same timestamp, however often you send it. Overlapping chunks therefore join into one
trajectory.

### Short chunks are padded

A chunk shorter than `HoldPadding.min_horizon_ms` (500 ms by default) is extended by repeating
its last target, so NOVA has enough horizon to brake within. The padding is not counted as
commanded motion: when you leave the session, it waits for your last step, not for the end of the
padding.

To turn padding off, leave `HoldPadding` out of the pipeline. A chunk whose last step is not a
real stopping point then ends with a deceleration to standstill.

To stream short chunks, down to a single waypoint, as one continuous motion, set
`ActionChunkConfig.delay_until_braking_ms`. NOVA then keeps the robot's velocity for that long after
a chunk's last waypoint before it brakes, and each new chunk restarts the countdown.

- It needs NOVA 26.8.0-112 or newer.
- For chunks, leave `HoldPadding` out of the pipeline, because a padded chunk ends in a hold.
  Live targets are never padded.
- For teleoperation over a network, start with about 200 ms.
- When the session closes, the last target is sent once more without the delay, so the robot
  stops on it.

```python
from novapolicy import ActionChunkConfig, jog_tcp

config = ActionChunkConfig(delay_until_braking_ms=200)
async with jog_tcp(mg, tcp="Flange", config=config) as jogger:
    async for state in jogger:
        jogger.set_target(read_controller_pose())
```

### Late chunks are trimmed

A chunk is timestamped when you build it and sent a moment later. Leading waypoints whose time
has already passed cannot be reached, so they are dropped at send time and the rest is sent. A
chunk whose waypoints have all passed is dropped. The waypoints that are sent keep their
timestamps, so the trajectory is not distorted.

## Following a hand (`ServoHorizon`)

In teleoperation the target is a hand, which can stop and reverse faster than the arm can brake.
`ServoHorizon` puts a spring-damper between the target and NOVA. It follows the newest target,
rounds off motion the arm cannot follow, and sends its own braking path as a 400 ms horizon.

```python
from novapolicy import ServoHorizon, default_pipeline, jog_tcp

pipeline = [ServoHorizon(), *default_pipeline()]
async with jog_tcp(mg, tcp="Flange", pipeline=pipeline) as jogger:
    async for state in jogger:
        jogger.set_target(read_hand_pose())
```

- With the defaults it settles without overshoot and follows a target at constant speed about
  `lookahead_s` (0.115 s) behind.
- It applies to `set_target` only. A chunk passes through unchanged and resets it.
- Run the client next to NOVA where you can. Network delay adds to the lag.

Its settings (gain, horizon, speed and acceleration limits) are described in
[pipeline.md](pipeline.md).

## The send pipeline (`pipeline=[...]`)

What `set_target` and `set_chunk` produce reaches NOVA through an ordered list of modules, the
same kind `PolicyExecutor` uses. Without `pipeline=` the joggers run `default_pipeline()`:
`HoldPadding()`, `Timestamps()`, `DropLate()`.

An explicit list is exactly what runs. To add a module, extend the defaults:

```python
from novapolicy import EaseIn, ServoHorizon, default_pipeline, jog_joints

async with jog_joints(
    mg, pipeline=[ServoHorizon(), *default_pipeline(), EaseIn(duration_s=1.0)]
) as jogger:
    ...
```

A motion that starts at full speed, such as a sine at `t = 0`, makes the robot lunge from a
standstill. `EaseIn(duration_s=1.0)` blends from where the robot stood into the motion over the
first second.

Each motion group runs its own copy of the pipeline. What each module does, and the order they
must be in, is in [pipeline.md](pipeline.md).

## Closing the session

When the jogging connection closes, the robot stops wherever it is on its path.

Leaving the `async with` block normally waits for the waypoints NOVA already accepted to finish,
so a clean exit does not cut motion short. It does not wait after a fault, an e-stop or a stop
condition, because there closing the connection is the stop. If NOVA stops reporting progress,
the wait gives up after 3 s and logs it.

**End your motion at rest.** Nothing in the session slows down for you. If the last waypoint
still has full velocity, the robot is at that velocity when the connection closes. Shape the
motion so it reaches zero velocity:

| motion | ease | why |
| --- | --- | --- |
| oscillation about a home pose | its **amplitude** | ramping the amplitude to zero returns to the home pose and stops there |
| a closed or fixed path (circle, line) | its **phase** | an amplitude envelope pulls the TCP off the path (to a circle's centre, for example); warping the phase keeps it on the path and only slows it |

Use a curve with zero first and second derivative at both ends, such as smootherstep,
`6x⁵ − 15x⁴ + 10x³`. Plain smoothstep (`3x² − 2x³`) zeroes the velocity but starts and ends at
peak acceleration, which makes the robot jerk. `EaseIn` uses smootherstep too.
`jogging_tcp.py` shows the phase warp, and `jogging_joints.py --chunked` the amplitude
envelope.

## Stop conditions

A stop condition is a function that gets a `StopContext` on every tick and returns `True` to end
the loop. The loop then ends normally, and `jogger.stop_condition_triggered` holds the function's
name. Keep it fast: it runs on every tick and must not block.

```python
from novapolicy import StopContext, jog_tcp


def too_low(ctx: StopContext) -> bool:
    return ctx.state.pose.position.z < 100.0


async with jog_tcp(mg, tcp="Flange", stop_conditions=[too_low]) as jogger:
    async for state in jogger:
        jogger.set_target(next_pose())

if jogger.stop_condition_triggered:
    print(f"stopped by {jogger.stop_condition_triggered}")
```

## Dual-arm

Pass several motion groups, then pass targets and chunks as a dict per motion group.

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

With several motion groups, the loop yields a dict of states, and `start_joint_position` is a
dict per motion group. Every motion group's value is checked before any is sent.

## Reference

### `jog_joints()` and `jog_tcp()`

```text
jog_joints(motion_groups, *, config=None, stop_conditions=None,
           start_joint_position=None, pipeline=None)
jog_tcp(motion_groups, *, tcp="", config=None, stop_conditions=None,
        start_joint_position=None, pipeline=None)
```

| parameter | type | default | meaning |
| --- | --- | --- | --- |
| `motion_groups` | `MotionGroup` or `list[MotionGroup]` (`jog_joints`); `MotionGroup` or `dict[MotionGroup, str]` (`jog_tcp`) | required | the robots to jog; for `jog_tcp` the dict maps each group to its TCP name |
| `tcp` | `str` | `""` | `jog_tcp` only: TCP name when you pass a single motion group |
| `config` | `ActionChunkConfig` or `None` | `ActionChunkConfig()` | session settings, see below |
| `stop_conditions` | `list[StopCondition]` or `None` | `None` | checks run on every tick; see [Stop conditions](#stop-conditions) |
| `start_joint_position` | `list[float]`, or a dict per motion group | `None` | joint positions to move to before the session starts |
| `pipeline` | list of modules or `None` | `default_pipeline()` | modules between your targets and NOVA, exactly as given |

### Jogger

| member | meaning |
| --- | --- |
| `async for state in jogger` | yields the current state about every 10 ms: a `RobotState`, or a dict per motion group |
| `set_target(target)` | send one live target, or a dict per motion group |
| `set_chunk(chunk, *, dt_ms)` | send a chunk of future targets, or a dict per motion group |
| `elapsed` | seconds on the session clock since the loop first had state |
| `target` | the last target sent (for a chunk, its last step) |
| `state()` | the current state, or `None` before the first state arrives |
| `stop_condition_triggered` | name of the stop condition that ended the loop, or `None` |

### `ActionChunkConfig`

| field | default | meaning |
| --- | --- | --- |
| `state_rate_ms` | `10` | state stream update rate in ms |
| `single_step_dt_ms` | `50.0` | spacing for a live target sent before the loop has state, and for a chunk with `dt_ms=0` |
| `max_expected_latency_ms` | `100.0` | how far after its reference a policy chunk's first step is due; see [executor.md](executor.md#setting-the-lead-for-your-network) |
| `delay_until_braking_ms` | `None` | how long NOVA keeps the velocity after a chunk's last waypoint before it brakes; `None` brakes at the end of every chunk |

### Waypoint kinds

Each waypoint carries either joint positions or a TCP pose:

| mode | waypoint kind | steps format | use case |
| --- | --- | --- | --- |
| `"joint"` | `JointWaypoint` (`JOINTS`) | joint radians `[j1, j2, ..., j6]` | joint space (default) |
| `"cartesian"` | `PoseWaypoint` (`POSE`) | TCP pose `[x, y, z, rx, ry, rz]` (mm + rad) | Cartesian space |

`jog_joints` and `jog_tcp` pick the kind for you. Under a policy, the mode is chosen from the
schema: TCP waypoints when it contains `Observation.tcp(..., action=True)`.

## Examples

In `examples/jogging/`. The joint and TCP examples drive robots already in the cell (`ur5e-left`
and `ur10e`; `ur5e-left` and `ur5e-right` with `--dual`). The other two create a virtual UR10e
of their own.

| script | what it shows |
| --- | --- |
| `jogging_joints.py` | joint 0 oscillates as live targets through `ServoHorizon`; `--chunked` sends 8-step chunks with an amplitude envelope, `--dual` drives two arms |
| `jogging_tcp.py` | the TCP traces a 50 mm circle, eased by its phase, as live poses through `ServoHorizon`; `--chunked` sends 100-step chunks at 10 ms, `--dual` two arms circling in opposite directions |
| `jogging_keyboard.py` | hold a key and the TCP moves: live targets on the default pipeline |
| `jogging_quest.py` | a mocked Meta Quest controller drives the TCP through `ServoHorizon`; `--script within\|beyond\|seam\|all` picks the motion, `--speed 0.5` plays it at half speed, `--raw` sends each pose alone |

With `viewer=nova.viewers.Rerun()` on the program, jogging is shown live in Rerun. See
[rerun.md](rerun.md).

## Error detection

The `async for` loop raises when something goes wrong:

| error | when |
| --- | --- |
| `MotionError` | NOVA paused the robot near a joint limit, a self-collision or a singularity, and the pause lasted several state updates |
| `EmergencyStopError` | the controller is in an e-stop, protective stop or another safety state |
| `JoggingNotSupportedError` | the NOVA instance has no action chunk streaming (raised when the session opens) |
| `RuntimeError` or `OSError` | the action chunk stream or the state stream ended while the session ran |
| `ValueError` | a target or step has the wrong number of values, or a value that is not finite |
| `TypeError` | a chunk passed to `set_target`, a single target passed to `set_chunk`, or a bare value with several motion groups |

The blocking pause states are:

| state | meaning |
| --- | --- |
| `PAUSED_NEAR_JOINT_LIMIT` | a joint reached its limit |
| `PAUSED_NEAR_COLLISION` | self-collision detected |
| `PAUSED_NEAR_SINGULARITY` | kinematic singularity |

`PAUSED_BY_USER` follows a pause or stop request (the session clock pauses too) and never
raises.

There is no state for "the waypoints ran out". When the queue is empty the robot brakes at the
last waypoint and the state stays `RUNNING`.

## Troubleshooting

| symptom | what to do |
| --- | --- |
| warning "step(s) were already due when sent and were dropped" | the network or the policy is slower than the lead: raise `ActionChunkConfig(max_expected_latency_ms=...)`, e.g. to 150 over a remote network; at the stop the log counts how many steps were dropped |
| `JoggingNotSupportedError` when the session opens | upgrade NOVA: action chunk streaming needs api-gateway 26.6 or newer |
| the robot brakes toward every live target and moves in jerks | add `ServoHorizon` to the pipeline, or send chunks |
| the robot overshoots and rings when a hand stops in teleoperation | use `ServoHorizon` instead of sending each pose alone |
| the robot lunges at the start of a time-based motion | add `EaseIn(duration_s=1.0)` at the end of the pipeline, or start the motion at zero velocity |
| the robot keeps moving or stops abruptly at the end | shape the motion to end at rest; see [Closing the session](#closing-the-session) |
| chunks make the arm swing back and forth around the set speed | send chunks less often, or use live targets; do not replace a long chunk every iteration |
| the robot stops between chunks | send the next chunk before the previous one ends, or set `delay_until_braking_ms` |
| `delay_until_braking_ms` has no effect | remove `HoldPadding` from the pipeline, and check the NOVA version (26.8.0-112 or newer) |
| `MotionError` near a joint limit, collision or singularity | move the target away from that region, or start from another `start_joint_position` |
| `TypeError` with several motion groups | pass a dict per motion group to `set_target` and `set_chunk` |
