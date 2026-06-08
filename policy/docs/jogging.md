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
from policy import jog_joints

HOME = [0, -1.57, 1.57, -1.57, -1.57, 0]

async with jog_joints(mg, start_joint_position=HOME) as jogger:
    async for state in jogger:
        # Single target (server interpolates from current position)
        jogger.set_target([0.0, -1.57, 1.57, -1.57, -1.57, 0.0])
```

## TCP jogging

```python
from policy import jog_tcp
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

## Dual-arm

```python
from policy import jog_joints, jog_tcp

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
