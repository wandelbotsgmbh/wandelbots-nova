# `novapolicy.rerun` — visualization (optional)

Live [Rerun](https://rerun.io) logging for policy execution. **Entirely optional
and zero-cost when no viewer is active** — the executor checks `_is_rerun_active()`
and skips all logging if Rerun isn't initialized, so nothing here runs in production.

## Usage

Add `viewer=nova.viewers.Rerun` to the `@nova.program` decorator to get
real-time 3D visualization of the execution. The executor automatically logs
robot meshes, action chunk TCP paths, TCP trails, camera images, and joint
timeseries — zero overhead when no viewer is active.

`viewer=viewers.Rerun()` remains supported for compatibility, but its implicit global
registration is deprecated and will be removed in the next major release. Prefer a factory as
shown below so importing the program has no viewer side effects and every run gets fresh state.

```python
from nova import viewers
from novapolicy import SequentialExecution


@nova.program(
    id="my_policy",
    viewer=lambda: viewers.Rerun(
        state_sample_interval_ms=10.0  # 100 Hz live state
    ),
)
async def run(ctx):
    ...
    executor = PolicyExecutor(
        schema,
        policy,
        execution=SequentialExecution(),
        timeout_s=10.0,
    )
    await executor.run()  # data streams to Rerun viewer automatically
```

Requires `wandelbots-nova[nova-rerun-bridge]`.

## What it logs

| Module            | Logs                                                                                                                                                    |
| ----------------- | ------------------------------------------------------------------------------------------------------------------------------------------------------- |
| `observation.py`  | Robot joint state per step (drives the 3D robot model).                                                                                                 |
| `action_chunk.py` | The action chunk as a 3D TCP path (executed steps as a gradient line strip, discarded receding-horizon tail in dim gray) plus an inspectable text dump. |
| `images.py`       | JPEG-compressed camera frames.                                                                                                                          |
| `streaming.py`    | Background task that streams robot state at the viewer's configured rate (30 Hz by default) and the latest camera frames at 15 Hz.                       |
| `target_tracking.py` | Commanded vs. actual joints and TCP pose, the derived position/orientation error, and the target trail. |
| `kinematics.py`   | Forward kinematics, used to turn a commanded joint target into a TCP position for the error plots.       |
| `blueprint.py`    | The viewer layout (panels for 3D scene, cameras, plots, action text).                                                                                          |
| `sink.py`         | `AsyncLogSink` — runs submitted writes on a worker thread so the producing loop does not pay for them.   |
| `logtime.py`      | Thread-local timestamp pinning, so an entry written off-thread is stamped when it was produced.          |
| `logger.py`       | `PolicyRerunLogger` — the single entry point the executor talks to; ties the above together.                                                            |
| `constants.py`    | Colors / widths / thresholds for the chunk visuals.                                                                                                     |

## Enabling it

Start a Rerun viewer before running the executor (e.g. via `nova.viewers`). When a
viewer is active, `PolicyExecutor` lazily constructs a `PolicyRerunLogger` and streams
observations, action chunks, and camera frames to it. No viewer → none of this loads.

Rerun reads the latest WebRTC frames between policy chunks. Other camera backends
continue to log at policy-observation cadence unless they expose a compatible
`get_latest_frame(max_age_s=...)` method. Camera images are JPEG-compressed before transport
to keep the live viewer responsive.

## Writes do not run on the producing loop

Rerun logging is not free: one tick's worth of tracking and action-chunk entries
costs a meaningful fraction of a 10 ms control tick, and occasionally a multiple of
one. Paid on a jogging control loop that also has to produce the next waypoint
chunk, that lengthens ticks well past their budget and costs motion smoothness.

Writes are therefore submitted to `AsyncLogSink` and executed on a worker thread.
The Rerun SDK serialises in Rust and releases the GIL, so this moves the cost off
the producing thread rather than shuffling it around. Two consequences:

- Entries carry the timestamp measured when they were **submitted**, pinned
  thread-locally by `logtime.py`, so a backlog on the worker does not stretch the
  timeline. Rerun's built-in `log_time` is stamped from the same instant, so the
  timelines agree.
- The queue is bounded and drops its **oldest** entries on overflow — showing the
  present matters more than showing everything here. Drops are counted and logged
  at debug level on close.

The sink is drained before the recording is disconnected, so the tail of a run is
not lost.

## Tracking error is logged per state packet

Commanded-vs-actual is logged once per state packet, at the instant that packet was
generated, rather than once per control tick. The two run at different rates, and
state packets can arrive in bursts: sampling the cached pose once per tick re-reads
the same packet for tens of milliseconds and discards the rest, which draws the
error as a repeating flat shelf. The commanded value is resolved for *that packet's*
own server timestamp, so both sides of the difference describe the same instant.

`tcp_error` is split across two views, `TCP position error [mm]` and
`TCP orientation error [rad]`. A time-series view shares one Y axis, so plotting
millimetres and radians together flattens the radian series onto zero, where it
reads as "no error" whatever its value.

When reading those plots: a constant time lag on a curved path appears as
sinusoidal `dx`/`dy`/`dz` with a **constant** `position_norm_mm`, because the error
vector rotates with the path while its magnitude stays put. The norm is the series
to read for whether tracking is steady.

`state_sample_interval_ms` controls actual robot-state samples used by the 3D mesh,
TCP trail, and joint plots. It does not change the policy or jogging command cadence;
configure policy timing or `WaypointConfig` separately when commands themselves need
a shorter interval. `trajectory_sample_interval_ms` independently controls sampling
of planned trajectories.
