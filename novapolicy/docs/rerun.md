# `novapolicy.rerun` — visualization (optional)

Live [Rerun](https://rerun.io) logging for policy execution. **Entirely optional
and zero-cost when no viewer is active** — the executor checks `_is_rerun_active()`
and skips all logging if Rerun isn't initialized, so nothing here runs in production.

## Usage

Add `viewer=nova.viewers.Rerun()` to the `@nova.program` decorator to get
real-time 3D visualization of the execution. The executor automatically logs
robot meshes, action chunk TCP paths, TCP trails, camera images, and joint
timeseries — zero overhead when no viewer is active.

```python
from nova import viewers
from novapolicy import SequentialExecution


@nova.program(
    id="my_policy",
    viewer=viewers.Rerun(state_sample_interval_ms=10.0),  # 100 Hz live state
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

Requires `wandelbots-nova[nova-rerun-bridge]`. Run `uv run download-models` once
to fetch robot meshes.

## What it logs

| Module            | Logs                                                                                                                                                    |
| ----------------- | ------------------------------------------------------------------------------------------------------------------------------------------------------- |
| `observation.py`  | Robot joint state per step (drives the 3D robot model).                                                                                                 |
| `action_chunk.py` | The action chunk as a 3D TCP path (executed steps as a gradient line strip, discarded receding-horizon tail in dim gray) plus an inspectable text dump. |
| `images.py`       | JPEG-compressed camera frames.                                                                                                                          |
| `streaming.py`    | Background task that streams robot state at the viewer's configured rate (30 Hz by default) and the latest camera frames at 15 Hz.                       |
| `blueprint.py`    | The viewer layout (panels for 3D scene, cameras, action text).                                                                                          |
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

`state_sample_interval_ms` controls actual robot-state samples used by the 3D mesh,
TCP trail, and joint plots. It does not change the policy or jogging command cadence;
configure policy timing or `WaypointConfig` separately when commands themselves need
a shorter interval. `trajectory_sample_interval_ms` independently controls sampling
of planned trajectories.
