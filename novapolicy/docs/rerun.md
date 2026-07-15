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

@nova.program(id="my_policy", viewer=viewers.Rerun())
async def run(ctx):
    ...
    executor = PolicyExecutor(schema, policy, timeout_s=10.0)
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
| `streaming.py`    | Background task that streams robot state at 30 Hz and side-effect-free camera previews at 15 Hz.                                                       |
| `blueprint.py`    | The viewer layout (panels for 3D scene, cameras, action text).                                                                                          |
| `logger.py`       | `PolicyRerunLogger` — the single entry point the executor talks to; ties the above together.                                                            |
| `constants.py`    | Colors / widths / thresholds for the chunk visuals.                                                                                                     |

## Enabling it

Start a Rerun viewer before running the executor (e.g. via `nova.viewers`). When a
viewer is active, `PolicyExecutor` lazily constructs a `PolicyRerunLogger` and streams
observations, action chunks, and camera frames to it. No viewer → none of this loads.

WebRTC cameras provide a side-effect-free preview read, so Rerun receives live images
between policy chunks without advancing temporal frame-history buffers used by the model.
Other camera backends continue to log at policy-observation cadence unless they expose a
compatible `get_latest_frame(max_age_s=...)` method. Camera images are JPEG-compressed before
transport to keep the live viewer responsive.
