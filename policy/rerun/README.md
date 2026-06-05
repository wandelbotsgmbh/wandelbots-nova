# `policy.rerun` — visualization (optional)

Live [Rerun](https://rerun.io) logging for policy execution. **Entirely optional
and zero-cost when no viewer is active** — the executor checks `_is_rerun_active()`
and skips all logging if Rerun isn't initialized, so nothing here runs in production.

## What it logs

| Module            | Logs                                                                                                                                                    |
| ----------------- | ------------------------------------------------------------------------------------------------------------------------------------------------------- |
| `observation.py`  | Robot joint state per step (drives the 3D robot model).                                                                                                 |
| `action_chunk.py` | The action chunk as a 3D TCP path (executed steps as a gradient line strip, discarded receding-horizon tail in dim gray) plus an inspectable text dump. |
| `images.py`       | Camera frames.                                                                                                                                          |
| `streaming.py`    | Background task that streams live robot state into the viewer.                                                                                          |
| `blueprint.py`    | The viewer layout (panels for 3D scene, cameras, action text).                                                                                          |
| `logger.py`       | `PolicyRerunLogger` — the single entry point the executor talks to; ties the above together.                                                            |
| `constants.py`    | Colors / widths / thresholds for the chunk visuals.                                                                                                     |

## Enabling it

Start a Rerun viewer before running the executor (e.g. via `nova.viewers`). When a
viewer is active, `PolicyExecutor` lazily constructs a `PolicyRerunLogger` and streams
observations, action chunks, and camera frames to it. No viewer → none of this loads.
