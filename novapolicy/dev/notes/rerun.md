# Notes moved from docs/rerun.md

Material moved out of the customer guide [docs/rerun.md](../../docs/rerun.md): the module layout,
how activation is wired, why writes run off the loop, and the worker, view and frame details
behind [ADR 0020](../adr/0020-rerun-off-the-loop-in-the-robots-frame.md). The text is kept as it
was, with links fixed for this folder.

## Activation

Live [Rerun](https://rerun.io) logging for policy execution and jogging. **Entirely optional
and zero-cost when no viewer is active** — the session lifecycle checks `_is_rerun_active()`
when the executor or a jogger starts and skips all logging if Rerun isn't initialized, so
nothing here runs in production.

Start a Rerun viewer before running the executor or a jogger (e.g. via `nova.viewers`). When
a viewer is active, the session lifecycle (`SessionLifecycle.start_monitoring`, shared by
`PolicyExecutor` and the joggers) constructs a `PolicyRerunLogger` and streams observations,
action chunks, and camera frames to it. No viewer → none of this loads.

## What it logs, by module

| Module            | Logs                                                                                                                                                    |
| ----------------- | ------------------------------------------------------------------------------------------------------------------------------------------------------- |
| `observation.py`  | Measured robot state: the 3D robot model, joint scalars, the TCP marker and trail.                                                                      |
| `action_chunk.py` | The action chunk as a 3D TCP path (executed steps as a gradient line strip, discarded receding-horizon tail in dim gray) plus an inspectable text dump. |
| `images.py`       | JPEG-compressed camera frames.                                                                                                                          |
| `streaming.py`    | Background task that logs controller timing (server timestamp, each scheduled waypoint request) about 30 times a second and the latest camera frames at 15 Hz; it draws the robot state the logger hands it. |
| `target_tracking.py` | Commanded vs. actual joints and TCP pose, the derived position/orientation error, and the target trail. |
| `kinematics.py`   | Forward kinematics and frame helpers: a commanded joint target as a TCP position for the error plots, reported poses moved into the robot's frame. |
| `context.py`      | `RerunContext` — what every entry is logged against (recording, timelines, robots, frames) and the drawing helpers the modules share. |
| `blueprint.py`    | The viewer layout (panels for 3D scene, cameras, plots, action text).                                                                                          |
| `sink.py`         | `AsyncLogSink` — runs submitted writes on a worker thread so the producing loop does not pay for them.   |
| `logtime.py`      | Thread-local timestamp pinning, so an entry written off-thread is stamped when it was produced.          |
| `logger.py`       | `PolicyRerunLogger` — the single entry point the executor talks to; ties the above together.                                                            |
| `constants.py`    | Colors / widths / thresholds for the chunk visuals.                                                                                                     |

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

## Tracking error is logged per state packet: why

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

Commanded-vs-measured tracking is logged once per state packet the session receives, for the
executor and the jogger alike. The robot state (the 3D mesh, joint plots, TCP marker and trail)
comes from the same packets, at most once per 1/30 s of their generation time
(`STATE_SAMPLE_PERIOD_S`). Neither changes the policy or jogging command cadence.

## Worker, views and frames: the details

Why Rerun works the way [ADR 0020](../adr/0020-rerun-off-the-loop-in-the-robots-frame.md) decided:

- **Closing the sink.** Close sets a stop flag rather than relying on a sentinel, since a full
  queue may have no room for one; the worker drains, then stops. A write submitted after close runs
  inline, under a lock, because the worker may still be mid-write on the same trails. A failed
  entry is skipped, which is why the tracking calls also guard their snapshot on the caller's
  thread.
- **Time stamps.** An instant from before the session start (a zero-initialised value is machine
  boot on the monotonic clock) is stamped "now" instead of dragging the axis days back. Plots show
  `policy_time`; left unset, a view falls back to the selected timeline, in practice `log_time`.
  Each norm series is listed explicitly in its view, since the norms are siblings of the component
  groups.
- **The robot mesh** moves at most once per sample period of generation time, so it moves evenly
  however packets arrived.
- **A dedicated recording** (`novapolicy`) holds the policy data and its blueprint, so NOVA
  planning keeps its own. The program's own viewer settings decide how the robot is drawn
  (`show_collision`).
- **One visualizer per controller.** A robot is one kinematic chain even where the API splits it
  into motion groups, and an arm cannot be placed without the part it rides on; not one per
  recording either, since a cell can hold several robots. The URDF is the only rendering path;
  when it cannot be had, the run continues without the mesh and the overlays still draw.
- **Overlays in the robot's frame.** A URDF robot sits in Rerun's frame graph, and the 3D view
  drops anything with no path to its root, so each overlay entity declares its own robot's frame
  (on the entity itself: a frame on the parent path does nothing for its children). Each motion
  group's DH table starts at its own base and the controller reports `mounting = [0, 0, 0]`, so
  the overlay `DHRobot`s are re-based where the visualizer places each chain, and reported poses,
  relative to the chain's mount, are moved into the robot's frame. Without either, trails and
  markers land about a metre from the TCP they belong to.
- **The commanded TCP is the driven one.** Measured poses come back at the TCP the caller drives,
  so a joint target's TCP position uses that TCP's offset, not the controller's active one; a
  different offset shows a constant tool-sized error that reads as the policy being wrong. Unless
  the logger is asked for a TCP offset (`use_tcp_offset_for_joint_actions`), joint chunks are
  drawn at the flange: a schema with joint actions declares no TCP, and silently applying the
  active one's offset can displace the marker from what is sent. The session lifecycle asks for
  it, so the executor draws joint targets at the schema's TCP, or at the controller's active
  one when the schema names none; the joint jogger draws them at the active one.
