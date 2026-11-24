# Rerun visualization

See policy execution and jogging live in [Rerun](https://rerun.io): the robot in 3D, the
commanded path, camera images, and plots of commanded against actual motion. Use it to check
what a policy commands and how well the robot follows.

Visualization is optional. It runs only when the program has a Rerun viewer. Without one,
nothing is logged and nothing is loaded.

## Setup

Install the Rerun extra:

```bash
pip install "wandelbots-nova[nova-rerun-bridge]"
```

Add `viewer=nova.viewers.Rerun()` to the `@nova.program` decorator. `PolicyExecutor`,
`jog_joints` and `jog_tcp` then log to the viewer on their own.

```python
import nova
from novapolicy import PolicyExecutor, SequentialExecution


@nova.program(
    id="my_policy",
    viewer=nova.viewers.Rerun(),
)
async def run(ctx):
    ...
    executor = PolicyExecutor(
        schema,
        policy,
        execution=SequentialExecution(),
        timeout_s=10.0,
    )
    await executor.run()  # data streams to the Rerun viewer
```

Jogging works the same way:

```python
import nova
from novapolicy import jog_joints


@nova.program(id="jog", viewer=nova.viewers.Rerun())
async def run(ctx):
    mg = (await ctx.nova.cell().controller("ur10e"))[0]
    async with jog_joints(mg) as jogger:
        ...
```

The policy data goes to its own recording, `novapolicy`, next to NOVA's planning recording.

## What you see

| panel | shows |
| --- | --- |
| 3D View | the robot model from the measured state, the TCP marker and trail, and each action chunk as a TCP path: the steps that run as a colored line, the discarded tail of a replaced chunk in dim gray |
| camera panels | one panel per camera in the schema, JPEG-compressed |
| Joints target/actual `<motion group>` | commanded and measured joint positions |
| TCP target/actual `<motion group>` | commanded and measured TCP position and orientation |
| TCP position error `<motion group>` [mm] | commanded minus measured TCP position, per axis, and its norm `position_norm_mm` |
| TCP orientation error `<motion group>` [rad] | commanded minus measured TCP orientation, and its norm `orientation_norm_rad` |
| Action Chunks | a text log of each action chunk and the run status |

The plots use the `policy_time` timeline. Camera panels appear only when the schema has cameras.

Commanded against measured is logged once per state packet, at the time the packet was
generated, for the executor and the joggers alike. The robot model moves at most 30 times a
second. Logging does not change how often the policy runs or targets are sent.

## Reading the tracking plots

- Read `position_norm_mm` and `orientation_norm_rad` to judge whether tracking is steady.
- A constant time lag on a curved path shows as sine-shaped `dx`, `dy` and `dz` with a
  **constant** `position_norm_mm`. The error vector turns with the path while its size stays the
  same. That is a steady lag, not a tracking problem.
- Position and orientation errors are in separate panels because they have different units.

### Which TCP joint targets are drawn at

For a joint-space policy, the commanded joints are converted to a TCP position for the plots:

- `PolicyExecutor` uses the schema's TCP, or the controller's active TCP when the schema names
  none.
- `jog_joints` uses the controller's active TCP.

If that TCP differs from the one the robot reports, the plots show a constant offset of about
the tool's size.

## Options

| setting | where | meaning |
| --- | --- | --- |
| `trajectory_sample_interval_ms` | `nova.viewers.Rerun(...)` | how densely NOVA's viewer samples planned trajectories (default 50 ms) |
| `show_collision_link_chain`, `show_collision_tool` | `nova.viewers.Rerun(...)` | whether the robot is drawn with its collision geometry |

## Cameras

With `WebRTCCameras`, Rerun shows the latest frames between policy chunks, about 15 times a
second. Other camera sources are shown at the rate the policy observes them, unless they provide
a `get_latest_frame(max_age_s=...)` method.

## Troubleshooting

| symptom | what to do |
| --- | --- |
| nothing is logged | add `viewer=nova.viewers.Rerun()` to `@nova.program`, and install `wandelbots-nova[nova-rerun-bridge]` |
| the robot model is missing, but paths and markers are drawn | the robot's model could not be loaded; the run continues without it |
| a constant error of about the tool's size | the plots use another TCP than the robot reports; see [Which TCP joint targets are drawn at](#which-tcp-joint-targets-are-drawn-at) |
| a sine-shaped position error | check `position_norm_mm`; if it is constant, the robot follows with a steady lag |
| some entries are missing when the viewer is busy | expected: under load the oldest pending entries are dropped to keep the view current |
| camera panels update slowly | use `WebRTCCameras`, or give the camera source a `get_latest_frame(max_age_s=...)` method |
