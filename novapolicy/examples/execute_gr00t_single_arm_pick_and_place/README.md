# GR00T single-arm pick and place

Runs an IsaacLab-trained GR00T policy ("pick up the cube and place it on the yellow
square") on the real UR10e `ur10e-calibration` with a Zimmer gripper, and replays the
policy's training episodes on the same cell.

| Script | What it does |
|---|---|
| `run_sequential.py` | Policy, stop-and-go (the default mode): runs one chunk to standstill, then observes and infers again |
| `run_continuous.py` | Policy, streaming: re-infers as fast as the server answers, chunks merged on a timestep queue |
| `gr00t_ur10e.py` | Shared implementation of the two `run_*` scripts: the policy's facts, the cell and the limits as constants at the top, the schema and the run |
| `zimmer_gripper.py` | The Zimmer gripper as `Observation.computed` / `Action.computed` hooks, and the hold that keeps the arm still while the jaws move |
| `episode_settings.json` | Written by [`../settings_from_episode.py`](../settings_from_episode.py) for episode 75: rate, home, the deltas' coordinate system, the gains and the gripper values; `gr00t_ur10e.py` loads it |

Without `--execute` every script is a dry run: nothing moves. With `--execute` it asks
for `yes` before the gripper or the arm moves. Keep a hand on the e-stop.

## Setup

From the repository root, once:

```bash
uv sync --extra novapolicy-gr00t --extra nova-rerun-bridge
uv pip install --python .venv <zimmer-gripper-controller>   # path or git URL; only for --execute
export NOVA_API=http://<nova-host>
```

## Run

```bash
# Policy
uv run python novapolicy/examples/execute_gr00t_single_arm_pick_and_place/run_sequential.py [--execute]
uv run python novapolicy/examples/execute_gr00t_single_arm_pick_and_place/run_continuous.py [--execute]

# Training-episode replay, with the generic replay script
uv run python novapolicy/examples/replay/replay_episode.py --episode <episode_000075.parquet> \
    --controller ur10e-calibration --joints-from state --gripper-column 6 \
    --gripper zimmer:172.31.13.49 --tcp umi_corrected --lift-to-mm 200 --speed 0.4 \
    [--stop-at-grip] [--execute]
```

The episode's joints are in `observation.state` (its actions are end-effector deltas) and
the gripper command in `action[6]`. `--stop-at-grip` replays up to the recorded grasp and
stops with the jaws open, so the cube can be placed between them; run again without it to
replay the whole episode. `--speed 0.4` is ~90 mm/s (the recording: median 229 mm/s).

## Where the settings come from

[`episode_settings.json`](episode_settings.json) is measured, not typed: rate, home joints, the
deltas' coordinate system, the training controller's gains and the gripper values. Regenerate it
for another policy or episode (nothing moves):

```bash
uv run python novapolicy/examples/settings_from_episode.py \
    --episode <dataset>/data/chunk-000/episode_000075.parquet \
    --modality <dataset>/meta/modality.json \
    --robot arm mg=0@ur10e-calibration tcp=umi_corrected joints=arm_joints action=eef_delta \
            gripper=gripper gripper_closed=-0.023 \
    --write novapolicy/examples/execute_gr00t_single_arm_pick_and_place/episode_settings.json
```

The script works for any number of robots: pass `--robot` once per arm (for a dual-arm policy,
`--robot left ...` and `--robot right ...`). For each robot it measures whether the actions are
absolute joint targets, joint deltas, absolute TCP targets or TCP deltas, and the scale between
action units and motion, so metres vs. millimetres or radians vs. degrees need not be declared;
`RobotSettings.action(mg)` builds the matching `Action`.

It stops instead of writing when a fit is poor, two readings fit about equally well, or the episode
cannot tell. Per robot, two inputs cannot be measured: `tcp=`, the point on the tool the training poses refer to (every TCP of this robot fits
episode 75 about equally well, because the tool barely rotates), and `gripper_closed=`, the fully
closed gripper state (the training config's close command; the fingers stop on the cube first).

What is left as constants at the top of [`gr00t_ur10e.py`](gr00t_ur10e.py) is what no episode
contains: the policy server, the prompt, the image size, the cameras and the gripper's address. See [the GR00T setup
guide](../../docs/gr00t.md#setting-up-a-policy-on-a-real-robot) for the whole workflow.
