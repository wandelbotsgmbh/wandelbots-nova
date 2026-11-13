# GR00T single-arm pick and place

Runs an IsaacLab-trained GR00T policy ("pick up the cube and place it on the yellow
square") on the real UR10e `ur10e-calibration` with a Zimmer gripper, and replays the
policy's training episodes on the same cell.

| Script | What it does |
|---|---|
| `run_sequential.py` | Policy, stop-and-go (the default mode): runs one chunk to standstill, then observes and infers again |
| `run_continuous.py` | Policy, streaming: re-infers as fast as the server answers, chunks merged on a timestep queue |
| `gr00t_ur10e.py` | Shared implementation of the two `run_*` scripts: the policy's facts, the cell and the limits as constants at the top, the schema, the Zimmer gripper and the run |
| `fit_episode.py` | Measures from a training episode what the actions mean (frame, gains, gripper polarity) and the training envelope |

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
uv run python novapolicy/examples/gr00t_single_arm_pick_and_place/run_sequential.py [--execute]
uv run python novapolicy/examples/gr00t_single_arm_pick_and_place/run_continuous.py [--execute]

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

The constants at the top of [`gr00t_ur10e.py`](gr00t_ur10e.py) come from the training setup
(checkpoint-14000, IsaacLab `UR10ePlaceCubeEnvCfg`, dataset `ur10e_place_lerobot_unmodified`) and
the cell. The ones the training config does not state, checked against a training episode
(nothing moves):

```bash
uv run python novapolicy/examples/gr00t_single_arm_pick_and_place/fit_episode.py \
    --episode <episode_000075.parquet>
```

It prints the delta frame, the position and rotation gains, the gripper polarity and values, and
the training envelope the limits should cover. See [the GR00T setup guide](../../gr00t/README.md)
for the whole workflow.
