# GR00T single-arm pick and place

Runs an IsaacLab-trained GR00T policy ("pick up the cube and place it on the yellow
square") on the real UR10e `ur10e-calibration` with a Zimmer gripper, and replays the
policy's training episodes on the same cell.

| Script | What it does |
|---|---|
| `run_sequential.py` | Policy, stop-and-go (the default mode): runs one chunk to standstill, then observes and infers again |
| `run_continuous.py` | Policy, streaming: re-infers as fast as the server answers, chunks merged on a timestep queue |
| `replay_pick_and_place.py` | Replays a training episode's recorded joint path with the Zimmer gripper gripping and releasing at the recorded frames (a wrapper around the generic [`../replay/replay_episode.py`](../replay/replay_episode.py)) |
| `policy.toml` | Everything declared about the policy, the cell and the guards; the scripts read it |
| `gr00t_ur10e.py` | Shared implementation of the two `run_*` scripts |

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

# Training-episode replay (an episode parquet of the GR00T training dataset)
uv run python novapolicy/examples/gr00t_single_arm_pick_and_place/replay_pick_and_place.py \
    --episode <episode_000075.parquet> [--show-grasp | --execute] [--speed 0.4]
```

`--show-grasp` replays up to the recorded grasp and stops with the jaws open, so the cube
can be placed between them before `--execute` replays the whole episode. The replay runs
at `--speed` times the recorded rate (0.4 by default, ~90 mm/s).

## The policy config

[`policy.toml`](policy.toml) declares everything the scripts use: the policy's facts
(checkpoint-14000, IsaacLab `UR10ePlaceCubeEnvCfg`, dataset `ur10e_place_lerobot_unmodified`),
the cell (controller, TCP `umi_corrected`, cameras, gripper) and the guards. Check it against
the server, the checkpoint, the training data and the robot (nothing moves):

```bash
uv run python -m novapolicy.gr00t.inspect_policy \
    --config novapolicy/examples/gr00t_single_arm_pick_and_place/policy.toml \
    --data <folder with checkpoint-14000 and the ur10e_place_lerobot_unmodified dataset>
```

The run scripts also check the server against it at every start. Execution settings that
are not part of the policy (speed factor, chunk length, gripper force) stay at the top of
`gr00t_ur10e.py`. See [the GR00T setup guide](../../gr00t/README.md) for the whole workflow.
