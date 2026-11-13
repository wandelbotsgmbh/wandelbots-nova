# GR00T single-arm pick and place

Runs an IsaacLab-trained GR00T policy ("pick up the cube and place it on the yellow
square") on the real UR10e `ur10e-calibration` with a Zimmer gripper, and replays the
policy's training episodes on the same cell.

| Script | What it does |
|---|---|
| `run_continuous.py` | Policy, streaming: re-infers as fast as the server answers, chunks merged on a timestep queue |
| `run_sequential.py` | Policy, stop-and-go: runs one chunk to standstill, then observes and infers again |
| `replay_episode.py` | Replays a training episode's recorded joint path, gripping and releasing at the recorded frames |
| `gr00t_ur10e.py` | Shared implementation of the two `run_*` scripts |

Without `--execute` every script is a dry run: nothing moves. With `--execute` it asks
for `yes` before the gripper or the arm moves. Keep a hand on the e-stop.

## Setup

From the repository root, once:

```bash
uv sync --extra novapolicy-gr00t --extra nova-rerun-bridge
uv pip install <zimmer-gripper-controller>   # path or git URL; only needed with --execute
export NOVA_API=http://<nova-host>
```

## Run

```bash
# Policy
uv run python novapolicy/examples/gr00t_single_arm_pick_and_place/run_continuous.py [--execute]
uv run python novapolicy/examples/gr00t_single_arm_pick_and_place/run_sequential.py [--execute]

# Training-episode replay (episode parquet from the LeRobot training dataset)
uv run python novapolicy/examples/gr00t_single_arm_pick_and_place/replay_episode.py \
    --episode <episode_000075.parquet> [--show-grasp | --execute]
```

`--show-grasp` stops at the recorded grasp pose with the jaws open, so the cube can be
placed between them before `--execute` replays the episode.

## Cell configuration

The constants at the top of `gr00t_ur10e.py` (and `replay_episode.py`) describe the cell:
controller and TCP (`umi_corrected`), the GR00T server, the RealSense camera server and
serial numbers (top, side, wrist), and the gripper's IO-Link master. Speed and safety
limits sit next to them: `DT_MS`, the workspace box, `MIN_TCP_Z_MM`, `MAX_TILT_DEG`.
