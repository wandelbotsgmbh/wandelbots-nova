# GR00T single-arm pick and place

Runs an IsaacLab-trained GR00T policy ("pick up the cube and place it on the yellow
square") on the real UR10e `ur10e-calibration` with a Zimmer gripper.

| Script | What it does |
|---|---|
| `run_sequential.py` | Policy, stop-and-go (the default mode): runs one chunk to standstill, then observes and infers again |
| `run_continuous.py` | Policy, streaming: re-infers as fast as the server answers, chunks merged on a timestep queue |
| `gr00t_ur10e.py` | Shared implementation of the two `run_*` scripts: the policy's facts, the cell and the limits as constants at the top, the schema and the run |
| `zimmer_gripper.py` | The Zimmer gripper as `Observation.computed` / `Action.computed` hooks, and the hold that keeps the arm still while the jaws move |
| `episode_settings.json`, `episode_settings.py` | Measured from training episode 75: rate, home, the deltas' coordinate system, the gains and the gripper values; `gr00t_ur10e.py` loads it |

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

```

## Where the settings come from

[`episode_settings.json`](episode_settings.json) is measured, not typed. Episode 75's recorded
joints, run through the robot's forward kinematics, give the path the training robot moved;
comparing it with the recorded actions gave what each action means (here TCP deltas in the world
frame), the scale between action units and realized motion (the training controller realized only
part of each command), the gripper polarity and values, the rate and the home joints.
[`episode_settings.py`](episode_settings.py) loads it, and `RobotSettings.action(mg)` builds the
matching `Action`. Two values cannot be measured and were set by hand: `tcp`, the point on the
tool the training poses refer to, and `gripper_closed`, the fully closed gripper state (the
fingers stop on the cube first).

What is left as constants at the top of [`gr00t_ur10e.py`](gr00t_ur10e.py) is what no episode
contains: the policy server, the prompt, the image size, the cameras and the gripper's address. See [the GR00T setup
guide](../../docs/gr00t.md#setting-up-a-policy-on-a-real-robot) for the whole workflow.
