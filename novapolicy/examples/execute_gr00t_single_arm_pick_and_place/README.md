# GR00T single-arm pick and place

Runs an IsaacLab-trained GR00T policy ("pick up the cube and place it on the yellow square") on a
real UR10e with a Zimmer gripper. The policy runs behind a GR00T inference server. The general
workflow is in [the GR00T guide](../../docs/gr00t.md#setting-up-a-policy-on-a-real-robot).

| File | What it does |
| --- | --- |
| `run_sequential.py` | Runs the policy one chunk at a time: each chunk to standstill, then observe and infer again. Start here. |
| `run_continuous.py` | Runs the policy continuously: infers again as fast as the server answers, chunks merged on a timestep queue. |
| `gr00t_ur10e.py` | Shared code of the two `run_*` scripts: the constants for the cell, the schema and the run. |
| `zimmer_gripper.py` | The Zimmer gripper as `Observation.computed` / `Action.computed` hooks, and the hold that keeps the arm still while the jaws move. |
| `episode_settings.json`, `episode_settings.py` | What the policy's actions mean, read from a training episode; `gr00t_ur10e.py` loads it. |

## Setup

From the repository root, once:

```bash
uv sync --extra novapolicy-gr00t --extra nova-rerun-bridge
uv pip install --python .venv <zimmer-gripper-controller>   # path or git URL; only for --execute
export NOVA_API=http://<nova-host>
```

Then set the constants at the top of [`gr00t_ur10e.py`](gr00t_ur10e.py) for your cell:

| Constant | Set it to |
| --- | --- |
| `GROOT_HOST`, `GROOT_PORT` | the GR00T inference server |
| `PROMPT` | the instruction the policy was trained with |
| `IMAGE_SIZE` | the training cameras' resolution |
| `CAMERA_SERVER`, `CAMERAS` | the camera server's URL, and each policy camera key's device serial |
| `GRIPPER_HOST` | the IO-Link master of the Zimmer gripper |
| `N_ACTION_STEPS`, `TIMEOUT_S` | steps per chunk in sequential mode, and the run's time limit |
| `SAFE_Z_MM`, `HOME_SETTINGS` | the height to lift to before the move home, and the speed of that move |

The robot comes from `motion_group` in `episode_settings.json`; the example uses
`0@ur10e-calibration`.

## Run

```bash
uv run python novapolicy/examples/execute_gr00t_single_arm_pick_and_place/run_sequential.py [--execute]
uv run python novapolicy/examples/execute_gr00t_single_arm_pick_and_place/run_continuous.py [--execute]
```

| Flag | Default | |
| --- | --- | --- |
| `--execute` | off: dry run | move the robot and the gripper; asks for `yes` first |

Without `--execute` a script does a dry run: one real observation and one inference. It prints the
TCP targets and the gripper command and commands nothing. The gripper is not connected, since
connecting homes it and moves the jaws.

With `--execute` the script asks for `yes`, then homes and opens the gripper, lifts the tool if it
is low, moves the arm home and runs the policy. Keep a hand on the e-stop.

## episode_settings.json

The file says what one robot's columns of a training episode mean. `RobotSettings.action(mg)` in
[`episode_settings.py`](episode_settings.py) builds the matching `Action`.

| Field | Meaning |
| --- | --- |
| `rate_hz` | the policy's step rate |
| `motion_group`, `tcp` | the robot, and the point on the tool the training poses refer to |
| `joints_key`, `action_key` | the policy's state and action keys for the arm |
| `action_kind` | `joint_absolute`, `joint_delta`, `tcp_absolute` or `tcp_delta` |
| `scale`, `rotation_scale` | realized motion per action unit: mm (TCP position) or rad (joints, TCP rotation) |
| `use_tool_coordinate_system` | `tcp_delta` only: deltas in the tool frame, else in the world |
| `home_deg` | the start joints, in degrees |
| `gripper_state_key`, `gripper_action_key` | the policy's gripper keys |
| `gripper_close_when_negative`, `gripper_open`, `gripper_closed`, `gripper_holding` | the gripper command's polarity, and the state values for open, fully closed and holding |
| `fit_r2` | how well the chosen reading explains the recorded motion |

`tcp` and `gripper_closed` cannot be read from an episode. Set them by hand.

## Troubleshooting

| Symptom | What to do |
| --- | --- |
| `episode_settings.json has no gripper values for 'arm'` | Fill in `gripper_open`, `gripper_closed` and `gripper_holding`. |
| `Schema does not satisfy GR00T server requirements` | The server expects other keys. Compare with `get_server_info()`; see the [GR00T guide](../../docs/gr00t.md#inspecting-a-server). |
| The script cannot import `zimmer_gripper_controller` | Install the gripper package (the `uv pip install` line under Setup). It is only needed with `--execute`. |
| The dry run's frames have the wrong shape | Set `IMAGE_SIZE` to the training cameras' resolution. |
| The tool lands offset while it rotates | Check `tcp` in `episode_settings.json`. |
| The arm moves the wrong way or too far | Check `action_kind`, `scale` and `use_tool_coordinate_system` against a training episode. |
