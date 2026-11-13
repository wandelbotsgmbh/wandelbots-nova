# Setting up a GR00T policy on a NOVA robot

How to take a trained GR00T checkpoint behind a running inference server to a supervised run
on a real robot. [`examples/gr00t_single_arm_pick_and_place`](../examples/gr00t_single_arm_pick_and_place/)
is the worked example: an IsaacLab-trained cube pick-and-place on a UR10e with a Zimmer gripper.
The client API itself is in [docs/gr00t.md](../docs/gr00t.md).

## 1. Reach the server

The GR00T inference server speaks ZMQ (port 5555 in the container, often a NodePort outside
it). `Gr00tPolicyClient(host, port=...).get_server_info()` lists its state, action, video and
language keys and the action horizon. Install the client with
`uv sync --extra novapolicy-gr00t`.

## 2. Declare the policy in a `policy.toml`

Everything a deployment needs is written down in one file, `policy.toml`: the policy's facts
(keys, prompt, rate, image size, what each action means, gripper values), the cell (controller,
TCP, cameras, home) and the guards. Nothing is guessed or fitted at run time; the format is in
[`policy_config.py`](policy_config.py), and the example's
[`policy.toml`](../examples/gr00t_single_arm_pick_and_place/policy.toml) is a complete one.

Start a new one from a template:

```bash
uv run python -m novapolicy.gr00t.inspect_policy \
    --groot-host <host> --groot-port <port> --controller <controller> --tcp <tcp> \
    --data <local folder with the checkpoint and the training dataset> \
    --write-template policy.toml
```

The template has the facts the evidence states filled in: keys, horizon, language key, prompt,
rate, image size. Everything else is `"TODO"`, with what the evidence shows as a comment, and
must be declared from the training setup: the action representation, frame, unit and gain,
the gripper polarity and values, the TCP, the cameras, home and the guards. A config with a
`TODO` left does not load.

The user must declare the TCP. When a policy commands end-effector poses, its values refer
to one point on the tool; nothing in the data recovers that point unless the training tool
rotated a lot. Rotations pivot about it: a TCP d mm away from the training TCP shifts the
position by about d × angle while rotating.

## 3. Check the config

```bash
uv run python -m novapolicy.gr00t.inspect_policy --config policy.toml --data <folder>
```

Nothing moves. Every declared value is checked against the evidence and reported as PASS,
WARN, FAIL, or SKIP where no evidence covers it; any FAIL exits with code 1. The run scripts
also check the server against the config at every start.

| Evidence | Checks |
|---|---|
| server | state/action/video keys, language key, horizon; that it returns the end-effector action raw |
| checkpoint (`statistics.json`, ...) | joint unit against the robot's current joints, gripper values within the trained range, binary gripper action |
| dataset `meta/` (`info.json`, `tasks.jsonl`) | rate, image size, prompt |
| robot (`NOVA_API`) | the TCP exists; forward kinematics from NOVA matches its pose |
| training episodes (`episode_*.parquet`) | the declared frame and gain against the joints' forward kinematics, delta vs. absolute, gripper polarity and values, home vs. the episode start; guards vs. the training envelope (warning only) |

`--data` searches a folder for the checkpoint (the folder holding `statistics.json`; the
newest `checkpoint-N` if there are several), the dataset's `meta/` and its episodes. Point
at them with `--checkpoint`, `--meta` and `--episode` instead, or read the checkpoint from
the server's pod with `--kubeconfig <file>` (`--kube-target`, default `gr00t/deploy/gr00t`).

Fits against episodes only check declared values, never supply them, and need episodes that
exercise what they check. Keeping the dataset's `meta/` and a few episodes next to each
checkpoint lets `--data` find everything in one folder.

## 4. Check the cell against the training scene

If the policy was trained in simulation, replay one training episode's recorded joint path on
the real robot before running the policy:
[`examples/replay/replay_episode.py`](../examples/replay/replay_episode.py) replays any
episode (`--controller`, `--joints-from state|action`, `--gripper-column`, `--speed`), and
the pick-and-place example wraps it with its gripper. A replay that picks and places proves
the robot frame, table and target positions match; whatever goes wrong afterwards is the
policy. It does not check the TCP: joints replay the same path whatever the TCP is.

## 5. Build the schema and the client

- **State:** `Observation.joint_positions(key, source=mg, action=False)` when the policy's
  actions are not joint targets; `Observation.computed` for state NOVA does not have (a
  gripper on its own bus); `Observation.tcp(..., tcp=...)` to pin the TCP even if the model
  does not take it.
- **Cameras:** one `Observation.image` per video key. Frame the real images like the training
  cameras: crop to the training aspect, then resize to the training resolution.
- **Actions:** `Action.tcp(key, mode="absolute")` for end-effector targets,
  `Action.joint_positions` for joint targets, `Action.computed` for actuators outside NOVA.
- **Client:** subclass `Gr00tPolicyClient` where the model needs it. The example's
  `SingleArmGr00tClient` reads everything from `policy.toml`: it sends the prompt under the
  declared language key, turns the raw end-effector deltas into absolute TCP targets as
  declared (frame, unit, gain, rotations about the TCP), and forces the step spacing to the
  declared rate.

## 6. Choose the execution mode

- **Sequential** (start here): run `n_action_steps` of each chunk to standstill, observe,
  infer again. Every chunk is checked on its own and the arm stops between chunks.
- **Continuous:** re-infer as fast as the server answers and replace the buffered tail. Chunks
  must meet without jumps: keep the steps that executed during inference frozen, blend the
  overlap and smooth it on the executor's timestep timeline (the example's `TcpPlanQueue`,
  modelled on the LeRobot async queue). A plain hard replace shakes.

Play steps at the training rate times a speed factor below 1 for the first runs.

## 7. Guard and run

- The guards in `policy.toml` are the deployer's safety choice: TCP speed, chunk travel,
  rotation step, tilt, yaw, workspace and table height. `--config` warns where they are
  tighter than the training episodes' envelope, since they would stop motion the policy
  learned.
- Check every chunk before it is sent and stop the run on any violation.
- Dry run first (one observation, one inference, print the targets), then a supervised run
  with a chunk limit and a hand on the e-stop.

## Things to keep in mind

- **Rotation vectors:** near a half-turn the same orientation reads as `+v` or `-v`. Unwrap
  rotation vectors (`novapolicy.chunking.unwrap_tcp_rotations`) before interpolating or
  blending them component-wise.
- **Commanded vs. realized actions:** an action that is a command to a controller (for example
  an IK target relative to the measured pose) can move the robot less than it says. How much
  depends on the training controller, so it can only be approximated on another robot:
  declare it and check it with `--config`. Policies trained on the motion that actually
  happened (next joint positions or poses) avoid this.
- **Sim-to-real:** a policy trained on rendered images reads real camera images less
  precisely. Once a replay confirms the cell, remaining placement errors come from the
  policy, not from the deployment settings.
