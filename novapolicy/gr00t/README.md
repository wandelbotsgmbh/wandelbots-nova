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
episode (`--controller`, `--joints-from state|action`, `--gripper-column`,
`--gripper io:<name>|zimmer:<host>`, `--stop-at-grip`, `--speed`); the pick-and-place
example's README has the command for its cell. A replay that picks and places proves
the robot frame, table and target positions match; whatever goes wrong afterwards is the
policy. It does not check the TCP: joints replay the same path whatever the TCP is.

## 5. Build the schema and the client

[`declared.py`](declared.py) runs the policy as `policy.toml` declares it.
`DeclaredGr00tClient(cfg)` sends the prompt under the declared language key, keeps the
declared state keys, frames camera images like the training cameras (crop to the training
aspect, resize to the training resolution), forces the declared step spacing, turns the raw
end-effector action into absolute TCP targets (frame, unit, gain, rotations about the TCP),
checks every chunk against the guards and checks the server against the config. A script
only builds the schema:

```python
from novapolicy import Action, Observation, PolicyExecutor, PolicySchema, SequentialExecution
from novapolicy.gr00t import policy_config
from novapolicy.gr00t.declared import DeclaredGr00tClient, move_home

cfg = policy_config.load("policy.toml")
mg = (await nova.cell().controller(cfg.cell.controller))[0]
client = DeclaredGr00tClient(cfg)
schema = PolicySchema(
    observations=[
        Observation.joint_positions(cfg.policy.joints.key, source=mg, action=False),
        Observation.tcp("tcp_pose", source=mg, tcp=cfg.cell.tcp),  # pins the declared TCP
        Observation.computed(gripper.observe),  # state NOVA does not have
        *(Observation.image(key, source=cameras[key]) for key in cfg.policy.video_keys),
    ],
    actions=[
        Action.tcp(cfg.policy.tcp_action.key, target=mg, mode="absolute"),
        Action.computed(gripper.act),  # an actuator outside NOVA
    ],
)
await move_home(mg, cfg)  # lifts straight up first when low
await PolicyExecutor(schema, client, execution=SequentialExecution()).run()
```

A gripper on its own bus enters through `Observation.computed` / `Action.computed`:
`gripper_state(cfg, jaw_gap_mm, holding=...)` maps the real jaw gap onto the declared state
values, and `next_gripper_switch(cfg, client.gripper_actions, client.executed_steps, ...)`
finds the step of the chunk that switches it. Pass `hold_while=lambda: gripper.busy` so the
arm holds still while the jaws move. The example's `ZimmerGripper` does all of this.

## 6. Choose the execution mode

- **Sequential** (start here, the default): run `n_action_steps` of each chunk to
  standstill, observe, infer again. Every chunk is checked on its own and the arm stops
  between chunks. `near_z_mm` / `near_steps` run shorter chunks close to the table, so the
  policy re-observes and corrects its placement more often; `max_chunks` caps a run.
- **Continuous** (`continuous=True`, with `ContinuousExecution()` and `n_action_steps=0`):
  re-infer as fast as the server answers. The client's `PlanQueue` keeps the steps that
  executed during inference frozen, blends the overlap and smooths it on the executor's
  timestep timeline, modelled on the LeRobot async queue. A plain hard replace shakes.

`speed_factor` below 1 plays the steps slower than the training rate for first runs.

## 7. Guard and run

- The guards in `policy.toml` are the deployer's safety choice: TCP speed, chunk travel,
  rotation step, workspace and table height. `--config` warns where they are
  tighter than the training episodes' envelope, since they would stop motion the policy
  learned. The client raises `GuardError` on the first chunk that breaks one, which stops
  the run; `check_guards` / `guard_violations` check targets on their own.
- Dry run first: one observation, one inference, the targets and guard result printed, and
  nothing commanded. Then a supervised run with a chunk limit and a hand on the e-stop.
- Before moving home, `move_home` (or `lift_if_low`) moves straight up when the tool is
  low, so the joint move cannot sweep it over the table or the object.

## Things to keep in mind

- **Rotation vectors:** near a half-turn the same orientation reads as `+v` or `-v`. Unwrap
  rotation vectors (`novapolicy.chunking.unwrap_tcp_rotations`) before interpolating or
  blending them component-wise; `tcp_targets` and `PlanQueue` already do.
- **Commanded vs. realized actions:** an action that is a command to a controller (for example
  an IK target relative to the measured pose) can move the robot less than it says. How much
  depends on the training controller, so it can only be approximated on another robot:
  declare it and check it with `--config`. Policies trained on the motion that actually
  happened (next joint positions or poses) avoid this.
- **Sim-to-real:** a policy trained on rendered images reads real camera images less
  precisely. Once a replay confirms the cell, remaining placement errors come from the
  policy, not from the deployment settings.
