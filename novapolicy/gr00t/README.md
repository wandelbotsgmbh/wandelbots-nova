# Setting up a GR00T policy on a NOVA robot

How to take a trained GR00T checkpoint behind a running inference server to a supervised run
on a real robot. [`examples/gr00t_single_arm_pick_and_place`](../examples/gr00t_single_arm_pick_and_place/)
is the worked example: an IsaacLab-trained cube pick-and-place on a UR10e with a Zimmer gripper.
The stock `Gr00tPolicyClient` is documented in [docs/gr00t.md](../docs/gr00t.md); this guide
builds on [`declared.py`](declared.py), which runs it as a `policy.toml` declares.

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

## 5. Write the run script

[`declared.py`](declared.py) runs the policy as `policy.toml` declares it.
`DeclaredGr00tClient(cfg)` sends the prompt under the declared language key, keeps the
declared state keys, frames camera images like the training cameras (crop to the training
aspect, resize to the training resolution), forces the declared step spacing, turns the raw
end-effector action into absolute TCP targets (frame, unit, gain, rotations about the TCP),
checks every chunk against the guards and checks the server against the config. A script
builds the cameras, the schema and the gripper, and runs:

```python
import asyncio

import nova
from novapolicy import (
    Action,
    Observation,
    PolicyExecutor,
    PolicySchema,
    SequentialExecution,
    WebRTCCameras,
)
from novapolicy.gr00t import policy_config
from novapolicy.gr00t.declared import ChunkLimitError, DeclaredGr00tClient, move_home


async def main(execute: bool) -> None:
    cfg = policy_config.load("policy.toml")
    async with nova.Nova() as n:  # NOVA_API from the environment
        mg = (await n.cell().controller(cfg.cell.controller))[0]
        webrtc = WebRTCCameras(api_url=cfg.cell.camera_server, frame_history=1)
        cameras = {key: webrtc.device(serial) for key, serial in cfg.cell.cameras.items()}
        client = DeclaredGr00tClient(cfg, max_chunks=120)
        gripper = MyGripper(cfg, client)  # see below
        client.hold_while = lambda: gripper.busy  # the arm holds still while the jaws move
        schema = PolicySchema(
            observations=[
                Observation.joint_positions(cfg.policy.joints.key, source=mg, action=False),
                Observation.tcp("tcp_pose", source=mg, tcp=cfg.cell.tcp),  # pins the TCP
                Observation.computed(gripper.observe),  # state NOVA does not have
                *(Observation.image(key, source=cameras[key]) for key in cfg.policy.video_keys),
            ],
            actions=[
                Action.tcp(cfg.policy.tcp_action.key, target=mg, mode="absolute"),
                Action.computed(gripper.act),  # an actuator outside NOVA
            ],
        )
        await client.connect([mg.id])
        try:
            await client.validate_schema(schema)  # the server serves the declared policy
            if not execute:
                await dry_run(mg, client, schema, cameras)  # step 7
                return
            await gripper.connect()  # homing moves the jaws
            await move_home(mg, cfg)
            executor = PolicyExecutor(
                schema, client, timeout_s=300, execution=SequentialExecution()
            )
            try:
                result = await executor.run()
                print(result.reason, result.steps)
            except ChunkLimitError as e:  # max_chunks: stops at standstill
                print(e)
        finally:
            await client.close()
```

**Gripper on its own bus.** It enters through `Observation.computed` / `Action.computed` and
needs `observe`, `act` and `busy`:

- `observe` returns `{cfg.policy.gripper.state_key: gripper_state(cfg, jaw_gap_mm, holding=...)}`,
  the real jaw gap mapped onto the declared state values.
- `act` asks `next_gripper_switch(cfg, client.gripper_actions, client.executed_steps,
  closed=...)` for the step of the chunk that switches the gripper and commands it after
  `step * client.dt_ms`, in a task, so the executor loop never blocks.
- `busy` is True while a command runs; the client holds the arm meanwhile (`hold_while`,
  set after the gripper exists because the gripper reads the client).

The example's `ZimmerGripper` in
[`gr00t_ur10e.py`](../examples/gr00t_single_arm_pick_and_place/gr00t_ur10e.py) is a complete
one for a Zimmer gripper on an IO-Link master (`zimmer-gripper-controller`; connecting homes
the jaws). The example folder is not a package: copy the class into your script.

## 6. Choose the execution mode

`DeclaredGr00tClient` arguments:

| Argument | Default | Effect |
|---|---|---|
| `continuous` | `False` | Sequential: run `n_action_steps` of each chunk to standstill, observe, infer again; every chunk is checked on its own. Continuous: re-infer as fast as the server answers; use with `ContinuousExecution()` and `PolicyExecutor(..., n_action_steps=0)` |
| `n_action_steps` | 8 | Sequential: steps of each chunk that run |
| `near_z_mm`, `near_steps` | off | Sequential: below this TCP height run only `near_steps` per chunk, so the policy re-observes and corrects more often close to the table |
| `speed_factor` | 1.0 | Below 1 plays the steps slower than the training rate; use it for first runs |
| `max_chunks` | none | Sequential: after this many chunks the client raises `ChunkLimitError` at standstill |
| `hold_while` | none | While it returns True, the arm holds still and the policy is not asked |

Continuous mode merges chunks on the executor's timestep timeline with `PlanQueue`: the
steps that executed during inference stay frozen, the overlap is blended and smoothed, as in
the LeRobot async queue. A plain hard replace shakes. `PolicyExecutor(timeout_s=...)` ends a
run in either mode.

## 7. Guard and run

- The guards in `policy.toml` are the deployer's safety choice: TCP speed, chunk travel,
  rotation step, workspace and table height. `--config` warns where they are tighter than
  the training episodes' envelope, since they would stop motion the policy learned. The
  client raises `GuardError` on the first chunk that breaks one, which stops the run.
- Dry run first. `client.preview(states, schema, frames)` does one inference and returns per
  motion group the raw action, the targets, the steps that would run, the guards they break
  and the gripper action, without executing or raising on a guard:

  ```python
  async def dry_run(mg, client, schema, cameras):
      for cam in cameras.values():
          await cam.connect()
      try:
          state = await mg.get_state(client.cfg.cell.tcp)
          frames = {key: cam.read() for key, cam in cameras.items()}
          preview = (await client.preview({mg.id: state}, schema, frames))[mg.id]
          for target in preview.targets[: preview.steps]:
              print([round(v, 1) for v in target[:3]])
          print("guards:", preview.violations or "OK")
      finally:
          for cam in cameras.values():
              await cam.disconnect()
  ```

  The gripper is not connected in a dry run, so `observe` must answer without it (the jaws
  assumed open).
- Then a supervised run with a chunk limit and a hand on the e-stop.
- `move_home(mg, cfg)` first moves straight up to `safe_z_mm` (default 200 mm) when the TCP
  is lower, so the joint move cannot sweep the tool over the table or the object, then moves
  to the declared home; both at `speed_mm_s` (default 100 mm/s). `lift_if_low` is the first
  half on its own.

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
