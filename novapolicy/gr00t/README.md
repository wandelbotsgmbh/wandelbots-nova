# Setting up a GR00T policy on a NOVA robot

How to take a trained GR00T checkpoint behind a running inference server to a supervised run
on a real robot. Nothing here is GR00T-specific beyond the client: a deployment is a plain
`PolicySchema`, a `Gr00tPolicyClient` and a `PolicyExecutor`.
[`examples/gr00t_single_arm_pick_and_place`](../examples/gr00t_single_arm_pick_and_place/) is
the worked example: an IsaacLab-trained cube pick-and-place on a UR10e with a Zimmer gripper.
The client API is in [docs/gr00t.md](../docs/gr00t.md).

## 1. Reach the server

The GR00T inference server speaks ZMQ (port 5555 in the container, often a NodePort outside it).
Install the client with `uv sync --extra novapolicy-gr00t`.
`Gr00tPolicyClient(host, port=...).get_server_info()` lists the state, action, video and language
keys and the action horizon. The client takes the language key and the state keys from the
server itself.

## 2. Know what the policy's actions mean

The server tells the keys, not their meaning. From the training setup, find out:

| Question | Where it goes |
|---|---|
| Joint targets, absolute TCP targets, or TCP deltas? | `Action.joint_positions`, `Action.tcp(mode="absolute")`, or `Action.tcp(mode="relative")` |
| Deltas per step, applied to the pose the previous step reached? | `accumulate=True` |
| Deltas in the world (robot base) or in the tool coordinate system? | `use_tool_coordinate_system=False` / `True`, as in NOVA's TCP jogging |
| Units (metres, degrees)? | `position_ops=[Scale(0.001)]`, `ops=[Rad2Deg()]` |
| Which point on the tool (TCP) do the poses refer to? | `Observation.tcp(..., tcp=...)` |
| Rate, camera resolution, prompt | `dt_ms`, `WebRTCCameras(resize=..., resize_mode="crop")`, `Observation.constant("language", ...)` |
| How is the gripper driven, and what is its state? | `Action.io`, or `Action.computed` / `Observation.computed` for one on its own bus |

The training config usually states these. A policy trained with a differential-IK controller
(IsaacLab's relative pose command) outputs commands, and the simulated robot realized only part of
each one; an extra `Scale` converts commanded to realized motion. The example's
[`settings_from_episode.py`](../examples/gr00t_single_arm_pick_and_place/settings_from_episode.py) measures frame, gain
and gripper polarity from one training episode when the config does not say.

The TCP must be known: nothing in the data recovers it unless the training tool rotated a lot.
Rotations pivot about it, so a TCP d mm off shifts the position by about d × angle while rotating.

## 3. Check the cell against the training scene

If the policy was trained in simulation, replay one training episode's recorded joint path on the
real robot before running the policy:
[`examples/replay/replay_episode.py`](../examples/replay/replay_episode.py). A replay that picks
and places proves the robot frame, table and target positions match; whatever goes wrong
afterwards is the policy. It does not check the TCP: joints replay the same path whatever the TCP
is.

## 4. Write the run script

```python
from novapolicy import (
    Action,
    Gr00tPolicyClient,
    Observation,
    PolicyExecutor,
    PolicySchema,
    Scale,
    SequentialExecution,
    TcpLimits,
    WebRTCCameras,
)

client = Gr00tPolicyClient(host, port=port, dt_ms=1000 / 30)  # the training rate
webrtc = WebRTCCameras(api_url=camera_server, resize=(320, 240), resize_mode="crop")
schema = PolicySchema(
    observations=[
        Observation.joint_positions("arm_joints", source=mg, action=False),
        Observation.tcp("tcp_pose", source=mg, tcp="umi_corrected"),  # the TCP deltas use
        Observation.computed(gripper.observe, keys=["gripper"]),
        *(Observation.image(key, source=webrtc.device(serial)) for key, serial in cameras.items()),
        Observation.constant(
            "language", value="pick up the cube and place it on the yellow square"
        ),
    ],
    actions=[
        Action.tcp(
            "eef_delta",
            target=mg,
            mode="relative",
            accumulate=True,
            use_tool_coordinate_system=False,
            position_ops=[Scale(0.001), Scale(1 / 0.24)],  # metres; 24 % realized in training
            orientation_ops=[Scale(1 / 0.30)],
        ),
        Action.computed(gripper.act),  # reads client.last_raw_action["gripper"]
    ],
)
executor = PolicyExecutor(
    schema,
    client,
    execution=SequentialExecution(),
    n_action_steps=8,
    timeout_s=300,
    tcp_limits=TcpLimits(max_speed_mm_s=650, max_chunk_travel_mm=250, z_mm=(25, 1000)),
)
result = await executor.run()
```

A gripper on its own bus enters through `Observation.computed` (its state, with `keys=` so the
schema check counts it) and `Action.computed` (its command, read from
`client.last_raw_action`). The example's `ZimmerGripper` is a complete one: it maps the jaw gap
onto the training state values, switches at the step of the chunk that asks for it, and makes
`observe` wait while the jaws move, so the policy is not asked again until they settle.

## 5. Choose the execution mode

- **Sequential** (start here): run `n_action_steps` of each chunk to standstill, observe, infer
  again. Every chunk is checked on its own and the arm stops between chunks.
- **Continuous:** re-infer as fast as the server answers. Wrap the client in
  `QueuedPolicyClient(client)` and use `ContinuousExecution()` with `n_action_steps=0`: it keeps
  the steps that executed during inference frozen, blends the overlap and smooths it on the
  executor's timeline. Without it, every replacement makes the arm jump.

A `dt_ms` above the training step plays the policy slower; use it for first runs.

## 6. Limit and run

- `TcpLimits` (speed, rotation speed, chunk travel, x/y/z bounds) are checked on every chunk
  after units and relative targets are resolved; a breaking chunk ends the run with
  `TcpLimitError`. They are the deployer's choice, not facts of the policy: keep them wider than
  the motion the policy learned (`settings_from_episode.py` prints the training envelope).
- Dry run first: one real observation and one inference, the targets printed and checked, nothing
  commanded. `resolve_actions(schema, chunk, states)` gives the targets the executor would send.
- Then a supervised run with a hand on the e-stop. Before moving home, move straight up when the
  tool is low, so the joint move cannot sweep it over the table.

## Things to keep in mind

- **Rotation vectors:** near a half-turn the same orientation reads as `+v` or `-v`. Relative TCP
  actions and `QueuedPolicyClient` unwrap them; do the same
  (`novapolicy.chunking.unwrap_tcp_rotations`) before interpolating rotation vectors yourself.
- **Commanded vs. realized actions:** how much of a command the training controller realized can
  only be approximated on another robot. Policies trained on the motion that actually happened
  (next joint positions or poses) avoid this.
- **Sim-to-real:** a policy trained on rendered images reads real camera images less precisely.
  Once a replay confirms the cell, remaining placement errors come from the policy, not from the
  deployment settings.
