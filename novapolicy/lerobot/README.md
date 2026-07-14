# Run a LeRobot policy with NOVA PolicyExecutor

This guide is for users who want to execute a trained LeRobot policy through NOVA.

The setup has two parts:

1. Start LeRobot's async inference server where the policy checkpoint is available.
2. Configure `LeRobotPolicyClient` on the NOVA/robot side and run it with `PolicyExecutor`.

The NOVA side sends observations to the LeRobot server and receives action chunks. Model weights are
not uploaded by the client; `pretrained_name_or_path` is interpreted by the LeRobot server.

## Quickstart

### 1. Install the LeRobot policy extra

Install this in the environment that runs NOVA policy execution:

```bash
uv add wandelbots-nova --extra novapolicy-lerobot
```

If you run the LeRobot server in a separate Python environment, install LeRobot there too. LeRobot
currently requires Python 3.12:

```bash
mamba create -y -n lerobot-server python=3.12 pip
conda activate lerobot-server
python -m pip install --upgrade pip
python -m pip install 'wandelbots-nova[novapolicy-lerobot]'
```

### 2. Put the checkpoint where the LeRobot server can read it

Use either:

- a server-local checkpoint path, for example `/models/pick_place_act/pretrained_model`, or
- a Hugging Face model id supported by `policy_class.from_pretrained(...)`.

For a server-local checkpoint, copy the whole pretrained model directory to the server host:

```bash
scp -r ./pretrained_model user@gpu-server:/models/my_lerobot_policy
```

The NOVA client will later pass that same server-side path:

```python
pretrained_name_or_path="/models/my_lerobot_policy"
```

### 3. Start the LeRobot async server

Run this on the machine that should execute inference:

```bash
conda activate lerobot-server

python -m lerobot.async_inference.policy_server \
  --host=0.0.0.0 \
  --port=8080
```

The server listens first. It loads the policy only after `LeRobotPolicyClient` connects and sends
`SendPolicyInstructions`. `PolicyExecutor` does not know about the LeRobot protocol directly; it
only calls the generic policy-client API. For LeRobot, `SendPolicyInstructions` is sent while the
executor is still in its `CONNECTING` phase, so model-loading/setup latency does not consume the
execution timeout.

Optional LeRobot server timing flags:

- `--fps`: controls LeRobot's server-side `TimedAction` timestamps. `LeRobotPolicyClient`
  ignores those timestamps and sets `ActionChunk.dt_ms = 1000 / fps`; `PolicyExecutor` then uses
  that `dt_ms` when scheduling waypoints. Keep the server flag aligned with the client `fps` if you
  set both.
- `--inference_latency`: adds a target response delay in `GetActions`. Leave it at the LeRobot
  default unless you intentionally want the server to pace responses.
- `--obs_queue_timeout`: how long `GetActions` waits for an observation before returning empty.

### 4. Configure the NOVA policy client

```python
from nova import Nova
from nova.config import NovaConfig
from novapolicy import (
    BoolMapping,
    LeRobotPolicyClient,
    Observation,
    PolicyExecutor,
    PolicySchema,
    WebRTCCameras,
)

cameras = WebRTCCameras(
    api_url="http://<nova-or-camera-host>:8011/webrtc-streamer",
    resize=(320, 240),  # width, height expected by your policy/cameras
)

async with Nova(config=NovaConfig(host="http://<nova-host>")) as nova:
    cell = nova.cell("cell")
    arm = (await cell.controller("cobot"))[0]

    schema = PolicySchema(observations=[
        Observation.joint_positions("arm", source=arm),
        Observation.io("gripper", source=arm, io="digital_out[0]", mapping=BoolMapping()),
        Observation.image(
            "cam_scene_1",
            source=cameras.device("<scene-camera-device-id>"),
        ),
    ])

    from novapolicy.lerobot import load_execution_settings

    # This config must be visible to the NOVA client. The model weights may
    # remain at a server-only path.
    execution = load_execution_settings("./my_lerobot_policy/config.json")

    policy = LeRobotPolicyClient(
        server_address="<lerobot-server-host>:8080",
        pretrained_name_or_path="/models/my_lerobot_policy",  # path on the LeRobot server
        policy_type="act",
        fps=15,
        playback_speed=1.0,
        actions_per_chunk=execution.chunk_size,
        device="cuda",
    )

    executor = PolicyExecutor(
        schema,
        policy,
        policy_rate_hz=-1,
        n_action_steps=execution.n_action_steps,
        interpolate_chunk_ramps=True,
        timeout_s=80,
    )
    result = await executor.run()
```

## Configuration reference

### `server_address`

Address of the LeRobot async inference server in `"host:port"` form.

Examples:

```python
server_address="127.0.0.1:8080"
server_address="gpu-server.internal:8080"
```

### `pretrained_name_or_path`

Checkpoint path or Hugging Face model id passed to the LeRobot server.

Important: for a remote server, this is resolved on the server machine, not on the NOVA client.
The client does not upload model weights.

```python
pretrained_name_or_path="/models/my_lerobot_policy"
pretrained_name_or_path="org/my-policy"
```

### `policy_type`

LeRobot policy type sent to the server, for example:

```python
policy_type="act"
```

The NOVA client-side decoder is policy-architecture agnostic as long as the server returns a flat
joint action vector matching the schema action DOF.

### `fps`

Control/dataset frequency used by the NOVA client to time returned actions:

```python
fps=15  # ActionChunk.dt_ms = 1000 / 15
```

This should match the policy's intended control rate. This is separate from the LeRobot server's
optional `--fps` CLI flag. The server flag is used by LeRobot when it creates `TimedAction`
timestamps; `LeRobotPolicyClient` does not use those timestamps. It decodes the returned action
tensors into NOVA `ActionChunk`s and sets the chunk `dt_ms` from this client-side `fps`.
`PolicyExecutor` then uses that `dt_ms` when scheduling the returned chunk.

### `playback_speed`

Explicit physical playback speed relative to the dataset rate:

```python
playback_speed=0.75  # execute 25% slower: 15 Hz dataset actions use 88.89 ms intervals
```

The dataset frequency remains `fps=15`; only the physical `ActionChunk.dt_ms` is scaled:

```text
dt_ms = 1000 / (fps * playback_speed)
```

Keep this at `1.0` for nominal dataset timing. Values below `1.0` are useful when the NOVA
best-effort waypoint tracker follows the learned actions more aggressively than the original data
collection controller.

### Checkpoint execution settings

LeRobot checkpoints define both values needed for ACT chunk execution:

- `chunk_size`: number of actions predicted by the model
- `n_action_steps`: number of predicted actions intended for execution before replanning

Load them rather than duplicating magic numbers:

```python
from novapolicy.lerobot import load_execution_settings

settings = load_execution_settings("./pretrained_model")
# settings.chunk_size == 11
# settings.n_action_steps == 8
```

The source can be a local checkpoint directory, a direct `config.json` path, or a Hugging Face
model id. If `pretrained_name_or_path` names a path that exists only on a remote inference server,
the NOVA client cannot inspect it: LeRobot's current async RPC has no checkpoint-metadata method.
Provide a client-local copy of `config.json` or set both values explicitly.

The UR3 example performs this discovery automatically. Explicit CLI values take precedence:

```text
--actions-per-chunk 11 --n-action-steps 8
```

### `actions_per_chunk`

Number of action steps requested from the server. For ACT, use the checkpoint's `chunk_size` so the
full prediction remains available for logging and visualization. `PolicyExecutor.n_action_steps`
then limits execution to the checkpoint's intended execution horizon.

The LeRobot async server does not infer this. It is part of LeRobot's `RemotePolicyConfig`, and the
server slices `policy.predict_action_chunk(...)` to the requested length.

### Settled ACT chunks

For NOVA waypoint jogging, execute the checkpoint-defined ACT execution horizon and wait for NOVA
to report that its waypoint buffer reached standstill before the next inference:

```python
PolicyExecutor(
    schema,
    policy,
    policy_rate_hz=-1,
    n_action_steps=settings.n_action_steps,
    interpolate_chunk_ramps=True,
)
```

This prevents the policy observation from being captured while the robot is still moving and avoids
executing the lower-confidence tail beyond the checkpoint's configured execution horizon. The
default sequential mode (`policy_rate_hz=-1`) automatically waits for exact NOVA standstill and, if
ACT's first target is farther away than the spacing inside its executed horizon, prepends a
same-`dt_ms` interpolated bridge to one continuous motion request. IO and computed actions fire when
NOVA's server clock reaches policy waypoint zero; the
robot does not stop at that boundary. Endpoint interpolation allocates additional same-`dt_ms`
intervals for acceleration and braking of each settled request. See
[`docs/executor.md`](../docs/executor.md#bridging-a-distant-first-waypoint).

### `use_async_queue`

Enable this only when the robot transport can track LeRobot's fixed-rate client queue:

```python
from novapolicy.lerobot import AsyncQueueAggregation, LeRobotPolicyClient

policy = LeRobotPolicyClient(
    ...,
    use_async_queue=True,
    async_queue_aggregation=AsyncQueueAggregation.WEIGHTED_AVERAGE,
    async_queue_refill_threshold=0.75,
)
```

Aggregation is applied only when an old and a new action target the same future timestep:

| Mode | Merge |
|---|---|
| `WEIGHTED_AVERAGE` | `0.3 * old + 0.7 * new` (LeRobot default) |
| `LATEST_ONLY` | `new` |
| `AVERAGE` | arithmetic mean of every prediction received for the timestep |
| `CONSERVATIVE` | `0.7 * old + 0.3 * new` |

The generic client retains LeRobot's weighted-average default. The physical UR3 example defaults to
`AVERAGE`; repeated plug-task runs showed lower peak path curvature without the delayed transition
caused by conservative aggregation.

The client normally consumes one action each policy control tick, requests a refill when 75% of the
previous chunk remains by default, and merges overlapping actions using the selected enum mode.
Refills use LeRobot's `must_go` flag because the server's default one-radian observation similarity
tolerance would otherwise defer most ACT inference until queue depletion. Configure
`PolicyExecutor` with `policy_rate_hz=fps` and `n_action_steps=0`.

NOVA's jogger clock advances independently of the Python control loop. Before consuming an action,
the executor maps the latest acknowledged raw NOVA controller timestamp to the absolute LeRobot
timestep; if local work delayed a tick, the client drops every action whose execution time elapsed.
Threshold-triggered inference remains asynchronous while NOVA executes its published lookahead. It
is merged on a later controller-synchronized tick instead of blocking after a timestep has already
been selected. The client then prepends the predecessor from NOVA's published trajectory and retains
the selected action plus its immutable successor. The replacement therefore contains an exact
past/current/future seam before aggregation begins. IO remains sourced from the selected current
action, not the prepended predecessor.

Between inference updates the client consumes actions internally without resending a shrinking
tail, so existing NOVA waypoints keep their original timeline. The initial queue prediction receives
a measured-state bridge and execution waits for its exact policy-zero boundary. The timestamp
already assigned to policy waypoint zero becomes the immutable action-timestep origin. Later merged
lookaheads use ``origin + action_timestep * policy_dt`` directly in the raw controller-timer domain.
Client wall time, server/client speed-ratio estimation, and post-boundary re-origining are not part of
queue timestamp calculation. Only each integer timestamp sent to NOVA is quantized. The overlapping
prefix is retained instead of being restarted from a measured-state hold at ``now``. This prevents
both catch-up motion and repeated zero-velocity braking/acceleration. In Rerun, the three-point
retained replacement seam is shown in Nova Violet while fresh policy output remains orange. This is
still LeRobot async ACT queue execution, not model-side RTC.

### `state_overrides`

Optional raw observation values to replace before sending to LeRobot.

Use this only for checkpoints whose training data expects constants or special values in
`observation.state`. For example, if a legacy dataset stored zero instead of measured arm joints:

```python
state_overrides={f"arm_{idx}": 0.0 for idx in range(1, 7)}
```

Do not use overrides for normal datasets trained with real joint observations.

## Observation and action mapping

`LeRobotPolicyClient` maps the NOVA `PolicySchema` into LeRobot feature metadata.

For example:

```python
Observation.joint_positions("arm", source=arm)
Observation.io("gripper", source=arm, io="digital_out[0]", mapping=BoolMapping())
Observation.image("cam_scene_1", source=camera)
```

becomes:

```python
{
    "observation.state": {
        "dtype": "float32",
        "shape": (7,),
        "names": ["arm_1", "arm_2", "arm_3", "arm_4", "arm_5", "arm_6", "gripper"],
    },
    "observation.images.cam_scene_1": {
        "dtype": "image",
        "shape": (240, 320, 3),
        "names": ["height", "width", "channels"],
    },
}
```

Image shape comes from the first camera frame. Configure image dimensions in the camera source, for
example:

```python
WebRTCCameras(..., resize=(320, 240))
```

Returned LeRobot actions are decoded as flat joint targets followed by IO actions. Joint targets are
split according to the schema's joint action motion groups. IO actions are written once per returned
chunk using the first action step. TCP actions from a flat LeRobot action vector are not decoded yet.

## Protocol notes

The LeRobot async server flow, implemented inside `LeRobotPolicyClient`, is:

1. `Ready` when the policy client connects
2. `SendPolicyInstructions` during the optional policy `prepare(...)` step, while the executor is
   still in `CONNECTING`
3. `SendObservations` with the latest consumed action timestep
4. `GetActions` in a background refill task
5. Consume one queued action per control tick and blend overlapping future timesteps

`PolicyExecutor` triggers this indirectly by calling the `PolicyClient` methods; it does not send
LeRobot RPCs itself. The executor switches to `EXECUTING` after preparation, so
readiness/model-loading time is excluded from `timeout_s`.

`SendPolicyInstructions` contains a pickled LeRobot `RemotePolicyConfig` with:

- `policy_type`
- `pretrained_name_or_path`
- `lerobot_features`
- `actions_per_chunk`
- `device`
- `rename_map` (left empty by this client)

The server does not expose model metadata before setup. Keep `policy_type`, `fps`, and
`actions_per_chunk` in your deployment configuration next to the checkpoint path.

## Troubleshooting

### The server cannot load the checkpoint

Check that `pretrained_name_or_path` exists on the LeRobot server machine, not just on the NOVA
client machine.

### Image shape or missing image errors

`LeRobotPolicyClient` needs the first image frame to declare LeRobot feature metadata. Make sure the
camera is connected and that the image key in `Observation.image(...)` matches the key expected by
the checkpoint.

### Actions are the wrong dimension

The returned flat action vector must match the total DOF of the schema's joint action motion groups
plus any schema IO actions. For a single 6-axis arm with one gripper IO action, the policy should
return seven action values per step.
