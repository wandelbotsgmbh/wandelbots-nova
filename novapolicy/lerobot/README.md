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
pretrained_name_or_path = "/models/my_lerobot_policy"
```

### 3. Start the LeRobot async server

Run the server on the machine that has the checkpoint and should execute model inference. The server
starts without a checkpoint argument; it loads the checkpoint path sent by the NOVA client when that
client connects.

```bash
# Terminal 1 — inference host
conda activate lerobot-server

python -m lerobot.async_inference.policy_server \
  --host=0.0.0.0 \
  --port=8080
```

Keep this process running. A successful startup logs that the gRPC server is listening on port 8080.
The first client connection sends `policy_type`, checkpoint path, action-chunk size, and inference
device. Loading happens while `PolicyExecutor` is in `CONNECTING`, before its execution timeout
starts.

The server also accepts `--fps`, `--inference_latency`, and `--obs_queue_timeout`. They are not
required for the basic setup. If you set server `--fps`, keep it aligned with the client's `fps`.

### 4. Execute the policy through NOVA

Save the following as `run_lerobot_policy.py` on the NOVA client machine. Replace the host names,
controller, camera device, checkpoint paths, and IO key with values for your cell and checkpoint.
The checkpoint's `config.json` must be readable by the client; model weights only need to exist on
the inference server.

```python
import asyncio

from nova import Nova
from nova.config import NovaConfig
from novapolicy import (
    BoolMapping,
    LeRobotPolicyClient,
    Observation,
    PolicyExecutor,
    PolicySchema,
    SequentialExecution,
    WebRTCCameras,
)
from novapolicy.lerobot import load_execution_settings

NOVA_HOST = "http://<nova-host>"
LEROBOT_SERVER = "<lerobot-server-host>:8080"
SERVER_CHECKPOINT = "/models/my_lerobot_policy"
CLIENT_CHECKPOINT_CONFIG = "./my_lerobot_policy/config.json"
CAMERA_API = "http://<camera-host>:8011/webrtc-streamer"
CAMERA_DEVICE = "<scene-camera-device-id>"
FPS = 15.0
PLAYBACK_SPEED = 1.0


async def main() -> None:
    settings = load_execution_settings(CLIENT_CHECKPOINT_CONFIG)
    cameras = WebRTCCameras(api_url=CAMERA_API, resize=(320, 240))

    async with Nova(config=NovaConfig(host=NOVA_HOST)) as nova:
        arm = (await nova.cell("cell").controller("cobot"))[0]
        schema = PolicySchema(
            observations=[
                Observation.joint_positions("arm", source=arm),
                Observation.io(
                    "gripper",
                    source=arm,
                    io="digital_out[0]",
                    mapping=BoolMapping(),
                ),
                Observation.image(
                    "cam_scene_1",
                    source=cameras.device(CAMERA_DEVICE),
                ),
            ]
        )
        policy = LeRobotPolicyClient(
            server_address=LEROBOT_SERVER,
            pretrained_name_or_path=SERVER_CHECKPOINT,
            policy_type=settings.policy_type,
            fps=FPS,
            playback_speed=PLAYBACK_SPEED,
            actions_per_chunk=settings.chunk_size,
            device="cuda",
        )
        executor = PolicyExecutor(
            schema,
            policy,
            execution=SequentialExecution(),
            n_action_steps=settings.n_action_steps,
            timeout_s=80,
        )
        result = await executor.run()
        print(f"Stopped: {result.reason}; steps={result.steps}; duration={result.duration_s:.1f}s")


if __name__ == "__main__":
    asyncio.run(main())
```

Run it while the async server is still listening:

```bash
# Terminal 2 — NOVA client host
python run_lerobot_policy.py
```

This example uses settled ACT execution: infer a chunk, execute the checkpoint-defined action
horizon, reach standstill, and infer again. If the checkpoint has no camera input, remove
`WebRTCCameras` and `Observation.image(...)`. Observation names and ordering must still match the
checkpoint dataset.

### Continuous asynchronous execution

To keep a fixed-rate LeRobot action queue while NOVA continuously replaces the active lookahead,
keep the same schema and use the following policy and executor options:

```python
from novapolicy import ContinuousExecution
from novapolicy.lerobot import AsyncQueueAggregation

policy = LeRobotPolicyClient(
    server_address=LEROBOT_SERVER,
    pretrained_name_or_path=SERVER_CHECKPOINT,
    policy_type=settings.policy_type,
    fps=FPS,
    playback_speed=PLAYBACK_SPEED,
    actions_per_chunk=settings.chunk_size,
    device="cuda",
    use_async_queue=True,
    async_queue_aggregation=AsyncQueueAggregation.AVERAGE,
    async_queue_smoothing=True,
)
executor = PolicyExecutor(
    schema,
    policy,
    execution=ContinuousExecution(rate_hz=FPS * PLAYBACK_SPEED),
    n_action_steps=0,
    timeout_s=80,
)
result = await executor.run()
```

Use sequential execution first. Enable the continuous queue only when the checkpoint and robot task
have been validated with overlapping action chunks.

## Configuration reference

### `server_address`

Address of the LeRobot async inference server in `"host:port"` form.

Examples:

```python
server_address = "127.0.0.1:8080"
server_address = "gpu-server.internal:8080"
```

### `pretrained_name_or_path`

Checkpoint path or Hugging Face model id passed to the LeRobot server.

Important: for a remote server, this is resolved on the server machine, not on the NOVA client.
The client does not upload model weights.

```python
pretrained_name_or_path = "/models/my_lerobot_policy"
pretrained_name_or_path = "org/my-policy"
```

### `policy_type`

LeRobot policy type sent to the server, for example:

```python
policy_type = "act"
```

The NOVA client-side decoder is policy-architecture agnostic as long as the server returns a flat
action vector matching the schema-derived joint, TCP, and IO layout.

### `fps`

Control/dataset frequency used by the NOVA client to time returned actions:

```python
fps = 15  # ActionChunk.dt_ms = 1000 / 15
```

This should match the policy's intended control rate. This is separate from the LeRobot server's
optional `--fps` CLI flag. The server flag is used by LeRobot when it creates `TimedAction`
timestamps; `LeRobotPolicyClient` does not use those timestamps. It decodes the returned action
tensors into NOVA `ActionChunk`s and sets the chunk `dt_ms` from this client-side `fps`.
`PolicyExecutor` then uses that `dt_ms` when scheduling the returned chunk.

### `playback_speed`

Explicit physical playback speed relative to the dataset rate:

```python
playback_speed = 0.75  # execute 25% slower: 15 Hz dataset actions use 88.89 ms intervals
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

Applications should pass these values to ``LeRobotPolicyClient.actions_per_chunk`` and
``PolicyExecutor.n_action_steps``. Explicit application configuration may override checkpoint
metadata.

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
    execution=SequentialExecution(),
    n_action_steps=settings.n_action_steps,
)
```

This prevents the policy observation from being captured while the robot is still moving and avoids
executing the lower-confidence tail beyond the checkpoint's configured execution horizon. The
`SequentialExecution` automatically waits for exact NOVA standstill and, if ACT's first target is
farther away than the spacing inside its executed horizon, prepends a
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
    async_queue_smoothing=False,
)
```

Aggregation is applied only when an old and a new action target the same future timestep:

| Mode | Merge |
|---|---|
| `WEIGHTED_AVERAGE` | `0.3 * old + 0.7 * new` (LeRobot default) |
| `AVERAGE` | arithmetic mean of every prediction received for the timestep |

The generic client retains LeRobot's weighted-average default. Physical UR3 plug-task runs used
`AVERAGE`, which showed lower peak path curvature. Those runs also enabled ``async_queue_smoothing``, which applies the
generic ``novapolicy.smooth_action_chunk(...)`` transform to the outgoing aggregated
lookahead. The four-point active prefix is restored unchanged after filtering. The generic client
leaves this disabled, and IO action values are never filtered.

The client normally consumes one action each policy control tick, requests a refill when 75% of the
previous chunk remains by default, and merges overlapping actions using the selected enum mode.
Refills use LeRobot's `must_go` flag because the server's default one-radian observation similarity
tolerance would otherwise defer most ACT inference until queue depletion. Configure `PolicyExecutor` with
`execution=ContinuousExecution(rate_hz=fps * playback_speed)` and `n_action_steps=0`.
Continuously replaced chunks do not expose per-chunk endpoint ramps.

NOVA's jogger clock advances independently of the Python control loop. Before consuming an action,
the executor maps the latest acknowledged raw NOVA controller timestamp to the absolute LeRobot
timestep; if local work delayed a tick, the client drops every action whose execution time elapsed.
Threshold-triggered inference remains asynchronous while NOVA executes its published lookahead. It
is merged on a later controller-synchronized tick instead of blocking after a timestep has already
been selected. The client then prepends the predecessor from NOVA's published trajectory and retains
the selected action plus two immutable successors. The replacement therefore contains an exact
four-point seam preserving position, velocity, and one-step acceleration context before aggregation
begins. IO remains sourced from the selected current action, not the prepended predecessor.

Between inference updates the client consumes actions internally without resending a shrinking
tail, so existing NOVA waypoints keep their original timeline. The initial queue prediction receives
a measured-state bridge and execution waits for its exact policy-zero boundary. The timestamp
already assigned to policy waypoint zero becomes the immutable action-timestep origin. Later merged
lookaheads use ``origin + action_timestep * policy_dt`` directly in the raw controller-timer domain.
Client wall time, server/client speed-ratio estimation, and post-boundary re-origining are not part of
queue timestamp calculation. Only each integer timestamp sent to NOVA is quantized. The overlapping
prefix is retained instead of being restarted from a measured-state hold at ``now``. This prevents
both catch-up motion and repeated zero-velocity braking/acceleration. In Rerun, the four-point
retained replacement seam is shown in Nova Violet while fresh policy output remains orange. This is
still LeRobot async ACT queue execution, not model-side RTC.

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

Returned LeRobot actions use a fixed flat layout: joint targets first, then TCP targets, then IO
actions. Joint targets are split according to the schema's joint action motion groups. Each TCP
target contributes six values in NOVA's native format: `[x, y, z, rx, ry, rz]` in millimetres and
rotation-vector radians. IO actions are written once per returned chunk using the first action step.

A motion group can be controlled through joints or TCP, but not both. To observe joints while
controlling the same robot in Cartesian space, disable the inferred joint action explicitly:

```python
schema = PolicySchema(
    observations=[
        Observation.joint_positions("arm", source=arm, action=False),
        Observation.tcp("eef", source=arm, tcp="Flange", action=True),
        Observation.io("gripper", source=arm, io="digital_out[0]", mapping=BoolMapping()),
    ]
)
```

For this schema the action vector is `[eef_x, eef_y, eef_z, eef_rx, eef_ry, eef_rz, gripper]`.

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

The returned flat action vector must contain the total DOF of all joint action motion groups, six
values for every TCP action target, and one value for every IO action. For a single 6-axis arm with
one gripper IO action, the policy should return seven action values per step. A TCP-controlled arm
with one gripper IO also returns seven values: six Cartesian components followed by the IO value.
