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

The server listens first. It loads the policy only after a client connects and sends
`SendPolicyInstructions`.

Optional LeRobot server timing flags:

- `--fps`: controls LeRobot's server-side `TimedAction` timestamps. NOVA uses the
  `LeRobotPolicyClient(fps=...)` value for `ActionChunk.dt_ms`, so keep them aligned if you set both.
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

    policy = LeRobotPolicyClient(
        server_address="<lerobot-server-host>:8080",
        pretrained_name_or_path="/models/my_lerobot_policy",  # path on the LeRobot server
        policy_type="act",
        fps=15,
        actions_per_chunk=8,
        device="cuda",  # server-side torch device: cuda, mps, or cpu
    )

    executor = PolicyExecutor(schema, policy, n_action_steps=8, timeout_s=10)
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
timestamps; `LeRobotPolicyClient` decodes the returned action tensors into NOVA `ActionChunk`s and
uses this client-side `fps` for the chunk `dt_ms`.

### `actions_per_chunk`

Number of action steps requested from the server.

The LeRobot async server does not infer this. It is part of LeRobot's `RemotePolicyConfig`, and the
server slices `policy.predict_action_chunk(...)` to the requested length.

### `device`

Server-side torch device. This is the device on the machine running the LeRobot server, not the
NOVA/IPC/client machine.

Typical values:

- `"cuda"` when the LeRobot server runs on an NVIDIA GPU host
- `"mps"` when the LeRobot server runs on Apple Silicon
- `"cpu"` for CPU-only testing

The LeRobot async protocol does not expose a hardware-capability RPC. Treat this as deployment
configuration. To check the server host:

```bash
nvidia-smi

python - <<'PY'
import torch
print('cuda available:', torch.cuda.is_available())
print('mps available:', torch.backends.mps.is_available())
PY
```

Passing the wrong value usually fails during `SendPolicyInstructions`, when the server calls
`policy.to(device)`.

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

Returned LeRobot actions are decoded as flat joint targets and split according to the schema's joint
action motion groups. IO/TCP actions from a flat LeRobot action vector are not decoded yet.

## Protocol notes

The LeRobot async server flow is:

1. `Ready`
2. `SendPolicyInstructions`
3. `SendObservations`
4. `GetActions`

`SendPolicyInstructions` contains a pickled LeRobot `RemotePolicyConfig` with:

- `policy_type`
- `pretrained_name_or_path`
- `lerobot_features`
- `actions_per_chunk`
- `device`
- `rename_map` (left empty by this client)

The server does not expose model metadata before setup. Keep `policy_type`, `fps`,
`actions_per_chunk`, and `device` in your deployment configuration next to the checkpoint path.

## Troubleshooting

### The server cannot load the checkpoint

Check that `pretrained_name_or_path` exists on the LeRobot server machine, not just on the NOVA
client machine.

### CUDA/MPS/device errors during `SendPolicyInstructions`

Check the LeRobot server host and set `device` accordingly. For a remote NVIDIA GPU server, use
`device="cuda"`. For a local Mac server, use `device="mps"` only if MPS is available, otherwise use
`device="cpu"`.

### Image shape or missing image errors

`LeRobotPolicyClient` needs the first image frame to declare LeRobot feature metadata. Make sure the
camera is connected and that the image key in `Observation.image(...)` matches the key expected by
the checkpoint.

### Actions are the wrong dimension

The returned flat action vector must match the total DOF of the schema's joint action motion groups.
For a single 6-axis arm, the policy should return six action values per step.
