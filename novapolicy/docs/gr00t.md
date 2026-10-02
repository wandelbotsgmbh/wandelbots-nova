# Gr00tPolicyClient

ZMQ transport for [NVIDIA Isaac GR00T](https://github.com/NVIDIA/Isaac-GR00T) inference servers.

Implements the same REQ/REP msgpack protocol as `gr00t.policy.server_client.PolicyServer`, so `Gr00tPolicyClient` is a drop-in replacement for `gr00t.policy.server_client.PolicyClient` — but integrated with the NOVA `PolicyExecutor` lifecycle.

## Usage

```python
from novapolicy import (
    BoolMapping,
    ContinuousExecution,
    Gr00tPolicyClient,
    Observation,
    PolicyExecutor,
    PolicySchema,
    RTCConfig,
    TcpFormat,
)

schema = PolicySchema(
    observations=[
        Observation.joint_positions("left_arm", source=mg_left),
        Observation.tcp("left_eef_9d", source=mg_left, format=TcpFormat.ROT6D),
        Observation.joint_positions("right_arm", source=mg_right),
        Observation.tcp("right_eef_9d", source=mg_right, format=TcpFormat.ROT6D),
        Observation.io(
            "left_gripper", source=mg_left, io="digital_out[0]", mapping=BoolMapping(on=100.0)
        ),
        Observation.constant("language", value="Pick up the box."),
    ]
)

client = Gr00tPolicyClient(host="gpu-server", port=5555, dt_ms=66.7, rtc=RTCConfig())

executor = PolicyExecutor(
    schema,
    client,
    execution=ContinuousExecution(rate_hz=20),
    n_action_steps=8,
)
result = await executor.run()
```

The client uses `PolicySchema` observations to build GR00T-compatible numpy array observations and decode the returned action arrays.

## Real-Time Chunking (RTC)

RTC reuses the tail of the previous action prediction as a warm start for the
diffusion denoising process, producing smoother overlapping action chunks.
See [`rtc.md`](./rtc.md) for the full investigation and design.

### Server-side patch (required)

The GR00T inference server must forward `options` to the model. Apply this
one-line patch to `gr00t/novapolicy/gr00t_policy.py` in the server's Isaac-GR00T
checkout:

```diff
-            model_pred = self.model.get_action(**collated_inputs)
+            model_pred = self.model.get_action(**collated_inputs, options=options)
```

Alternatively, inject it at container startup via sed (no rebuild needed):

```yaml
# In the deployment args, before the server starts:
sed -i 's/model_pred = self.model.get_action(\*\*collated_inputs)/model_pred = self.model.get_action(**collated_inputs, options=options)/' /workspace/gr00t/novapolicy/gr00t_policy.py
```

### Client-side usage

```python
from novapolicy import ContinuousExecution, Gr00tPolicyClient, PolicyExecutor, RTCConfig

# Enable RTC
client = Gr00tPolicyClient(
    host="gpu-server",
    port=30555,
    dt_ms=66.7,
    rtc=RTCConfig(),  # pass RTCConfig to enable, None (default) to disable
)

# RTC requires continuously replaced chunks.
executor = PolicyExecutor(
    schema,
    client,
    execution=ContinuousExecution(rate_hz=20),
    n_action_steps=8,
)
```

RTC can be toggled at runtime:

```python
client.rtc = None  # disable — next calls use pure diffusion from noise
client.rtc = RTCConfig()  # re-enable
client.reset_rtc()  # clear stored action (call on episode/task boundaries)
```

### RTCConfig parameters

| Parameter | Default | Description |
|-----------|---------|-------------|
| `denoising_steps` | `8` | Diffusion denoising iterations |
| `max_overlap_factor` | `0.75` | Max fraction of action horizon to overlap |
| `ramp_rate` | `3.0` | Exponential blending ramp speed |
| `systematic_latency_offset` | `0.02` | Fixed latency added to measured inference time (s) |

## Wire Protocol

Uses the [GR00T REQ/REP msgpack protocol](https://github.com/NVIDIA/Isaac-GR00T/blob/main/gr00t/novapolicy/server_client.py):

- **Endpoints**: `ping`, `get_action`, `reset`, `get_modality_config`
- **Observations**: numpy arrays serialized as `.npy` bytes inside msgpack
- **Actions**: returned as `(action_dict, info_dict)` tuple

## Observation Keys

The `key` argument in each `Observation.*()` call becomes the GR00T state key:

```python
Observation.joint_positions("left_arm", source=mg)  # → obs["state.left_arm"]
Observation.tcp("left_eef_9d", source=mg, format=TcpFormat.ROT6D)  # → obs["state.left_eef_9d"]
Observation.io("left_gripper", source=mg, io="digital_out[0]")  # → obs["state.left_gripper"]
```

### Computed observations and actions

`Action.computed` fires for every policy (the executor runs it after each call).
`Observation.computed` also fires for GR00T, but its values are merged into GR00T's
numeric state payload — so each computed value must be a scalar/sequence of floats keyed
to a model state input. Non-numeric outputs (strings, images) are only supported on the
callback (bring-your-own-function) path.

## Inspecting a Server

Before writing your schema, query the server to see what it expects:

```python
import asyncio
from novapolicy import Gr00tPolicyClient


async def main():
    client = Gr00tPolicyClient(host="gpu-server", port=5555)
    await client.connect([])

    info = await client.get_server_info()
    print(info)
    # {
    #   'state_keys': ['left_joint_positions', 'right_joint_positions'],
    #   'action_keys': ['left_joint_positions', 'right_joint_positions'],
    #   'video_keys': ['exterior_image_1', 'exterior_image_2', 'left_wrist_image', 'right_wrist_image'],
    #   'language_keys': ['annotation.language.language_instruction'],
    #   'action_horizon': 16,
    #   'action_configs': [
    #     {'rep': 'RELATIVE', 'type': 'NON_EEF', 'format': 'DEFAULT', 'state_key': 'left_joint_positions'},
    #     {'rep': 'RELATIVE', 'type': 'NON_EEF', 'format': 'DEFAULT', 'state_key': 'right_joint_positions'},
    #   ],
    # }

    await client.close()


asyncio.run(main())
```

Use this to:
- Match your `Observation.*()` keys to `state_keys` and `video_keys`
- Confirm the `action_horizon` (chunk size)
- Check `action_configs` for `RELATIVE` vs `ABSOLUTE` action mode

> **Note:** The server does not report `dt_ms` (step timing). This must be set
> on the client to match the training data rate. For example, data recorded at
> 15 Hz → `dt_ms=66.7`:
>
> ```python
> client = Gr00tPolicyClient(host="gpu-server", dt_ms=66.7)
> ```

## Debugging Chunk Alignment

To diagnose timing issues (snap-back, overshoot, wrong playback speed), monkey-patch the client's `get_actions` to log per-step diagnostics:

```python
import math, time

_orig_get = client.get_actions
_step_count = 0
_last_time = time.monotonic()


async def _logged_get(states, schema, images, io_values):
    global _step_count, _last_time
    _step_count += 1
    gap = time.monotonic() - _last_time

    t0 = time.monotonic()
    result = await _orig_get(states, schema, images, io_values)
    inference_ms = (time.monotonic() - t0) * 1000
    _last_time = time.monotonic()

    if hasattr(result, "joints") and result.joints:
        skip = inference_ms / result.dt_ms if result.dt_ms > 0 else 0
        n = len(next(iter(result.joints.values())))
        print(
            f"Step {_step_count} | gap={gap:.2f}s | inference={inference_ms:.0f}ms | skip~{skip:.1f}/{n}"
        )

        for mg_id, steps in result.joints.items():
            obs = states.get(mg_id)
            if obs and steps:
                current = list(obs.joints)
                snap = max(abs(math.degrees(steps[0][j] - current[j])) for j in range(len(current)))
                eff_idx = min(int(skip), len(steps) - 1)
                eff_delta = max(
                    abs(math.degrees(steps[eff_idx][j] - current[j])) for j in range(len(current))
                )
                print(
                    f"  {mg_id}: snap={snap:.2f}\u00b0, effective[{eff_idx}] delta={eff_delta:.2f}\u00b0"
                )

    return result


client.get_actions = _logged_get
```

Key metrics to watch:
- **snap**: How far step[0] is from current position. Should be <1\u00b0. Large values indicate wrong `dt_ms` or missing `observation_time` alignment.
- **skip**: Steps skipped due to inference delay. Should be < action_horizon.
- **effective delta**: Distance from current position to the step the robot will actually target. Should be positive and growing (robot moving forward, not snapping back).
