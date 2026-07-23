# GR00T Real-Time Chunking (RTC) — Investigation & Implementation Guide

## Status: ✅ VERIFIED WORKING (server-side, GPU-resident)

The production design is **server-side, options-driven** (not the PR #320 wrapper):

- The **client** (`Gr00tPolicyClient` + `compute_rtc_options`) computes the RTC
  timing `options` (`rtc_overlap_steps`, `rtc_frozen_steps`, `rtc_ramp_rate`,
  `action_horizon`, `denoising_steps`) and sends them with each `get_action`.
- The **server** (`Gr00tPolicy._get_action`, patched) keeps the previous
  normalized action tensor on the GPU (`self._rtc_prev_action`) and injects it
  into `collated_inputs["inputs"]["action"]` so the diffusion head inpaints.
- The previous action is **always stored** after inference — not gated on
  `options` (that gating was the original bug: the first call has `options=None`,
  so nothing was ever stored and injection never happened).

**Verification (synthetic, direct ZMQ):** requesting a full freeze
(`rtc_overlap_steps = rtc_frozen_steps = action_horizon = 16`) makes the new
chunk reproduce the previous chunk **bit-exactly**:

```
full-freeze  mean|chunk2 - chunk1| = 0.000000 rad   (frozen reproduces prev)
fresh call   mean|chunk3 - chunk1| = 0.004311 rad   (natural diffusion variance)
```

Note: freezing is in **normalized** space. `decode_action` is **state-dependent**,
so the same frozen normalized values decode to different physical values if the
observation state changes between calls. On the real robot the state is
continuous, so decoded motion stays smooth. (A naive test that jumps the joint
state between calls will *appear* to show no freezing — it's the decode, not RTC.)

### Two bugs that previously caused disconnected chunks
1. **Server storage gating** — `self._rtc_prev_action` was only stored when
   `options is not None`; the first call has no options → never stored → never
   injected. Fixed by always storing.
2. **Client legacy timestamp placement** — `PolicyExecutor` re-placed every
   chunk at "now" (legacy relative mode). The server's frozen waypoints only
   line up on the **absolute** timeline, so the executor now anchors overlapping
   chunks on raw NOVA session timestamps, backdated by
   `seam_backdate_steps` — see `novapolicy/chunking.py::placement`.

## What is RTC?

Real-Time Chunking (RTC) is a technique that improves action smoothness during streaming inference. Instead of predicting actions from pure noise on every call, the model uses the **previous prediction** as a warm start for the diffusion denoising process. This enables overlapping action chunks with smooth blending between consecutive predictions.

**Without RTC**: Each inference starts from random noise → independent, potentially jerky action chunks.

**With RTC**: Each inference starts from the tail of the previous prediction → temporally consistent, smooth transitions between chunks.

## How RTC Works (Architecture)

### Model Level (`gr00t/model/gr00t_n1d7/gr00t_n1d7.py`)

The diffusion action head (`get_action_with_features`) checks if `"action"` is present in `action_input`:

```python
if "action" in action_input:
    # RTC mode: Use previous action as initialization instead of pure noise
    # 1. Copy the tail of the previous action into the first N steps
    actions[:, :rtc_overlap_steps, :] = action_input["action"][:, -rtc_overlap_steps:, :]
    
    # 2. Freeze the first frozen_steps (latency compensation — don't change these)
    vel_strength[:, :rtc_frozen_steps, :] = 0.0
    
    # 3. Exponential ramp from frozen → overlap (gradual blend)
    vel_strength[:, rtc_frozen_steps:rtc_overlap_steps, :] = ramp
    
    # 4. Full denoising for steps beyond overlap
    # (vel_strength stays 1.0)
```

Required `options` dict:
| Parameter | Type | Description |
|-----------|------|-------------|
| `action_horizon` | int | Length of action chunk (e.g., 16) |
| `rtc_overlap_steps` | int | Steps from previous prediction to reuse as warm start |
| `rtc_frozen_steps` | int | Steps to freeze completely (latency compensation) |
| `rtc_ramp_rate` | float | Exponential ramp rate for blending (higher = faster transition to full denoising) |

### Client-Side Wrapper (PR #320 — `RTCPolicyWrapper`)

From [NVIDIA/Isaac-GR00T#320](https://github.com/NVIDIA/Isaac-GR00T/pull/320):

```python
class RTCPolicyWrapper(BasePolicy):
    def __init__(
        self,
        policy,
        control_freq,
        denoising_steps=8,
        max_rtc_overlap_factor=0.75,
        latency_queue_size=10,
        systematic_latency_offset=0.02,
    ): ...

    def get_action(self, observation):
        # On subsequent calls: merge previous action INTO observation
        if self._previous_action is not None:
            observation = {**observation, **self._previous_action}
            config = self._get_config()
        else:
            config = None

        action = self.policy.get_action(observation, config)
        self._previous_action = action.copy()
        return action

    def _get_config(self):
        avg_latency = sum(self.latency_queue) / len(self.latency_queue)
        frozen_steps = int(avg_latency * self.control_freq)
        executed_steps = int(self.control_freq * between_inference_time)
        max_rtc_steps = self._action_horizon * self._max_rtc_overlap_factor

        overlap_steps = int(
            max(
                min(self._action_horizon - executed_steps + frozen_steps, max_rtc_steps),
                frozen_steps,
            )
        )

        return {
            "denoising_steps": self.denoising_steps,
            "rtc_overlap_steps": overlap_steps,
            "rtc_frozen_steps": frozen_steps,
        }
```

### ZMQ Server Protocol

In `gr00t/novapolicy/server_client.py`:
```python
# Server registers: self.policy.get_action as "get_action" endpoint
# Client sends: {"endpoint": "get_action", "data": {"observation": ..., "options": ...}}
# Server calls: policy.get_action(observation=obs, options=opts)
```

## Current Gap: `Gr00tPolicy._get_action` Does NOT Forward `options`

**Critical finding**: In `gr00t/novapolicy/gr00t_policy.py` (line 408):
```python
def _get_action(self, observation, options=None):  # options says "currently unused"!
    ...
    model_pred = self.model.get_action(**collated_inputs)  # ← options NOT passed!
```

The model's `get_action(inputs, options=None)` accepts options, but `Gr00tPolicy` doesn't forward them. The `RTCPolicyWrapper` from PR #320 was designed to work at the **wrapper level** — it merges the previous action into the observation dict so the collate function includes it in `action_input`, and passes the config as `options`.

**For RTC to work via ZMQ, the server-side `Gr00tPolicy._get_action` must be patched**:
```python
# Current (broken for RTC):
model_pred = self.model.get_action(**collated_inputs)

# Fixed:
model_pred = self.model.get_action(**collated_inputs, options=options)
```

## How to Enable RTC on Our Deployment

### Step 1: Server-Side Patch

The GR00T inference server needs `Gr00tPolicy._get_action` modified to forward `options`:

```python
# In gr00t/novapolicy/gr00t_policy.py, line ~408:
with torch.inference_mode():
    model_pred = self.model.get_action(**collated_inputs, options=options)
```

Additionally, the previous action must flow through the processor into `action_input`. This likely requires:
- The observation to include action arrays (from the previous prediction)
- The collate function to recognize action keys and include them in `action_input`

### Step 2: Client-Side Changes (`Gr00tPolicyClient`)

Our `Gr00tPolicyClient` needs an RTC wrapper that:

1. **Stores the previous normalized action** returned by the server
2. **Merges it into the observation** on subsequent calls (key: `"action"`)
3. **Computes RTC parameters** based on measured inference latency
4. **Passes them as `options`** to the ZMQ `get_action` endpoint

### Step 3: Configuration

Add RTC parameters to the client/executor:

```python
client = Gr00tPolicyClient(
    host="gpu-server",
    port=30555,
    rtc=True,  # Enable RTC
    control_freq=20,  # Policy rate (Hz) — matches ContinuousExecution.rate_hz
    max_rtc_overlap_factor=0.75,  # Max fraction of chunk to overlap
    rtc_ramp_rate=3.0,  # Denoising ramp rate
    denoising_steps=8,  # DiT denoising iterations
)
```

## Implementation Plan

### Option A: Server-side RTCPolicyWrapper (Recommended)

Wrap the `Gr00tPolicy` with `RTCPolicyWrapper` **on the server** before starting `PolicyServer`:

```python
from gr00t.policy import Gr00tPolicy
from gr00t.eval.robot.rtc_policy import RTCPolicyWrapper  # from PR #320

base_policy = Gr00tPolicy(embodiment_tag=..., model_path=..., device="cuda:0")
rtc_policy = RTCPolicyWrapper(
    base_policy,
    control_freq=20,  # must match ContinuousExecution.rate_hz
    denoising_steps=8,
    max_rtc_overlap_factor=0.75,
)
server = PolicyServer(rtc_policy, port=30555)
server.run()
```

**Pros**: No client changes needed. RTC is transparent.
**Cons**: Server must know `control_freq` upfront; latency measurement only sees ZMQ overhead, not end-to-end.

### Option B: Client-side RTC in `Gr00tPolicyClient` (Our wrapper)

Implement RTC in our `Gr00tPolicyClient.get_actions()`:

1. Store previous raw action response (normalized numpy arrays)
2. On next call, include them as `action` key in observation state dict
3. Compute overlap/frozen steps from measured latency
4. Pass as `options` to the ZMQ call

**Pros**: Full control over latency measurement (end-to-end including cameras). 
**Cons**: Requires server-side `_get_action` patch to forward `options`.

### Option C: Hybrid

- Client sends previous action as part of observation (ensures `action_input` is populated)
- Client sends RTC config as `options`
- Server patches `Gr00tPolicy._get_action` to forward `options`

This is what PR #320's design implies: the wrapper lives wherever latency is measured.

## Key Parameters for Our Setup

| Parameter | Value | Rationale |
|-----------|-------|-----------|
| `control_freq` | 20 | Matches `ContinuousExecution(rate_hz=20)` |
| `action_horizon` | 16 | GR00T N1.7 default |
| `n_action_steps` | 8 | Receding horizon (only execute first 8 of 16) |
| `denoising_steps` | 8 | Default; can try 4 for lower latency |
| `max_rtc_overlap_factor` | 0.75 | 12 of 16 steps overlap (conservative) |
| `rtc_ramp_rate` | 3.0 | Default exponential ramp |
| `systematic_latency_offset` | 0.02 | 20ms systematic offset |

## References

- [PR #320: Realtime chunking implementation](https://github.com/NVIDIA/Isaac-GR00T/pull/320)
- Model code: `gr00t/model/gr00t_n1d7/gr00t_n1d7.py:330-380`
- Server: `gr00t/novapolicy/server_client.py` (PolicyServer/PolicyClient)
- Policy API: `gr00t/novapolicy/gr00t_policy.py` (Gr00tPolicy._get_action)
