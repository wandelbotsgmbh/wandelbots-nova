# NOVA Policy Service Adaptation Plan (Use `lerobot-inference` as Base)

## Context

We are pivoting from the current `nova_policy/policy-service` implementation to the colleague-provided app:

- Base repo: `/Users/stefanwagner/Git/lerobot-inference`
- Current (to be replaced): `wandelbots-nova/nova_policy/policy-service`

Key requested differences to address:

1. Policy source is environment-driven (not request-driven).
2. No mock data and no explicit observation payload in current app.
3. Policy app should **not** directly control robot motion long-term.
4. Control should happen in SDK extra (application layer), with a **bidirectional** loop:
   - SDK extra sends observations (robot state + camera refs/frames) to inference app.
   - Inference app returns action chunks.
   - SDK extra executes movement (jogging-style), similar to `lerobot_robot_nova` behavior.

---

## Status update (2026-04-24)

### Completed so far

- [x] Added the actual standalone phase-1 service in `/Users/stefanwagner/Git/lerobot-inference/policy_service`.
- [x] Switched the actual implementation focus from the old `nova_app` wrapper to a standalone FastAPI control-plane service.
- [x] Kept policy source environment-driven via `POLICY_PATH` with `POLICY_KIND` and `PRELOAD_POLICY_ON_STARTUP` support.
- [x] Added HTTP control-plane endpoints in the actual service: `GET /healthz`, `GET /policy`, `GET /policies`, `GET /policies/{policy}`, `POST /policies/{policy}/start`, `POST /policies/{policy}/stop`, `GET /policies/{policy}/runs/{run}`.
- [x] Reused the real `lerobot-inference` runtime pieces (`nova_app/app/runtime.py`, `nova_app/app/config.py`, `nova_app/app/runner.py`) instead of the earlier mock/draft service code.
- [x] Updated the `lerobot-inference` Dockerfile so the default container entrypoint is now `python -m policy_service`.
- [x] Fixed ACT policy loading in the real repo by importing `lerobot.policies` before `PreTrainedConfig.from_pretrained(...)` is used.
- [x] Fixed FastAPI route ordering so `/policies/{policy}/runs/{run}` is no longer shadowed by `/policies/{policy}`.
- [x] Added/validated typed ACT discovery work in `wandelbots-nova/nova_policy` (`ACTPolicy`, `PolicyServiceClient.get_policy()`, explicit `PolicyExecutionContext`, `n_action_steps` override path).
- [x] Validated the actual phase-1 service locally against `https://spjhrikg.instance.wandelbots.io` using the virtual `ur10e` controller, dataset-backed cameras, and policy `StefanWagnerWandelbots/act_virtual_teleop_pickplace_easy`.
- [x] Observed real run lifecycle/log evidence: policy preload/load, Nova connect, camera connect, jogging start, and terminal run state (`PREPARING -> TIMED_OUT` for the short validation run).

### Additional phase-2 progress completed locally

- [x] Refactored the real `lerobot-inference` service into inference-only behavior for the active run path.
- [x] Added a Socket.IO data-plane in `/Users/stefanwagner/Git/lerobot-inference/policy_service/app.py` with:
  - `observation.push`
  - `action.chunk`
  - `session.state`
- [x] Added a dedicated inference engine in `/Users/stefanwagner/Git/lerobot-inference/policy_service/inference_engine.py` that:
  - loads ACT policy from env-driven `POLICY_PATH`
  - applies optional `n_action_steps` override from request policy payload
  - keeps camera ingestion internal to the service
  - predicts action chunks from external robot-state observations
- [x] Removed direct `robot.send_action(...)` execution from the standalone `policy_service` runtime path.
- [x] Added SDK-side typed realtime client primitives in `/Users/stefanwagner/Git/wandelbots-nova/nova_policy`:
  - `RobotStatePoint`
  - `ActionStep`
  - `ActionChunk`
  - `PolicyRealtimeSession`
  - `PolicyServiceClient.open_realtime_session()`
- [x] Added dependency wiring for Socket.IO:
  - `lerobot-inference` runtime now depends on `python-socketio`
  - `wandelbots-nova[nova-policy]` now includes `python-socketio` + `aiohttp`
- [x] Local validation completed for the new phase-2 split:
  - `GET /healthz` and `GET /policy` still work through the FastAPI control plane
  - runtime smoke test with a fake inference engine confirmed `PREPARING -> RUNNING`, `observation.push` -> typed `action.chunk`, and clean stop behavior
  - `PYTHONPATH=. uv run pytest -q nova_policy/tests` -> `7 passed`

### Phase-2 cluster realtime validation completed

- [x] Reworked the `lerobot-inference` Dockerfile to keep CUDA while avoiding the earlier oversized Python-slim image:
  - base image is now `nvidia/cuda:12.8.0-cudnn-runtime-ubuntu24.04`
  - PyTorch CUDA wheels are installed without duplicating the full NVIDIA wheel stack
  - resulting local image size is ~8.21 GiB instead of ~16 GiB
- [x] Built and pushed optimized realtime image `wandelbots.azurecr.io/ai/nova-policy-service:2026-04-24-05`.
- [x] Verified pushed image digest: `sha256:2327d5192a456dd27c17653ffe027c932ac2d9a7adecccd3e8af61098dfc2364`.
- [x] Deployed via Flux commit `cba18da454160fec759ced2b53a6a786ff4810c0` (`fix(nova-policy-service): use optimized cuda realtime image`).
- [x] Reconciled Flux in namespace `team-embodied-ai`; `apps-nova-policy-service` and `team-embodied-ai` reported `READY` at revision `main@sha1:cba18da4`.
- [x] Verified deployment uses image `wandelbots.azurecr.io/ai/nova-policy-service:2026-04-24-05` and rollout completed with `1/1` ready replica.
- [x] Verified public HTTP control-plane after rollout:
  - `GET /healthz` -> `200 {"status":"ok"}`
  - `GET /policy` -> configured ACT policy with `loaded=true`, `app_state="READY"`
- [x] Added explicit `allow_mock_images` start-request flag for Phase-2 smoke tests where policy expects visual inputs but the manual test intentionally sends only mock robot-state observations.
- [x] Validated Socket.IO websocket transport through the public cluster gateway using `transports=["websocket"]`.
- [x] Manual Phase-2 inference run succeeded on the cluster route:
  - observed run `run_03c221c7c3`
  - state transition `PREPARING -> RUNNING`
  - sent 20 mock joint/gripper observations via `observation.push`
  - received 20 `action.chunk` messages
  - returned joint vectors varied over time
  - stopped cleanly with terminal state `STOPPED`
- [x] Forced disconnect/reconnect smoke succeeded:
  - observed run `run_2a0fd8b028`
  - first Socket.IO connection received an action chunk
  - client disconnected intentionally
  - second Socket.IO connection resumed by run id and received another action chunk
  - run stopped cleanly with terminal state `STOPPED`

### SDK phase-3 continuation completed locally

- [x] Added a small SDK-side adapter boundary in `nova_policy/adapters.py`:
  - `PolicyAdapter`
  - `ACTAdapter`
  - `adapter_for_policy(...)`
- [x] Routed `MotionGroup.stream_policy(...)` start payload construction through `ACTAdapter` for ACT policies.
- [x] Added an opt-in realtime `MotionGroup.stream_policy(..., options=PolicyExecutionOptions(realtime=True))` loop that:
  - starts the HTTP control-plane run
  - waits for `RUNNING`
  - reads `MotionGroup.get_state(...)`
  - converts joints into `RobotStatePoint`
  - sends `observation.push` via `PolicyRealtimeSession.predict(...)`
  - receives and queues `action.chunk` steps
  - triggers the next observation when the queue reaches the configured low-water mark
  - supports `max_observations` for bounded smoke tests
  - supports stop/cancel through the existing `PolicyRunState.stop()` path
- [x] Added queue semantics for action chunks in SDK code:
  - enqueue all chunk steps
  - pop one step per loop when action execution is enabled
  - hold the last action step on underflow
- [x] Added `allow_mock_images` propagation from SDK options to the service start payload for cluster smoke tests.
- [x] Added/updated tests for ACT adapter payload mapping and realtime robot-state push behavior.
- [x] Validation after this continuation:
  - `PYTHONPATH=. uv run ruff check --config ruff.toml nova_policy/adapters.py nova_policy/motion_group_extensions.py nova_policy/client.py nova_policy/__init__.py nova_policy/tests/test_policy_extension.py` -> passed
  - `PYTHONPATH=. uv run pytest -q nova_policy/tests` -> `9 passed`

### SDK phase-4 initial jogging work completed locally

- [x] Added guarded SDK-side action application path behind `PolicyExecutionOptions(realtime=True, execute_actions=True)`.
- [x] Added joint-target normalization from `ActionStep.joints` to ordered NOVA joints.
- [x] Added proportional joint velocity calculation with configurable:
  - `joint_velocity_limit`
  - `joint_position_gain`
  - `joint_position_tolerance`
- [x] Added a first NOVA Jogging API sender that initializes jogging, sends one joint-velocity command, then pauses jogging.
- [x] Replaced the one-shot sender with a continuous `_JointJoggingSession` lifecycle that:
  - opens one NOVA Jogging websocket per realtime policy loop,
  - sends `InitializeJoggingRequest` once,
  - streams queued `JointVelocityRequest` commands,
  - sends `PauseJoggingRequest` on session close,
  - drains jogging responses while commands are streamed.
- [x] Added realtime option validation so unsafe/invalid combinations fail before a run starts:
  - `execute_actions=True` requires `realtime=True`
  - non-negative low-water mark and tolerance
  - positive max-observation count when provided
  - positive jogging velocity limit and position gain
- [x] Added realtime telemetry into yielded `PolicyRunState.metadata["realtime"]`:
  - `next_observation_seq`
  - `last_observation_seq`
  - `queued_action_steps`
  - `last_action_chunk`
  - `last_action_step` when execution is enabled
- [x] Added unit coverage for velocity clamping/tolerance behavior, continuous jogging request order, realtime metadata, option validation, and the guarded `execute_actions=True` realtime loop path.
- [x] Validation after this continuation:
  - `PYTHONPATH=. uv run ruff check --config ruff.toml nova_policy/motion_group_extensions.py nova_policy/tests/test_policy_extension.py` -> passed
  - `PYTHONPATH=. uv run pytest -q nova_policy/tests` -> `13 passed`

### Still intentionally not done

- [ ] Validate `execute_actions=True` against the virtual UR10e.
- [ ] Validate the continuous jogging lifecycle against virtual UR10e before production use.
- [ ] Full MotionGroup-native robot motion over returned `action.chunk` commands under safety-reviewed limits.
- [ ] Explicit auth validation for Socket.IO handshake headers/tokens; current cluster route accepted the unauthenticated manual websocket test just like the HTTP control-plane.

### Phase-1 deployment prep completed

- [x] Hardened the real `lerobot-inference` Docker build for private `code.wabo.run` dependencies using a BuildKit secret-mounted SSH key instead of the earlier ad-hoc auth setup.
- [x] Built and pushed image `wandelbots.azurecr.io/ai/nova-policy-service:2026-04-23-01`.
- [x] Verified pushed image digest: `sha256:7dd945c53824cc490db9f9f2e801428a3515053fa761796468e2e99fd18ee9cc`.
- [x] Updated Flux app manifests in `/Users/stefanwagner/Git/flux-apps/apps/nova-policy-service` with:
  - `POLICY_PATH=StefanWagnerWandelbots/act_virtual_teleop_pickplace_easy`
  - readiness/liveness probes on `/healthz`
  - ACR pull secret wiring
  - image tag `2026-04-23-01`
  - `BackendTrafficPolicy` for longer-lived realtime sessions
- [x] Render-validated the Flux kustomization locally via `kubectl kustomize apps/nova-policy-service`.

### Phase-1 cluster rollout completed

- [x] Committed and pushed Flux change `1c4888dafb69e53e1861f9d7d72d3460abf87763` in `flux-apps` (`feat(nova-policy-service): deploy phase-1 policy service image`).
- [x] Reconciled Flux source `physical-ai-flux-apps` and kustomization `team-embodied-ai` / `apps-nova-policy-service` in namespace `team-embodied-ai`.
- [x] Verified rollout success for deployment `nova-policy-service` on cluster context `developer-portal-gpucluster-dev`.
- [x] Verified the live route hostname: `https://nova-policy-service.ai.gpucluster-dev.wandelbots.io`.
- [x] Verified unauthenticated HTTP control-plane access through the public cluster gateway:
  - `GET /healthz` -> `200 {"status":"ok"}`
  - `GET /policy` -> configured ACT policy with `loaded=true`, `app_state="READY"`
- [ ] Still need to validate the same route explicitly from a NOVA-instance network path if infra requires that stronger check.
- [x] Verified service startup logs show successful policy preload on cluster without crash loops.
- [x] Verified `BackendTrafficPolicy` is attached and accepted by the gateway controller.
- [x] Verified current HTTP route behavior is not protected by additional auth for these GET endpoints.

### Known issue found during validation

- [ ] `observation.state` from the current Nova robot path is 6D while the validated ACT checkpoint expects 7D (`gripper.pos` missing). Current runner pads with zero and still runs, but this should be normalized explicitly in the future adapter/inference contract.

## Findings from code investigation

## 1) `lerobot-inference` already contains two execution modes

### A) Standalone script (`inference.py`)
- Loads policy and robot.
- Reads observation from robot (`robot.get_observation()`).
- Runs policy inference when action queue empty.
- Sends actions to robot directly (`robot.send_action(...)`).
- Supports chunked policy output and queue-based action consumption.

### B) Existing NOVA app wrapper (`nova_app/app/*`) — reference only
- Exposes NOVA program `start_inference`.
- Uses env var `POLICY_PATH` (`runtime.resolve_policy_path`).
- Builds robot config from runtime params (`controller_name`, `motion_group`, `tcp`, camera settings).
- Performs direct robot motion in-app (`runner.py` -> `robot.send_action(...)`).

Important: this wrapper is **not** our target runtime shape. We use it as implementation reference only. Final integration is called from the Python SDK extra (`wandelbots-nova/nova_policy`), not via NOVA program wrapper.

## 2) Policy loading behavior

In `nova_app`:
- policy path is resolved from env (`POLICY_PATH`), as requested.
- optional preload on startup (`PRELOAD_POLICY_ON_STARTUP=true`).

This aligns with your desired policy-source behavior.

## 3) Camera behavior

Current app assumes camera plugins (mostly WebRTC) and obtains observations via LeRobot robot abstraction.
- No mock observation path yet in `nova_app`.
- No inbound observation API from external client yet.

## 4) Robot execution pattern to reuse

From `/Users/stefanwagner/Git/wandelbots-lerobot/lerobot_robot_nova`:
- `NovaRobot.send_action` sets target joints and relies on jogging/velocity control loop under the hood.
- `NovaClient` + `JoggingController` manage websocket jogging lifecycle, restart, and e-stop handling.

This is the correct reference for SDK-side execution semantics.

## 5) Gap vs desired architecture

Current `lerobot-inference` is effectively **closed-loop in one process** (observe + infer + execute).
Desired target is **split-loop**:
- Inference app: observe(+cameras) + infer only.
- SDK extra: execute actions on robot.

That requires a new bidirectional transport/API contract.

## 6) Action queue pattern from `lerobot-inference` (to inherit)

`lerobot-inference/inference.py` already implements a **queue-based action consumption** model that handles the mismatch between inference latency and control cadence:
- Policy inference produces a **chunk** of N action steps at once.
- Steps are enqueued and consumed one-per-control-tick at `control_dt_s` cadence.
- New inference is triggered only when the action queue is empty (or below a threshold).
- This decouples inference time (e.g. 200ms) from execution rate (e.g. 50ms per step).

The SDK-side execution loop **must inherit this queue-based pattern**:
1. On receiving an `action.chunk`, enqueue all steps.
2. Consume one step per `control_dt_s` tick via the jogging-style path.
3. When the queue drains (or falls below a configurable low-water mark), push the next `observation.push` to trigger a new inference.
4. If inference is still in-flight when the queue drains, hold the last commanded position (zero-velocity hold) until the next chunk arrives.

This is the primary mechanism for handling inference latency > control period and must be explicitly implemented in both service-side chunk production and SDK-side chunk consumption.

---

## Target architecture (adapted)

## Control-plane and data-plane split

### Inference App (service)
Responsibilities:
- Load/preload policy from env (`POLICY_PATH`).
- Maintain policy runtime state.
- Accept observation messages (task + robot state only: joints/gripper/optional tcp/io).
- Return action chunks + timing metadata.
- No robot command execution.
- Be deployable as both:
  - a NOVA app install target (`nova install`) on an instance,
  - and a Flux/Kubernetes deployment target on GPU cluster.

Current limitation (explicit): one service instance supports one configured policy path.
- `POLICY_PATH` defines the active policy for that deployment.
- Supporting multiple policies currently means deploying multiple inference app instances.

### SDK Extra (`nova_policy`)
Responsibilities:
- Own execution loop and stop/cancel semantics.
- Collect observations from MotionGroup/robot state (joints/gripper/optional tcp/io).
- Send robot-state observations to inference app.
- Receive chunked actions.
- Execute actions via jogging-style path (Nova/MotionGroup side).

---

## Proposed API contract (new, bidirectional)

Use a **hybrid transport**:
- **HTTP control-plane** for non-realtime operations (`healthz`, `get_policy`, optional start/stop lifecycle calls).
- **WebSocket data-plane** for realtime observation/action streaming.

Implementation choice: use a websocket library instead of hand-rolled protocol plumbing.
- Preferred: **Socket.IO** (`python-socketio`) for server + SDK client.
- Library handles reconnect/heartbeat/ack mechanics.
- Application protocol still defines safety semantics (`seq`, correlation, backpressure limits, safe hold/stop on disconnect).

Rationale: this matches NOVA’s existing realtime patterns (state streams + bidirectional jogging over websocket) while keeping control/discovery easy to operate via HTTP.

## A) Session/control channel (HTTP + socket events)
- HTTP:
  - `get_policy` (single configured policy per instance)
  - optional `session.start` / `session.stop` lifecycle calls
- Socket event:
  - `session.state` (streamed state updates)

Session contains:
- policy identity (resolved from env on service)
- task
- target metadata
- camera configuration metadata
- run state (`PREPARING/RUNNING/...`)

## B) Observation/action channel (socket / Socket.IO)
- `observation.push`
  Send one observation packet (joint/gripper/tcp/io state + timestamp + seq). No image payloads.

- `action.chunk`
  Stream back next action chunk and inference metadata.

Robot current state should also be sourced from a socket stream on SDK side and relayed to the policy service.

Action chunk response should include:
- `chunk_id`
- `steps` (policy-adapter normalized step list)
  - each step may include `joints`, `gripper`, `io`
- `n_action_steps`
- `control_dt_s` (execution cadence)
- optional `first_step_at_s` (absolute schedule anchor)
- `inference_latency_ms`
- `model_time`
- `policy_kind`
- optional diagnostics

### Transport responsibilities (library vs application)

Use `python-socketio` to avoid implementing low-level websocket lifecycle handling ourselves.

Handled by library:
- websocket/session lifecycle
- ping/pong heartbeat
- reconnect/backoff behavior
- event ack transport mechanics

Must still be defined and enforced by our protocol:
- `seq` monotonic observation numbering per session
- correlation of `action.chunk` to observation (`observation_seq`)
- backpressure policy (default max in-flight observations = 1)
- duplicate/drop handling on reconnect
- safety behavior on disconnect/underflow (SDK hold position or stop safely)

### Cross-environment gateway/auth implications (NOVA instance -> Flux GPU cluster)

From current Flux manifests (`flux-apps/apps/nova-policy-service`):
- Service is exposed via `HTTPRoute` on `envoy-shared` host `nova-policy-service.ai.${CLUSTER_ENV}.wandelbots.io`.
- No app-local auth policy is defined in this repo for that route.
- `BackendTrafficPolicy` with websocket-friendly idle timeout is currently not present for `nova-policy-service`.

Important caveat: auth/gateway defaults may be configured in the shared infrastructure repo (outside this app repo). Therefore, we must not assume the route is unauthenticated even if this repo contains no auth manifests.

Required implications for v1:
- Validate websocket upgrade + Socket.IO handshake through gateway from an actual NOVA instance network.
- Support auth on both planes:
  - HTTP control-plane (`Authorization: Bearer ...`)
  - Socket.IO handshake headers (same bearer token or dedicated service token)
- Add route timeout policy (`BackendTrafficPolicy`) if default gateway idle timeout closes long-lived realtime sessions.

Note: LeRobot policy families do not all behave identically (feature keys, chunk semantics, postprocessing expectations). We should define a per-policy-type adapter layer (e.g. ACT, PI0, Diffusion, SmolVLA) to normalize:
- inbound observation mapping
- model invocation/predict API
- returned action chunk format
- action name ordering and dimensionality

## Policy adapter interface (service-side)

```python
from dataclasses import dataclass
from typing import Protocol
import torch

@dataclass
class InferenceRequest:
    policy_path: str
    policy_kind: str
    task: str | None
    observation: dict[str, object]  # joints/gripper/tcp/io only
    overrides: dict[str, object]    # policy.* runtime overrides

@dataclass
class InferenceActionStep:
    joints: dict[str, float]
    gripper: dict[str, float] | None = None
    io: dict[str, bool | float] | None = None

@dataclass
class InferenceChunk:
    policy_kind: str
    chunk_id: str
    steps: list[InferenceActionStep]
    inference_latency_ms: float
    control_dt_s: float
    first_step_at_s: float | None = None

class PolicyTypeAdapter(Protocol):
    policy_kind: str

    def load(self, *, policy_path: str, overrides: dict[str, object]) -> None: ...
    def preprocess_observation(self, observation: dict[str, object], task: str | None) -> dict[str, torch.Tensor]: ...
    def predict_chunk(self, batch: dict[str, torch.Tensor]) -> InferenceChunk: ...
```

Adapter registry:
- `act` -> `ACTAdapter`
- `pi0`, `pi0_fast` -> dedicated adapter
- `diffusion` -> dedicated adapter
- others -> fallback adapter only if shape contract is verified

## ACT-specific parameter support (important)

From `lerobot/policies/act/modeling_act.py` + `configuration_act.py`, the only ACT parameter we should expose from SDK at runtime is:
- `n_action_steps`

Rationale:
- `device` should be service/runtime-owned (GPU cluster = CUDA, NOVA instance = CPU).
- `use_amp` should be service default/auto policy.
- `temporal_ensemble_coeff` should stay checkpoint/default-driven unless we add a dedicated use case later.
- `chunk_size` is model-structural for ACT checkpoints and should not be overridden at runtime.

For pretrained inference, architecture/training parameters are not runtime knobs and remain fixed from checkpoint config.

### Recommended config strategy

1. Keep explicit typed ACT runtime knob in SDK options:
   - `n_action_steps`
2. Service decides `device`/precision defaults based on deployment environment.
3. Service ACT adapter translates `n_action_steps` into LeRobot override (`cli_overrides`) when loading config.

Example conversion:
- `{"n_action_steps": 32}`
- -> `["--n_action_steps=32"]`

This keeps the contract minimal and deployment-safe.

## Python SDK-side API examples (explicit ACT first)

Important: v1 should expose an explicit ACT entrypoint/pattern. Future policy families (pi0, pi0.5, groot) are added with their own adapters and typed options.

Use a stronger pattern: **Polymorphic PolicySpec + Common ExecutionContext**.

- `policy` is a typed spec (`ACTPolicy`, later `PI05Policy`, `GrootPolicy`) that contains policy-family inference params.
- `PolicyExecutionContext` contains only common transport/runtime context (currently cameras), no policy-family knobs.
- SDK validates `policy` by type (discriminated union / sealed hierarchy).

This avoids factory helpers and keeps the API explicit and discoverable.

Example shape:

```python
@dataclass
class PolicyExecutionContext:
    cameras: dict[str, WebRTCCamera]

@dataclass
class ACTPolicy:
    path: str
    n_action_steps: int | None = None
```

For overrides, users update the typed policy object directly (immutable helper like `with_overrides(...)` can be added, but not required).

Prefer a typed `policy` object over a separate `policy_type` field, e.g.:
- `ACTPolicy(...)`
- later `PI05Policy(...)`, `GrootPolicy(...)`

This keeps the SDK API aligned with existing typed patterns in NOVA SDK and avoids duplicated `policy_type` + policy payload fields.

### A) Start ACT policy run (explicit, discovered policy)

```python
policy_client = nova_policy.PolicyServiceClient(base_url=policy_service_url)
policy = await policy_client.get_policy()
if not isinstance(policy, ACTPolicy):
    raise RuntimeError(f"Expected ACT policy, got {type(policy).__name__}")

run_policy = ACTPolicy(
    path=policy.path,
    n_action_steps=32,  # optional runtime override
)

state = await motion_group.execute_policy(
    policy=run_policy,
    task="pick and place",
    timeout_s=120,
    context=PolicyExecutionContext(
        cameras={
            "flange": {
                "type": "webrtc",
                "api_url": "http://camera-server:8081",
                "device_id": "World_UR10e_flange",
                "width": 640,
                "height": 480,
                "fps": 30,
            },
            "left": {
                "type": "webrtc",
                "api_url": "http://camera-server:8081",
                "device_id": "World_Camera_top",
                "width": 640,
                "height": 480,
                "fps": 30,
            },
        }
    ),
)
```

### B) Stream ACT run; SDK executes actions locally

```python
policy_client = nova_policy.PolicyServiceClient(base_url=policy_service_url)
policy = await policy_client.get_policy()
if not isinstance(policy, ACTPolicy):
    raise RuntimeError(f"Expected ACT policy, got {type(policy).__name__}")

stream_policy = ACTPolicy(
    path=policy.path,
    n_action_steps=16,  # optional runtime override
)

async for s in motion_group.stream_policy(
    policy=stream_policy,
    task="insert peg",
    timeout_s=90,
    context=PolicyExecutionContext(
        cameras={
            "flange": {
                "type": "webrtc",
                "api_url": "http://camera-server:8081",
                "device_id": "World_UR10e_flange",
                "width": 640,
                "height": 480,
                "fps": 30,
            }
        }
    ),
):
    if s.state == "FAILED":
        break
```

### C) Suggested socket message payload from SDK extra to service

```json
{
  "event": "observation.push",
  "session_id": "sess_123",
  "task": "pick and place",
  "observation": {
    "joint_1.pos": -1.2,
    "joint_2.pos": -0.8,
    "joint_3.pos": 1.1,
    "gripper.pos": 0.0
  },
  "policy": {
    "kind": "act",
    "path": "my-org/act-policy",
    "n_action_steps": 32
  },
  "cameras": {
    "flange": {
      "type": "webrtc",
      "api_url": "http://camera-server:8081",
      "device_id": "World_UR10e_flange",
      "width": 640,
      "height": 480,
      "fps": 30
    }
  }
}
```

---

## Policy discovery/selection (SDK pattern)

To align with existing NOVA SDK usage (e.g. controller discovery), SDK should support explicit policy discovery before execution.

For now, discovery is per service instance and returns exactly one policy (the configured `POLICY_PATH`). A `list_policies()` method can be added later when multi-policy routing is introduced.

Recommended flow:
1. `policy = await policy_service_client.get_policy()`
2. use the returned typed policy object directly (no dict->class conversion)
3. start/stream execution via `motion_group.execute_policy(...)`

Example:

```python
policy_client = nova_policy.PolicyServiceClient(base_url=policy_service_url)
policy = await policy_client.get_policy()
# returns exactly one typed policy instance, e.g. ACTPolicy(path="my-org/act-policy")
# raises if service has no configured policy or is unreachable

if not isinstance(policy, ACTPolicy):
    raise RuntimeError(f"Expected ACT policy, got {type(policy).__name__}")

# pass typed policy directly; override on the policy object, not via helper
run_policy = ACTPolicy(path=policy.path, n_action_steps=32)

await motion_group.execute_policy(
    policy=run_policy,
    task="pick and place",
    timeout_s=120,
    context=PolicyExecutionContext(
        cameras={
            "flange": {
                "type": "webrtc",
                "api_url": "http://camera-server:8081",
                "device_id": "World_UR10e_flange",
                "width": 640,
                "height": 480,
                "fps": 30,
            }
        }
    ),
)
```

Service-side this maps to a `get policy` capability. SDK client deserializes the response into a typed policy class (`ACTPolicy`, later `PI05Policy`, `GrootPolicy`) before returning it.

## Neuracore-inspired client interface sketch (for `nova_policy`)

Use one stable client-side policy API independent of transport/backend (direct/local/remote equivalent in our case is mostly remote socket for now).

```python
from dataclasses import dataclass
from typing import Protocol

@dataclass
class RobotStatePoint:
    timestamp: float
    joints: dict[str, float]
    gripper: dict[str, float] | None = None
    tcp: dict[str, float] | None = None
    io: dict[str, bool | float] | None = None

@dataclass
class ActionStep:
    joints: dict[str, float]
    gripper: dict[str, float] | None = None
    io: dict[str, bool | float] | None = None

@dataclass
class ActionChunk:
    chunk_id: str
    steps: list[ActionStep]
    inference_latency_ms: float
    control_dt_s: float  # e.g. 1 / fps from policy dataset metadata
    first_step_at_s: float | None = None  # optional absolute schedule anchor

class RuntimePolicy(Protocol):
    async def predict(self, state_point: RobotStatePoint, timeout_s: float = 5.0) -> ActionChunk: ...
    async def close(self) -> None: ...

# typed policy spec selected/discovered from policy service
@dataclass
class ACTPolicy:
    path: str
```

Suggested SDK flow:
1. Discover typed policy from policy service (`get_policy()` -> `ACTPolicy`).
2. Establish realtime session (via HTTP lifecycle call or `session.start` socket event).
3. Loop:
   - receive robot state from state socket
   - call policy `predict(state_point)` (internally sends `observation.push`, awaits `action.chunk`)
   - execute returned chunk locally via jogging-style path:
     - apply `steps[0]` immediately (or at `first_step_at_s` if provided)
     - apply subsequent steps at fixed cadence `control_dt_s`
     - each step can include joint + gripper + io commands
4. Stop/close session.

This keeps `motion_group.execute_policy()` ergonomics stable while hiding socket/event details behind a typed client abstraction.

### How Neuracore learnings are applied

1. **Typed policy client abstraction, one public API, pluggable backend**
   - Captured via `RuntimePolicy` protocol and typed policy classes.
   - Discovery via `get_policy()` (single policy per instance; `list_policies()` deferred to multi-policy phase).
   - Planned concrete backends:
     - `SocketRuntimePolicy` (primary)
     - optional `InProcessRuntimePolicy` (dev/test)
     - optional `HttpRuntimePolicy` compatibility shim (temporary, if needed)

2. **Typed observation container**
   - Captured via `RobotStatePoint`.
   - Socket observation messages should serialize exactly this structure.

3. **Discovery + bind endpoint before run**
   - Captured via `PolicyServiceClient.get_policy()` -> typed policy -> session lifecycle bind (HTTP or socket event).
   - Enforced in examples and phase DoD.

4. **Chunk execution control in SDK layer**
   - Captured via SDK execution loop applying `ActionChunk` with `control_dt_s` cadence.
   - Service only infers; SDK executes joints/gripper/io.

## Execution flow (desired)

1. SDK extra starts session.
2. Inference app enters `PREPARING` -> loads/warms policy.
3. SDK extra loop:
   - read current observation state (joints, optionally tcp/gripper/io)
   - push `observation.push` on socket
   - receive `action.chunk` (joint + gripper + io step commands + timing)
   - execute actions locally via jogging-like API using `control_dt_s` cadence
4. SDK updates/reads run status until stop/timeout/failure.

---

## Migration plan

## Phase 1 — Rebase service internals on `lerobot-inference` (without NOVA app wrapper)

1. Replace current `nova_policy/policy-service` runtime internals with code derived from:
   - `lerobot-inference/nova_app/app/runtime.py`
   - `lerobot-inference/nova_app/app/runner.py`
2. Keep existing NOVA policy-service run-state semantics, but expose over socket events.
3. Keep env-driven policy configuration (`POLICY_PATH`, preload flags).
4. Do not adopt NOVA program wrapper as integration surface; SDK extra remains caller.
5. Keep packaging/deployment ready for both `nova install` and Flux cluster rollout.
6. Validate route behavior for realtime transport via Flux gateway:
   - websocket upgrade/Socket.IO handshake
   - idle timeout behavior for long-lived sessions
7. Validate auth expectations for cross-environment calls (NOVA instance -> GPU cluster route) and align with infra-managed gateway policies.

Deliverable: service uses colleague inference core patterns, is consumed by `wandelbots-nova` SDK extra, deployable in both targets, and reachable from NOVA-instance network path with verified gateway/auth behavior.

## Phase 2 — Add bidirectional observation/action API

1. Add hybrid transport surface:
   - HTTP control-plane operations (`healthz`, `get_policy`, optional session lifecycle calls)
   - Socket.IO channels for `session.state`, `observation.push`, and `action.chunk`.
2. Add authentication hooks for both transports:
   - HTTP bearer auth handling
   - Socket.IO handshake auth handling
3. Factor model execution into an inference engine class:
   - `load_policy()`
   - `predict_chunk(observation)`
4. Remove robot execution from service loop.

Deliverable: service can infer from external observations and produce chunks only.

## Phase 3 — SDK extra executes robot movement

1. Extend `nova_policy` client to call HTTP control-plane operations and Socket.IO data-plane channels.
2. Implement SDK-side executor loop:
   - read state + cameras
   - send observations
   - consume chunks
   - command robot with jogging-like behavior
3. Reuse concepts from `lerobot_robot_nova`:
   - chunk queue
   - timing/fps pacing
   - e-stop/restart-safe behavior

Deliverable: robot control moved to SDK extra, inference app becomes pure inference service.

## Phase 4 — WebRTC camera integration and policy adapters

1. Camera source stays in inference app via WebRTC plugins (no image transport over SDK<->inference API).
2. SDK sends only robot state observations (joints/gripper/tcp/io).
3. Implement per-policy-type adapters and validate schema compatibility.

**Known trade-off (camera/state temporal alignment):** Because camera frames are ingested independently by the inference service (via WebRTC) and robot state observations arrive separately from the SDK, there is an inherent risk of temporal misalignment between image and joint-state observations. For v1 we accept this trade-off — the inference service uses nearest-available camera frame when a robot-state observation arrives. This is acceptable for policies trained at moderate control frequencies (≤30 Hz) but should be revisited if tighter synchronization is needed (e.g., high-speed manipulation or policies sensitive to observation co-temporality).

Deliverable: production WebRTC camera flow + robust multi-policy adapter layer.

---

## Detailed TODO checklist

## Service-side (actual phase-1 base now lives in `/Users/stefanwagner/Git/lerobot-inference/policy_service`)
- [x] Replace current runtime engine with `lerobot-inference`-style policy loader/inferencer.
- [x] Keep env vars as source of policy (`POLICY_PATH`, preload options).
- [x] Add transport dependency wiring (`python-socketio`) in service package/runtime image.
- [x] Add hybrid transport surface:
  - [x] Keep HTTP control-plane endpoints (`healthz`, `get_policy`, optional session lifecycle calls).
  - [x] Add Socket.IO channels for `session.state`, `observation.push`, and `action.chunk`.
- [x] Remove direct `robot.send_action` logic from service.
- [x] Integrate Socket.IO-based robot-state relay path from SDK extra in an opt-in MotionGroup realtime loop.
- [x] Add Flux gateway timeout policy for long-lived realtime sessions:
  - [x] add `BackendTrafficPolicy` for `nova-policy-service` route (request + connection idle timeout).
  - [x] verify websocket upgrade/Socket.IO handshake is stable through `envoy-shared` from the public cluster route.
- [ ] Confirm gateway auth behavior with infra-owned policies (outside this repo) and document required auth headers/tokens for this service.
- [ ] Add feature schema validation for inbound robot-state observations.
- [x] Keep camera ingestion fully internal via WebRTC plugin configuration.
- [ ] Implement production policy-type adapter registry (input/output normalization per policy family).
  - [x] Minimal SDK-side `ACTAdapter` exists for ACT start-payload normalization.
- [x] Preserve existing state machine + metadata contract.

## SDK extra (`wandelbots-nova/nova_policy`)
- [ ] Add HTTP control-plane client methods (`get_policy`, lifecycle) and Socket.IO observation/action methods.
  - [x] `get_policy()` typed discovery is implemented.
  - [x] existing HTTP run lifecycle methods are available (`start_run`, `stop_run`, `get_run`, `stream_run`).
  - [x] basic Socket.IO observation/action methods are implemented via `PolicyRealtimeSession.predict(...)`.
- [x] Add SDK transport dependency wiring (`python-socketio` AsyncClient).
- [ ] Support auth on both transports:
  - [x] send `Authorization` header on HTTP control-plane calls
  - [x] send auth headers during Socket.IO handshake using the same client header source
  - [ ] define token source (reuse NOVA access token vs dedicated policy-service token)
- [x] Add opt-in realtime executor loop in `motion_group_extensions.py` for observation/chunk exchange.
- [ ] Implement queue-based action consumption (inherit `lerobot-inference` queue pattern):
  - [x] Enqueue all steps from received `action.chunk`.
  - [ ] Consume one step per `control_dt_s` tick.
  - [x] Trigger next `observation.push` when queue drains or falls below low-water mark.
  - [x] Hold last commanded position if queue drains before next chunk arrives.
- [ ] Implement production local action application path (jogging-style semantics).
  - [x] Initial guarded one-shot NOVA Jogging API action sender exists behind `execute_actions=True`.
- [ ] Configure Socket.IO reconnect/heartbeat behavior (bounded retries/backoff + timeout handling).
- [ ] Enforce app-level stream semantics across reconnects:
  - [x] monotonic observation `seq`
  - [x] `action.chunk` correlation via `observation_seq`
  - [x] max in-flight observations (default 1 through request/response sequencing)
  - [ ] duplicate suppression / safe replay policy
- [x] Support stop/cancel and conflict handling with existing `PolicyRunState.stop()`.
- [x] Surface action/joint telemetry in stream metadata.
- [x] Allow optional `n_action_steps` override on discovered `ACTPolicy` before execution.

## Cross-repo alignment (`wandelbots-lerobot`)
- [ ] Reuse/port safe execution primitives from `lerobot_robot_nova` (jogging lifecycle, e-stop recovery patterns).
- [ ] Define minimal shared action format (`steps[].joints`, optional `steps[].gripper`, optional `steps[].io`, units, joint naming/order contract).
- [ ] Validate motion limits/collision integration path via NOVA setup data.

---

## Compatibility notes vs old plan

From `docs/nova-lerobot-policy-extension-plan.md`, these stay valid:
- MotionGroup-native SDK UX (`execute_policy`, `stream_policy`).
- Keep run states and async lifecycle behavior.
- Keep optional package boundary (extra, no core SDK invasive changes).

What changes now:
- Runtime base shifts from custom eval wrapper to colleague `lerobot-inference` app.
- Robot motion execution shifts from service to SDK extra.
- Need explicit bidirectional observation/action contract.

---

## Open decisions

1. Socket protocol details (transport choice fixed to Socket.IO over websocket):
   - Event schema, ordering, backpressure, reconnect/resume behavior.
   - Exact ack strategy (per-message vs batched ack).
2. Cross-environment gateway/auth model (NOVA instance -> GPU cluster route):
   - Is route auth enforced by shared infra policies (outside this repo)?
   - Which token is required for policy service calls (reuse NOVA access token vs dedicated token)?
   - How are auth headers forwarded/validated for Socket.IO handshake requests?
3. Gateway timeout policy for long-lived realtime sessions:
   - Is default `envoy-shared` idle timeout sufficient?
   - Do we require app-local `BackendTrafficPolicy` for this route?
4. Policy adapter strategy:
   - define adapter interface and required policy-family coverage for v1.
   - explicitly validate ACT inference override behavior (`n_action_steps`).
   - use typed policy union (`ACTPolicy | PI05Policy | GrootPolicy`) instead of separate `policy_type`.
5. Action execution API in SDK extra:
   - reuse internal jogging directly vs MotionGroup public API shim.
6. Session ownership/scoping:
   - per motion group, per controller, or per app globally.
7. Multi-policy future architecture:
   - introduce orchestration/marketplace layer to route to one of many single-policy inference deployments.

---

## Handoff package for next agent

## Relevant files to read first

### Planning/context docs
- `docs/nova-policy-bidirectional-inference-adaptation-plan.md` (this file)
- `docs/nova-lerobot-policy-extension-plan.md` (older plan; still useful for SDK UX and state model)

### Current SDK extra code (wandelbots-nova)
- `nova_policy/client.py`
- `nova_policy/models.py`
- `nova_policy/motion_group_extensions.py`
- `nova_policy/policy-service/policy_service/app_server.py`
- `nova_policy/policy-service/policy_service/runtime.py`
- `nova_policy/policy-service/policy_service/lerobot_inference_engine.py`
- `nova_policy/policy-service/policy_service/observation_sources.py`
- `nova_policy/policy-service/README.md`
- `pyproject.toml` (extras + packaging)

### Colleague inference base / actual phase-1 implementation (`lerobot-inference`)
- `/Users/stefanwagner/Git/lerobot-inference/policy_service/app.py`
- `/Users/stefanwagner/Git/lerobot-inference/policy_service/runtime.py`
- `/Users/stefanwagner/Git/lerobot-inference/policy_service/models.py`
- `/Users/stefanwagner/Git/lerobot-inference/nova_app/app/runtime.py`
- `/Users/stefanwagner/Git/lerobot-inference/nova_app/app/runner.py`
- `/Users/stefanwagner/Git/lerobot-inference/nova_app/app/config.py`
- `/Users/stefanwagner/Git/lerobot-inference/nova_app/app/api.py`
- `/Users/stefanwagner/Git/lerobot-inference/inference.py`
- `/Users/stefanwagner/Git/lerobot-inference/README.md`

### Robot execution reference (wandelbots-lerobot)
- `/Users/stefanwagner/Git/wandelbots-lerobot/lerobot_robot_nova/lerobot_robot_nova/nova.py`
- `/Users/stefanwagner/Git/wandelbots-lerobot/lerobot_robot_nova/lerobot_robot_nova/nova_client.py`
- `/Users/stefanwagner/Git/wandelbots-lerobot/lerobot_robot_nova/lerobot_robot_nova/jogging_controller.py`

### Flux deployment manifests
- `/Users/stefanwagner/Git/flux-apps/apps/nova-policy-service/deployment.yaml`
- `/Users/stefanwagner/Git/flux-apps/apps/nova-policy-service/httproute.yaml`
- `/Users/stefanwagner/Git/flux-apps/apps/nova-policy-service/kustomization.yaml`
- `/Users/stefanwagner/Git/flux-apps/gpucluster-dev/flux-apps.yaml`
- `/Users/stefanwagner/Git/flux-apps/apps/gaussian-splat/backend-traffic-policy.yaml` (reference for timeout policy pattern)
- `/Users/stefanwagner/Git/flux-apps/AGENTS.md` (notes that auth/gateway kustomizations live in infra repo)

---

## Execution plan in phases (with Definition of Done)

### Phase 1 — Adapt inference app deployment to Flux cluster

Goal: deploy colleague-based inference service shape to GPU cluster.

Tasks:
1. Rebase service internals toward `lerobot-inference` runtime/loading patterns.
2. Keep env-driven policy resolution (`POLICY_PATH`, preload flags).
3. Build/push container image to ACR.
4. Update Flux image tag and reconcile.
5. Verify pod readiness + health endpoint + logs.
6. Validate cross-environment connectivity from a NOVA-instance-like network:
   - HTTP control-plane (`/healthz`, `get_policy`)
   - Socket.IO connect + websocket upgrade
7. Validate auth requirements with infra-managed gateway policies and confirm expected token/header behavior.
8. If needed, add `BackendTrafficPolicy` for route idle/request timeouts to support long-lived realtime sessions.

Definition of Done:
- [x] Deployment uses new image tag in Flux and rollout is `Ready`.
- [x] `GET /healthz` (or equivalent health signal) returns success from cluster route.
- [x] Service logs show policy preload/load stage without crash loops.
- [x] Policy discovery endpoint (`get_policy`) reports the configured policy for the instance.
- [x] HTTP control-plane calls succeed through the external cluster route.
- [ ] Socket.IO handshake + websocket upgrade succeeds through gateway from NOVA-instance network path.
- [x] Auth behavior is verified/documented (accepted token path + expected unauthorized behavior) for current HTTP control-plane endpoints.
- [x] Deployment notes captured (image tag, commit SHA, endpoint URL).

Local pre-cluster progress already validated:
- [x] Local `policy_service` process starts and serves `GET /healthz` + `GET /policy`.
- [x] Real policy loading works in the adapted `lerobot-inference` repo after importing `lerobot.policies`.
- [x] Manual run against instance `spjhrikg.instance.wandelbots.io` reaches the virtual `ur10e`, connects dataset-backed cameras, and starts jogging control.
- [x] Run-state progression is observable over HTTP control-plane (`PREPARING -> TIMED_OUT` in the short validation run).

### Phase 2 — Manual inference run on cluster with mock robot-state input; verify joint output

Goal: prove inference works on cluster before SDK loop integration.

Tasks:
1. Start a session manually (HTTP lifecycle call or socket event).
2. Connect via Socket.IO data channel using the same auth mechanism intended for SDK clients.
3. Send robot-state observation messages (mock joints/gripper; no images).
4. Receive action chunks from Socket.IO data channel.
5. Print/log returned joint vectors continuously.
6. Validate reconnect behavior once (disconnect + resume) without crashing the run.

Definition of Done:
- [x] Session transitions `PREPARING -> RUNNING` successfully.
- [x] At least N>=20 action responses received in one run.
- [x] Returned joint vectors are printed/logged and vary over time (non-constant output).
- [x] Run ends cleanly via stop or timeout with terminal state recorded.
- [x] One forced disconnect/reconnect test succeeds without process crash (recover or fail-safe stop).
- [ ] Manual test path uses authenticated HTTP + Socket.IO calls that match planned SDK auth wiring.
  - Current cluster route accepted unauthenticated HTTP and Socket.IO calls; token source/header policy remains to be defined with infra.

### Phase 3 — Adapt SDK extra with explicit ACT policy support (pattern-based)

Goal: implement software-patterned ACT adapter path in SDK extra + service contract.

Tasks:
1. Introduce policy adapter abstraction (interface + registry).
2. Implement `ACTAdapter` explicitly (v1 only).
3. Add SDK-side typed ACT inference option:
   - `n_action_steps`
4. Keep explicit typed ACT policy class in SDK API (`ACTPolicy`) and use it directly from discovery.
5. Wire Socket.IO client in SDK extra for:
   - session control events,
   - observation push,
   - action chunk receive,
   - heartbeat/reconnect handling.
6. Implement auth propagation in SDK client:
   - include bearer token on HTTP control-plane requests
   - include same token (or configured dedicated token) in Socket.IO handshake
7. Ensure WebRTC stream descriptors are passed in SDK payload (`cameras.*`) and consumed by service runtime.
8. Use `get_policy()` for discovery (single policy per instance).

Definition of Done:
- [x] `ACTAdapter` is used for SDK ACT start-payload normalization.
- [x] SDK `stream_policy(..., options=PolicyExecutionOptions(realtime=True))` can run ACT observation/chunk exchange via Socket.IO channels.
- [x] ACT inference knobs are accepted, validated, and reflected in service runtime config.
- [x] Unit tests cover adapter mapping + payload conversion + bounded realtime robot-state push flow.
- [ ] SDK auth propagation works for both HTTP + Socket.IO paths against cluster route.
- [x] No direct robot motion is performed by inference service.

### Phase 4 — SDK-side local execution loop (jogging-style) and end-to-end validation

Goal: close the loop with SDK executing robot movement from returned action chunks.

Tasks:
1. Implement SDK execution loop consuming `action.chunk` messages using inherited queue-based pattern:
   - enqueue chunk steps, consume at `control_dt_s`, re-query on drain, hold position on underflow.
2. Apply actions using jogging-style semantics aligned with `lerobot_robot_nova` patterns.
3. Handle reconnect/stop/e-stop recovery paths.
4. Validate telemetry and metadata surfacing.

Definition of Done:
- [ ] End-to-end loop works: state socket -> inference -> action chunk -> SDK local execution.
- [ ] Stop/cancel behavior works and no orphan loops remain.
- [ ] E-stop or comm interruption is handled without process crash.
- [ ] Metadata includes latest action/joint telemetry and run state progression.

---

## Recommended immediate next step

Phase 3 is partially implemented locally: ACT payload normalization and opt-in SDK observation/chunk exchange now exist and are unit-tested. Next, continue with the remaining Phase 4 execution work:

1. harden SDK-side jogging action application for `PolicyExecutionOptions(realtime=True, execute_actions=True)`,
2. validate the continuous velocity-control lifecycle against safe primitives from `lerobot_robot_nova`,
3. consume one queued `action.chunk` step per `control_dt_s` tick against the virtual UR10e,
4. tune velocity limits/gains and add safety limit checks from NOVA setup data,
5. validate `execute_actions=True` against the virtual UR10e with a short timeout and stop/cancel checks,
6. then resolve the auth-token source for protected HTTP + Socket.IO gateway operation.
