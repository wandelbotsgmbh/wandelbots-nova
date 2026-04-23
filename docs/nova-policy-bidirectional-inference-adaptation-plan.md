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

## Service-side (`wandelbots-nova/nova_policy/policy-service`)
- [ ] Replace current runtime engine with `lerobot-inference`-style policy loader/inferencer.
- [ ] Keep env vars as source of policy (`POLICY_PATH`, preload options).
- [ ] Add transport dependency wiring (`python-socketio`) in service package/runtime image.
- [ ] Add hybrid transport surface:
  - [ ] Keep HTTP control-plane endpoints (`healthz`, `get_policy`, optional session lifecycle calls).
  - [ ] Add Socket.IO channels for `session.state`, `observation.push`, and `action.chunk`.
- [ ] Integrate Socket.IO-based robot-state relay path from SDK extra.
- [ ] Add Flux gateway timeout policy for long-lived realtime sessions:
  - [ ] add `BackendTrafficPolicy` for `nova-policy-service` route (request + connection idle timeout).
  - [ ] verify websocket upgrade/Socket.IO handshake is stable through `envoy-shared`.
- [ ] Confirm gateway auth behavior with infra-owned policies (outside this repo) and document required auth headers/tokens for this service.
- [ ] Remove direct `robot.send_action` logic from service.
- [ ] Add feature schema validation for inbound robot-state observations.
- [ ] Keep camera ingestion fully internal via WebRTC plugin configuration.
- [ ] Implement policy-type adapter registry (input/output normalization per policy family).
- [ ] Preserve existing state machine + metadata contract.

## SDK extra (`wandelbots-nova/nova_policy`)
- [ ] Add HTTP control-plane client methods (`get_policy`, lifecycle) and Socket.IO observation/action methods.
- [ ] Add SDK transport dependency wiring (`python-socketio` AsyncClient).
- [ ] Support auth on both transports:
  - [ ] send `Authorization` header on HTTP control-plane calls
  - [ ] send auth during Socket.IO handshake (headers/auth payload)
  - [ ] define token source (reuse NOVA access token vs dedicated policy-service token)
- [ ] Add executor loop in `motion_group_extensions.py`.
- [ ] Implement queue-based action consumption (inherit `lerobot-inference` queue pattern):
  - [ ] Enqueue all steps from received `action.chunk`.
  - [ ] Consume one step per `control_dt_s` tick.
  - [ ] Trigger next `observation.push` when queue drains or falls below low-water mark.
  - [ ] Hold last commanded position if queue drains before next chunk arrives.
- [ ] Implement local action application path (jogging-style semantics).
- [ ] Configure Socket.IO reconnect/heartbeat behavior (bounded retries/backoff + timeout handling).
- [ ] Enforce app-level stream semantics across reconnects:
  - [ ] monotonic observation `seq`
  - [ ] `action.chunk` correlation via `observation_seq`
  - [ ] max in-flight observations (default 1)
  - [ ] duplicate suppression / safe replay policy
- [ ] Support stop/cancel and conflict handling with existing `PolicyRunState.stop()`.
- [ ] Surface action/joint telemetry in stream metadata.
- [ ] Allow optional `n_action_steps` override on discovered `ACTPolicy` before execution.

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

### Colleague inference base (lerobot-inference)
- `/Users/stefanwagner/Git/lerobot-inference/nova_app/app/runtime.py`
- `/Users/stefanwagner/Git/lerobot-inference/nova_app/app/runner.py`
- `/Users/stefanwagner/Git/lerobot-inference/nova_app/app/config.py`
- `/Users/stefanwagner/Git/lerobot-inference/nova_app/app/api.py`
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
- [ ] Deployment uses new image tag in Flux and rollout is `Ready`.
- [ ] `GET /healthz` (or equivalent health signal) returns success from cluster route.
- [ ] Service logs show policy preload/load stage without crash loops.
- [ ] Policy discovery endpoint (`get_policy`) reports the configured policy for the instance.
- [ ] HTTP control-plane calls succeed from NOVA-instance network path (not only from inside cluster).
- [ ] Socket.IO handshake + websocket upgrade succeeds through gateway from NOVA-instance network path.
- [ ] Auth behavior is verified/documented (accepted token path + expected unauthorized behavior).
- [ ] Deployment notes captured (image tag, commit SHA, endpoint URL).

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
- [ ] Session transitions `PREPARING -> RUNNING` successfully.
- [ ] At least N>=20 action responses received in one run.
- [ ] Returned joint vectors are printed/logged and vary over time (non-constant output).
- [ ] Run ends cleanly via stop or timeout with terminal state recorded.
- [ ] One forced disconnect/reconnect test succeeds without process crash (recover or fail-safe stop).
- [ ] Manual test path uses authenticated HTTP + Socket.IO calls that match planned SDK auth wiring.

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
- [ ] `ACTAdapter` is used end-to-end for policy type `act`.
- [ ] SDK `execute_policy`/`stream_policy` can run ACT via Socket.IO channels.
- [ ] ACT inference knobs are accepted, validated, and reflected in service runtime config.
- [ ] Unit/integration tests cover adapter mapping + payload conversion + stop flow.
- [ ] SDK auth propagation works for both HTTP + Socket.IO paths against cluster route.
- [ ] No direct robot motion is performed by inference service.

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

Start with **Phase 1** (cluster deployment adaptation), explicitly including gateway/auth/connectivity validation from NOVA-instance network path, then execute **Phase 2** manual validation before touching SDK adapter work.
