# NOVA × LeRobot Policy Execution Extension Plan

## Goal

Provide a native-feeling policy execution extension for `wandelbots-nova` so users can:
1. Keep writing normal NOVA Python programs.
2. Optionally install an extra (e.g. `nova-lerobot-policy`).
3. Execute/stop policy inference from SDK code in a way that matches existing motion APIs.
4. Mix classical SDK motion steps and policy execution in one flow (e.g., move to home with `plan/execute`, then run policy).
## Background and problem statement

Today, policy inference is typically triggered through LeRobot CLI workflows (e.g. `lerobot-record ... --policy.path=...`), which are strongly oriented around dataset recording/evaluation flows.

For SDK users, the desired experience is a **single integrated workflow**:
- keep using normal `nova` program code (`@nova.program`, `motion_group` operations, IO),
- add policy execution as one additional capability in the same program,
- and freely combine deterministic motion steps and policy-driven control,
- without adopting recording-specific CLI orchestration as the main integration pattern.
This document defines a standalone architecture and API plan for that integration across:
- `wandelbots-nova` (SDK-side extension), and
- `wandelbots-lerobot` (policy runtime/service side).

---

## Product direction

## 1) Deploy as a NOVA app first

Primary target: run policy execution as a **NOVA app** (installable on NOVA).

- Near term: can also run locally in Docker for development.
- Mid/long term: run as managed service on cluster/Kubernetes via NOVA app deployment.
- UI/user setup can move into the NOVA app (policy selection, auth, defaults, guardrails).

### Responsibilities of the NOVA Policy App

The NOVA Policy App is the operational control plane for policy loading/execution. It should own:

- **Policy lifecycle management**
  - Keep exactly one policy loaded at a time (MVP constraint).
  - States at app/runtime level: `EMPTY`, `LOADING`, `READY`, `RUNNING`, `SWITCHING`, `ERROR`.
  - Handle policy switching when a different policy is requested.

- **Policy preparation pipeline**
  - Download (public source in MVP), load, and warm up policy.
  - Expose progress stages (`downloading`, `loading`, `warming`) via API/UI.
  - Guarantee `start/execute` works even when policy is not preloaded.

- **Execution orchestration**
  - Start/stop runs and enforce single active run policy per configured scope.
  - Maintain run states (`PREPARING`, `RUNNING`, `STOPPING`, `STOPPED`, `TIMED_OUT`, `FAILED`).
  - Provide run status and progress telemetry.

- **Auth and policy source abstraction (future)**
  - Implement device-login based auth for private policy access later.
  - Decouple application code from raw HF tokens.
  - Allow future non-HF backend (Wandelbots-hosted policy registry/storage).

- **Safety and constraints integration**
  - Accept NOVA-native `motion_group_setup` payloads.
  - Reuse existing NOVA collision/limits handling where already available in runtime path.
  - Keep additional stop-condition checks possible in application callbacks.

- **Configuration and UX**
  - Provide UI/config for default policy, preloading behavior, target robot selection, and camera setup.
  - Support optional preloading to keep policies in `READY` state for low-latency start.

---

## 2) Repository responsibilities

### A) `wandelbots-lerobot`
Build the policy runtime service (FastAPI + runtime manager), reusing existing plugin building blocks:
- Reuse and integrate with existing `lerobot_robot_nova` plugin (already present).
- Add policy runtime package (e.g. `lerobot_policy_nova_service`).
- Handle policy loading, execution loop, stop handling, and runtime telemetry.

### B) `wandelbots-nova`
Add optional SDK extension package (e.g. `nova_lerobot_policy`) plus MotionGroup-facing ergonomics.

- Keep optional install via extra.
- Provide API surface aligned with `motion_group.plan(...)`, `execute(...)`, `stream_execute(...)`.

---

## Constraints / limitations

- **Policy source (MVP):** for now, assume policy download works for **public repositories only**.
- **Private policy access (later):** authentication should be handled via **device login flow** in the NOVA app (not raw token passing from SDK calls). This also keeps a future path open for hosting policies in a Wandelbots-managed registry/storage instead of Hugging Face.
- Current policy lifecycle is timeout-driven execution; no semantic “task finished by policy” state yet.

---

## API style alignment with SDK programs

The service API should follow the existing experimental programs contract in `docs/programs_spec.md`.

Observed pattern in `programs_spec.md`:
- Resource scoping by cell: `/experimental/cells/{cell}/...`
- Start endpoint: `POST .../start` returning a run object
- Stop endpoint: `POST .../stop` returning `204`
- Asynchronous execution model
- Conflict when already running (`406`)

### Key design principle
Prefer MotionGroup-native SDK methods:
- `motion_group.execute_policy(...)` → execute until timeout or stop condition.
- `motion_group.stream_policy(...)` → async stream with ability to stop/cancel from program logic.

This is preferred over a standalone `PolicyExecutor`-only style.

---

## Service API contract (aligned to programs_spec)

Base URL example (dev): `http://localhost:8081`

Preferred shape (program-like):
- `GET /experimental/cells/{cell}/policies`
- `GET /experimental/cells/{cell}/policies/{policy}`
- `POST /experimental/cells/{cell}/policies/{policy}/start`
- `POST /experimental/cells/{cell}/policies/{policy}/stop`
- `GET /healthz`

Behavior alignment with `programs_spec.md`:
- `start` returns a run resource (`run`, `policy`, `state`, `start_time`).
- `stop` returns `204` (also if already stopped).
- `406` when run already active for same target scope.
- Execution is asynchronous.
- `start` must always work standalone: if policy is not cached/warm yet, it performs download+load+warmup before entering `RUNNING`.
- Preloading is optional (e.g. via NOVA app UI/background task) to reduce startup latency.
- No explicit `warm_start` flag is required in the API; warm handling is runtime responsibility.

### Request draft (`POST /experimental/cells/{cell}/policies/{policy}/start`)

```json
{
  "policy": {
    "path": "StefanWagnerWandelbots/act_virtual_teleop_pickplace_30fps_2",
    "n_action_steps": 10,
    "device": "cuda"
  },
  "target": {
    "controller_name": "ur10e",
    "motion_group": 0,
    "tcp": "flange"
  },
  "task": "pick the cube and place it in the box",
  "timeout_s": 120.0,
  "cameras": {
    "flange": {
      "type": "webrtc",
      "api_url": "http://camera-server",
      "device_id": "cam1",
      "width": 640,
      "height": 480,
      "fps": 30
    }
  },
  "gripper": {
    "use_gripper": true,
    "gripper_io_key": "digital_out[0]"
  },
  "motion_group_setup": {
    "motion_group_model": "UniversalRobots_UR10e",
    "cycle_time": 8,
    "global_limits": {
      "joints": []
    },
    "collision_setups": {
      "policy-guard": {
        "colliders": {},
        "link_chain": [],
        "tool": {},
        "self_collision_detection": true
      }
    }
  }
}
```

Notes:
- Collision data should be passed in the same shape used by NOVA SDK/OpenAPI: `motion_group_setup.collision_setups` (dictionary of named `CollisionSetup`s), not a custom ad-hoc bounds schema.
- In SDK calls, this should usually come from `await motion_group.get_setup(tcp)` and then optional user modifications to `motion_group_setup.collision_setups`.
- MVP supports a single target motion group (`target`).
- Future extension should support multiple targets (e.g. `targets: []`) for multi-robot policy execution.

`motion_group_setup` usage clarity (important):
- **Consumed by NOVA execution/guardrail layer:**
  - `collision_setups` (environment + self-collision checks)
  - `global_limits` (joint/TCP limits)
  - `tcp_offset`, `mounting`, `payload`, `cycle_time` (execution/planning consistency)
- **Not consumed by the ML policy network directly:** the policy primarily consumes observations (joint state, cameras, optional task text) and outputs actions.
- Therefore, `motion_group_setup` is passed to enforce robot-side safety/constraints during policy execution, not as policy model input features.

### Start response draft (program-run-like)

```json
{
  "run": "run_123",
  "policy": "app1.pick_place_policy",
  "state": "RUNNING",
  "start_time": "2026-04-20T11:00:00Z",
  "timeout_s": 120.0
}
```

### Optional status/progress payload (non-blocking extension)

```json
{
  "run": "run_123",
  "state": "PREPARING",
  "progress": {
    "stage": "downloading_policy",
    "percent": 35,
    "message": "Downloading model artifacts"
  }
}
```

```json
{
  "run": "run_123",
  "state": "PREPARING",
  "progress": {
    "stage": "warming_policy",
    "percent": 90,
    "message": "Running first inference warmup"
  }
}
```

```json
{
  "run": "run_123",
  "state": "RUNNING",
  "progress": {
    "control_fps": 29.8,
    "inference_latency_ms_p50": 22.4
  }
}
```

### State model (updated)

Because completion semantics are not yet policy-defined:
- `PREPARING`
- `RUNNING`
- `STOPPING`
- `STOPPED`
- `TIMED_OUT`
- `FAILED`

(Do **not** rely on a semantic `FINISHED/COMPLETED` state yet.)

---

## Guardrails and external control

MVP guidance:
- Reuse existing NOVA-side collision handling **only if already available through `motion_group_setup` in the runtime path**.
- Do not block MVP on adding new collision features inside the LeRobot plugin.

Recommended first step:
- Keep policy plugin focused on policy execution + stop.
- Perform additional guard checks in application code (e.g. callback with `stream_policy`) and stop on violation.

Optional/next step:
- Tighten runtime guardrails by forwarding full `motion_group_setup.collision_setups` into the execution backend when available.

---

## SDK packaging (`wandelbots-nova`)

Add optional dependency in `pyproject.toml`:

```toml
[project.optional-dependencies]
nova-lerobot-policy = [
  "httpx>=0.28,<0.29"
]
```

Install commands:

### With `uv`
```bash
uv add wandelbots-nova --extra nova-lerobot-policy
```

### With `pip`
```bash
pip install "wandelbots-nova[nova-lerobot-policy]"
```

Optional with rerun:

```bash
uv add wandelbots-nova --extra nova-lerobot-policy --extra nova-rerun-bridge
# or
pip install "wandelbots-nova[nova-lerobot-policy,nova-rerun-bridge]"
```

---

## Docker/runtime expectations

## Where it runs
- Dev mode: local Docker container.
- Target mode: NOVA app deployment (cluster/Kubernetes managed by NOVA).

## Image requirements
- Keep image very small and fast startup.
- Avoid large base images.
- Use multi-stage build + slim runtime base (e.g. `python:3.11-slim` or distroless Python runtime pattern where feasible).
- Avoid Alpine for this workload unless fully validated (Torch/native deps).

Example local run:

```bash
docker run --rm -it \
  -p 8081:8081 \
  -e NOVA_API=http://nova-instance \
  -e NOVA_ACCESS_TOKEN=$NOVA_ACCESS_TOKEN \
  -v lerobot_policy_cache:/cache \
  wandelbots/lerobot-policy-nova:latest
```

Note: HF token is intentionally omitted for now due to public-model-only limitation.

---

## Concrete Python SDK usage examples (MotionGroup-aligned)

> Target UX; names may be refined during implementation.

## 1) Execute policy until timeout

```python
import nova
from nova import run_program


@nova.program(name="Execute Policy")
async def execute_policy_program(ctx: nova.ProgramContext):
    n = ctx.nova
    cell = n.cell()
    controller = await cell.controller("ur10e")
    motion_group = controller[0]

    motion_group_setup = await motion_group.get_setup(tcp="flange")

    # execute_policy must ensure: download/load/warm (if needed) before RUNNING
    # and expose PREPARING progress (download + warmup stages).
    result = await motion_group.execute_policy(
        policy_path="StefanWagnerWandelbots/act_virtual_teleop_pickplace_30fps_2",
        task="pick the cube and place it in the box",
        timeout_s=120.0,
        n_action_steps=10,
        cameras={
            "flange": {
                "type": "webrtc",
                "api_url": "http://camera-server",
                "device_id": "cam1",
                "width": 640,
                "height": 480,
                "fps": 30,
            }
        },
        use_gripper=True,
        gripper_io_key="digital_out[0]",
        motion_group_setup=motion_group_setup,
    )

    print(result.state)  # e.g. timed_out / stopped / failed


if __name__ == "__main__":
    run_program(execute_policy_program)
```

## 2) Mix deterministic motion + policy execution (home first)

```python
@nova.program(name="Home then Policy")
async def home_then_policy(ctx: nova.ProgramContext):
    controller = await ctx.nova.cell().controller("ur10e")
    motion_group = controller[0]

    tcp = (await motion_group.tcp_names())[0]
    home = await motion_group.joints()

    # Deterministic SDK movement first
    traj = await motion_group.plan([nova.actions.joint_ptp(home)], tcp)
    await motion_group.execute(traj, tcp)

    # Then policy execution in the same workflow.
    # If not preloaded via app/UI, this call performs download+warmup and then starts execution.
    await motion_group.execute_policy(
        policy_path="StefanWagnerWandelbots/act_virtual_teleop_pickplace_30fps_2",
        task="pick the cube and place it in the box",
        timeout_s=120.0,
        n_action_steps=10,
    )
```

## 3) Stream policy and stop from program logic

```python
@nova.program(name="Stream Policy")
async def stream_policy_program(ctx: nova.ProgramContext):
    controller = await ctx.nova.cell().controller("ur10e")
    motion_group = controller[0]

    async for state in motion_group.stream_policy(
        policy_path="StefanWagnerWandelbots/act_virtual_teleop_pickplace_30fps_2",
        task="pick the cube and place it in the box",
        timeout_s=120.0,
        n_action_steps=10,
    ):
        # External stop condition example
        if state.elapsed_s > 20 and state.inference_latency_ms_p50 > 80:
            await state.stop()
            break
```

## 4) User-defined stop callback (condition checking in user code)

```python
@nova.program(name="Policy with callback stop")
async def policy_with_callback_stop(ctx: nova.ProgramContext):
    controller = await ctx.nova.cell().controller("ur10e")
    motion_group = controller[0]

    def should_stop(state) -> bool:
        # User-owned logic (pose/IO/custom guards)
        if state.elapsed_s > 45:
            return True
        if state.current_tcp_pose is not None and state.current_tcp_pose.position.z < 120:
            return True
        return False

    async for state in motion_group.stream_policy(
        policy_path="StefanWagnerWandelbots/act_virtual_teleop_pickplace_30fps_2",
        task="pick the cube and place it in the box",
        timeout_s=90.0,
    ):
        if should_stop(state):
            await state.stop()
            break
```

This keeps policy-plugin internals simple and delegates stop-condition evaluation to application code.

---

## Mapping from current `lerobot-record` invocation

- `--robot.nova_api` -> resolved from SDK/NOVA app config (prefer not user-passed each call)
- `--robot.controller_name` -> `target.controller_name`
- `--robot.use_gripper` -> `gripper.use_gripper`
- `--robot.gripper_io_key` -> `gripper.gripper_io_key`
- `--robot.cameras=...` -> `cameras`
- `--dataset.single_task=...` -> `task`
- `--policy.n_action_steps` -> `policy.n_action_steps`
- `--policy.path` -> `policy.path`

---

## Phase plan

1. **Mock-first vertical slice (start here)**
   - Build a NOVA app with UI + mock policy API on a NOVA instance.
   - Build SDK extra bridge (`execute_policy`, `stream_policy`) against the mock API.
   - Add tests proving end-to-end flow works.
2. **Contract hardening**
   - Freeze payloads, states, conflict/error behavior, and progress schema.
3. **Real runtime integration**
   - Replace mock internals with real LeRobot download/load/warm/execute runtime while keeping API stable.
4. **NOVA app packaging/deployment**
   - installable app, cluster deployment, app-level config
5. **UI/Auth follow-up**
   - app-managed policy selection and private model access/login
6. **Advanced controls**
   - optional collision setup forwarding in runtime path, external stop conditions, multi-robot roadmap

## Mock-first implementation backlog (file-by-file)

### P0 — NOVA app mock runtime + UI

1. Create NOVA app package for policy control (mock backend initially).
2. Implement mock runtime state machine:
   - app states: `EMPTY`, `LOADING`, `READY`, `RUNNING`, `SWITCHING`, `ERROR`
   - run states: `PREPARING`, `RUNNING`, `STOPPING`, `STOPPED`, `TIMED_OUT`, `FAILED`
3. Implement endpoints:
   - list/get policy definitions
   - start policy run
   - stop policy run
   - status/progress
4. UI views:
   - selected policy
   - current state
   - progress (`downloading`, `loading`, `warming`, `running`)
   - start/stop actions

### P1 — SDK extra bridge

1. Add optional extra dependency in `wandelbots-nova`.
2. Implement bridge client for policy app API.
3. Add MotionGroup-native methods:
   - `motion_group.execute_policy(...)`
   - `motion_group.stream_policy(...)`
4. Ensure mixed workflow works:
   - normal SDK motion (`plan/execute`) + policy execution in same program.

### P2 — Tests (must pass before real runtime)

1. Mock app tests:
   - state transitions and progress events
   - single loaded policy constraint
   - stop during `PREPARING` and `RUNNING`
2. SDK unit tests:
   - request/response mapping
   - error/conflict handling
3. Integration flow tests:
   - move to home with SDK, then execute mock policy, then stop

## Acceptance criteria for Phase 1

- NOVA app deploys and is operable from UI on a NOVA instance.
- `execute_policy` from SDK triggers mock app and returns deterministic run state.
- `stream_policy` yields progress/state updates and supports external stop.
- If policy is not preloaded, `start/execute` shows `PREPARING` progress before `RUNNING`.
- Single-loaded-policy behavior is enforced and tested.
- Mixed deterministic motion + policy flow is tested.

## Smoke test runbook (Phase 1)

1. Deploy/start NOVA policy app (mock runtime).
2. Confirm UI shows `EMPTY` state.
3. Run SDK example:
   - move robot to home via `plan/execute`
   - call `execute_policy(...)`
4. Verify state flow in API/UI:
   - `PREPARING` (with progress)
   - `RUNNING`
5. Stop run from SDK and verify terminal state (`STOPPED` or `TIMED_OUT`).
6. Request another policy and verify single-policy switching/conflict behavior.

---

## Most important files to reference/touch during implementation

### `wandelbots-nova` (SDK)

- `pyproject.toml`
  - Add and document the new optional extra (`nova-lerobot-policy`).
- `nova/cell/motion_group.py`
  - Main place to add MotionGroup-native APIs (`execute_policy`, `stream_policy`) and stop integration.
- `nova/program/utils.py`
  - Reuse/align cancellation semantics (`stoppable_run`) for streaming policy execution.
- `nova/cell/controller.py`
  - Reference for controller/motion-group resolution and IO interactions used by policy control.
- `nova/utils/collision_setup.py`
  - Canonical handling/validation of `CollisionSetup` / `CollisionSetups` payload shapes.
- `docs/programs_spec.md`
  - Keep service endpoint behavior aligned with start/stop run semantics.
- `docs/public.openapi.yaml`
  - Source of truth for `MotionGroupSetup`/collision payload schema references.

### `wandelbots-lerobot` (runtime/service side)

- `lerobot_robot_nova/lerobot_robot_nova/nova.py`
  - Existing Nova robot execution path (observation/action loop, gripper/IO behavior) to reuse.
- `lerobot_robot_nova/lerobot_robot_nova/config_nova.py`
  - Robot config shape and defaults (controller/motion_group/tcp/gripper/cameras).
- `lerobot_teleoperator_virtual_teleop/lerobot_teleoperator_virtual_teleop/eval_policy.py`
  - Most relevant existing policy inference loop and policy loading flow.
- `lerobot_teleoperator_virtual_teleop/lerobot_teleoperator_virtual_teleop/task_client.py`
  - Existing service-wrapper pattern for lazy heavy imports and sync/async bridging.
- `lerobot_teleoperator_virtual_teleop/lerobot_teleoperator_virtual_teleop/virtual_teleop.py`
  - Lifecycle patterns for connect/calibrate/start/stop/prefetch behavior.
- `lerobot_teleoperator_virtual_teleop/pyproject.toml`
  - Reference for plugin packaging/script entrypoints for the new policy service package.

### New files likely to be created

- In `wandelbots-lerobot` (new package, e.g. `lerobot_policy_nova_service/`):
  - `app.py` (FastAPI routes)
  - `runtime.py` (run manager + warm state)
  - `models.py` (request/response schemas)
  - `policy_loader.py` (public model fetch + cache)
  - `robot_adapter.py` (NOVA bridge using existing robot plugin)
  - `Dockerfile` (small/fast image)
- In `wandelbots-nova` (new package, e.g. `nova_lerobot_policy/`):
  - `client.py` (HTTP client)
  - `models.py` (typed payloads)
  - `motion_group_extensions.py` or direct integration in `motion_group.py`

---

## Open decisions

- final package names
- service endpoint naming (`/execute` vs `/start`)
- how much policy config lives in SDK call vs NOVA app config/UI
- private policy auth flow details for device login (and potential non-HF policy backend)
- single active run per motion group vs per controller/global
