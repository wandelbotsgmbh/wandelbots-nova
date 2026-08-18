# Completion-detection experiments

Measurement harness for the problem described in
`docs/architecture/incoming/execution-completion-detection.md` (and the research companion
`…-research.md`): what does the `MotionGroupState` stream actually deliver around the end of a
motion, at which rates, over which connections — and what would the SDK's execution state machine
conclude from exactly those frames?

## What one run records

A short motion (default: TCP −Z/+Z, `--amplitude` mm, planned via the SDK) is executed while:

| output file | content |
|---|---|
| `frames_<rate>.jsonl` | every frame received on a raw `state-stream` websocket at that rate (`step` = no `response_rate` param), with wall/monotonic receive timestamps. One observer per entry in `--rates`, all watching the *same* motion. |
| `poll.jsonl` | GET `…/motion-groups/{mg}/state` sampled at `--poll-interval` (the pull endpoint, research §8) |
| `events.jsonl` | timestamped experiment events: init/start/pause/resume sent, acks received, sdk `execute()` returned/timed out |
| `metadata.json` | host, mode, planned duration, planned final joints (for the at-target detector), … |
| `summary.json` | written by `analyze` |

Execution paths (`--mode`):

- **`sdk`** — `MotionGroup.execute()` (move_forward + `TrajectoryExecutionMachine`). A hang is
  detected by a `3×planned+15s` timeout and recorded, not fatal.
- **`api`** — `wandelbots_api_client` only, mirroring `arg3-api`: `add_trajectory` +
  `execute_trajectory` websocket with a minimal request generator. The run length is *time-based*
  (planned duration + margin + post-roll) so the recording never depends on any completion
  detector — the thing under test.

Scenarios (`--scenario`): `complete` (default) and `pause` (api mode only: pause at
`--pause-at`×duration, hold `--pause-hold-s`, resume, complete).

## Running

### Choosing the robot (virtual instances: k8s / cloud)

- **Existing controller:** pass `--controller <name>` — nothing is provisioned, the harness only
  moves that controller's first motion group.
- **Different virtual robot:** `--virtual` accepts a shorthand (`ur10e`, `ur5e`, `kr16`, and
  `kr270`/`kr240` for the ARG3 production models) *or any motion-group model string* of the
  instance — both the catalog form (`KUKA_KR270_R2700`, as printed by `--list-robots`) and the
  legacy form (`kuka-kr270_r2700`) work; catalog names are normalized to the form the
  add-controller API accepts. Discover what the instance supports:

  ```bash
  PYTHONPATH=. uv run python -m experiments.completion_detection.runner --list-robots
  ```

  The controller name defaults to `cdexp-<model>`, so different robots coexist in one cell and a
  `--virtual` change never reuses an old controller of the wrong type.

  **Caveat when extrapolating:** virtual controllers do not reproduce the physical terminal-state
  timing. A virtual KR270 steps at the production 4 ms rate but holds `END_OF_TRAJECTORY` for
  ~16 steps (~64 ms), whereas the physical controller publishes it for a single step
  (briefing §3). Virtual runs therefore *understate* the edge-loss problem; only the physical
  cell measures it truthfully.

```bash
# 1) complete, api-only path, observers at step rate and 200 ms, virtual robot auto-provisioned
PYTHONPATH=. uv run python -m experiments.completion_detection.runner \
    --mode api --virtual ur10e --label dev-k8s --runs 5

# 2) same motion through the SDK (does execute() return?)
PYTHONPATH=. uv run python -m experiments.completion_detection.runner \
    --mode sdk --virtual ur10e --label dev-k8s --runs 5 --batch <batch-from-step-1>

# 3) pause/resume comparison
PYTHONPATH=. uv run python -m experiments.completion_detection.runner \
    --mode api --scenario pause --virtual ur10e --label dev-k8s --batch <batch>

# 4) rate sweep
PYTHONPATH=. uv run python -m experiments.completion_detection.runner \
    --mode api --rates step,50,200,500 --virtual ur10e --label dev-k8s --batch <batch>

# analyze + visualize a batch
PYTHONPATH=. uv run python -m experiments.completion_detection.analyze  experiments/completion_detection/data/<batch>
PYTHONPATH=. uv run python -m experiments.completion_detection.visualize experiments/completion_detection/data/<batch>
# → experiments/completion_detection/data/<batch>/report.html
```

## Experiment matrix

Repeat the same batch (steps 1–4 above) per connection scenario; `--label` tags the environment:

| label | connection | robots | notes |
|---|---|---|---|
| `lan-ipc` | LAN, connected to the IPC | physical (+ virtual) | **someone must be at the cell**; see safety below |
| `dev-k8s` | dev machine → k8s instance | virtual | WAN latency is not evidence of a load problem (briefing §10) |
| `dev-cloud` | dev machine → NOVA cloud | virtual | later; needs `NOVA_ACCESS_TOKEN` |

Within each environment the matrix is: `mode ∈ {sdk, api}` × `scenario ∈ {complete, pause}` ×
rate sweep, ≥5 runs each for the intermittent effects.

## What the analysis answers

- **Frame census** per rate (reproduces briefing §3): how many `RUNNING` / `END_OF_TRAJECTORY` /
  no-`execute` frames; was the terminal edge delivered at all?
- **FSM replay**: the state sequence the *shipped* `TrajectoryExecutionMachine` would go through
  on exactly the recorded frames — `final_state != ended` ⇒ this stream would have hung the SDK.
- **Detector comparison** on identical data:
  `edge_fsm` (shipped) vs `arg3_fallback` (briefing §6b) vs `level`
  (standstill + no/terminal execute + at planned target + 1 s dwell — the CiA-402-shaped detector,
  research §9). Detection times are also given relative to the observed end of motion.
- **Pull endpoint** behaviour: does GET return `execute` during motion, at what latency
  (research open question §10.5).
- **Sequence-number gaps**: whether `sequence_number` can serve as a missed-transition detector
  (research open question §10.4).

## Safety (physical cells)

From the briefing §10 — read it before running against an IPC:

- Confirm what you are connected to first: `curl $NOVA_API/api/health`, then list controllers.
  This harness never provisions controllers unless `--virtual` is passed, and only ever moves the
  motion group given by `--controller`.
- On a physical cell someone must be at the robot; the default motion is ±`--amplitude` mm in Z
  from the *current* pose.
- Planning uses the SDK's payload resolution (`MotionGroup.get_setup`). Verify the controller has
  the correct payload configured before planning on a physical KUKA — planning unloaded caused
  motor-torque faults in production, which require manual recovery at the panel.
