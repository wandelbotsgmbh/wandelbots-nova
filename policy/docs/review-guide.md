# Review Guide — `policy` package (PR #428)

The diff reports ~11.5k lines, but the **hand-written core logic is ~3,500 lines across 23 files**.
Everything else is tests (~3.1k), examples (~1.3k), docs (~1k), and `uv.lock` churn (~480, generated).
This guide walks the core in dependency order so you can review in one or two sittings.

## TL;DR — what this package does

Runs **learned policies** (imitation / reinforcement learning) on NOVA robots. A policy emits
*action chunks* (short sequences of future robot commands); the `PolicyExecutor` streams those to
the robot as overlapping **waypoints** so motion stays smooth, and (for GR00T) blends consecutive
chunks with **Real-Time Chunking (RTC)** so there are no jumps between them.

```
policy model ──chunk──▶ PolicyExecutor ──waypoints──▶ jogging ──▶ robot
                 ▲                          │
              cameras / IO / state ─────────┘   estop watches in parallel
```

---

## Read in this order

### 1. Contracts — start here (~920 lines)
The types and protocols everything else implements. Understand these and the rest falls into place.

| File | Lines | What to look at |
|---|---|---|
| `policy/types.py` | 158 | `ActionChunk` (frozen Pydantic — note `model_copy`), `RobotState`, core data types |
| `policy/schema.py` | 655 | `PolicySchema` — how robot/camera/IO keys are declared and validated. Large but mostly declarative |
| `policy/policy_client.py` | 105 | The `PolicyClient` protocol every backend implements (`connect` / `get_actions` / `validate_schema` / `close`). The contract for sections 3–4 |

**Review question:** does the schema/contract cleanly separate *what a policy needs* from *how it's executed*?

### 2. The engine — the heart of the PR (~730 lines)
How chunks become timed robot motion.

| File | Lines | What to look at |
|---|---|---|
| `policy/chunking.py` | 144 | Chunk trimming + `placement_start_ms()` — **absolute** (overlapping/RTC) vs **relative** (drift-free) timestamp models. Small but conceptually central |
| `policy/executor.py` | 588 | `PolicyExecutor` — the main loop: `policy_rate_hz` timing (-1 wait / 0 ASAP / >0 fixed), per-group `cartesian`/`joint` mode selection, stop conditions, trimming. **The most important file to review carefully** |

**Review question:** is the timing/trimming logic correct at chunk boundaries? See `_send` and `_create_session`.

### 3. Waypoint jogging — ⚠️ the part blocked on NOVA 26.5 (~1,290 lines)
Streams waypoints to the robot. Depends on the not-yet-public `JoggingApi` waypoint models — this is
why CI can't install deps. Review the logic; the dependency lands with 26.5.

| File | Lines | What to look at |
|---|---|---|
| `policy/jogging/jogger.py` | 552 | `jog_joints` / `jog_tcp` public API + overloads. The `...` overload bodies are idiomatic stubs |
| `policy/jogging/waypoint_session.py` | 457 | Transport-level session: streaming, reconnection, lifecycle |
| `policy/jogging/waypoints.py` | 112 | Builds `PoseWaypointsRequest` / `JointWaypointsRequest` from steps |
| `policy/jogging/clock.py` | 90 | `JoggingTimeClock` — time scaling |
| `policy/jogging/session.py` | 80 | `JoggingStateTracker` — raises `MotionError` past a drift threshold |

**Review question:** is `WaypointConfig` strictly transport-level (no policy timing leaking in)?

### 4. GR00T backend + RTC (~870 lines)
A concrete `PolicyClient` for GR00T diffusion policies.

| File | Lines | What to look at |
|---|---|---|
| `policy/gr00t/client.py` | 436 | GR00T inference client, modality config, schema validation |
| `policy/gr00t/transport.py` | 170 | Server I/O |
| `policy/gr00t/rtc.py` | 163 | **Real-Time Chunking** — how overlapping chunks are blended. Read alongside `chunking.py` |
| `policy/gr00t/eef.py` | 97 | End-effector helpers |

**Review question:** does RTC require `policy_rate_hz >= 0`, and is that enforced at construction?

### 5. Peripherals — skim (~840 lines)
Supporting subsystems; mostly independent, safe to review last.

| File | Lines | What |
|---|---|---|
| `policy/cameras/webrtc.py` | 340 | WebRTC frame capture |
| `policy/cameras/protocol.py` + `manager.py` | 111 | Camera protocol + lifecycle |
| `policy/io.py` | 178 | IO read/write streaming |
| `policy/estop.py` | 164 | Emergency-stop monitor (runs as a parallel task) |
| `policy/rerun/*` | ~750 | Rerun visualization (logging, blueprint, action-chunk overlays). Pure observability — lowest risk |

---

## What you can safely skip
- **`uv.lock`** — generated; the bulk is registry-source noise, not dependency changes.
- **`policy/examples/`** — runnable samples, not shipped logic.
- **`policy/tests/`** — read selectively next to the code under review (`test_executor.py`,
  `test_chunking.py`, `test_rtc.py` are the highest-signal).
- **`policy/docs/` + `*.md`** — prose.

## Known non-issues
- `...` bodies in `@overload` / `Protocol` methods are idiomatic stubs (CodeQL flags them as false positives — already resolved).
- CI red is purely the unreleased `wandelbots-api-client` 26.5 dependency, **not** code. Merge is gated on the public 26.5 release.
