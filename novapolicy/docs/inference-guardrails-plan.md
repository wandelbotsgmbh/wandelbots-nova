# Policy inference guardrails

Three changes that make the inference path fail loudly instead of moving confidently in the
wrong direction. **All three are implemented**; this records what shipped, what changed
behaviour, and where the reasoning went against the original plan.

Planned against `novapolicy @ be71c93b` and `lerobot 0.6.1` (pinned in `.venv`).

## Scope

**In:** deriving execution settings from the checkpoint and validating what cannot be derived,
declared stale/teardown behaviour, unit conversion on the observation and action path.

**Out:** dataset recording and a LeRobot `Robot` plugin adapter — both belong to data
collection, which happens outside this repo.

The plan came out of reading five LeRobot/ROS 2 bridges
([rosetta](https://github.com/iblnkn/lerobot-robot-rosetta),
[leros2](https://github.com/ngres/leros2),
[lerobot-ros](https://github.com/ycheng517/lerobot-ros),
[zenoh](https://github.com/ROBOTIS-GIT/lerobot_robot_ros2_zenoh),
[lerobot_ws](https://github.com/Pavankv92/lerobot_ws)) against `novapolicy`. All of them
implement LeRobot's `Robot` interface — the robot is a plugin and LeRobot's loop drives it.
`novapolicy` runs the opposite way: NOVA owns the loop and the policy is a stateless service.
That direction is not up for discussion here; what follows takes the *contracts* those
projects declare, which is where `novapolicy` is genuinely thin.

## Verified current state

| Anchor | Fact |
| --- | --- |
| `lerobot/config.py` | `load_execution_settings` reads only `type`, `chunk_size`, `n_action_steps` from `config.json`. **Feature keys and shapes go unread.** |
| `lerobot/schema.py` | `validate_schema()` checks only internal consistency — duplicate targets, joint/TCP conflicts on one motion group. **Nothing is compared to the checkpoint.** |
| `executor.py:109` | `camera_max_age_s` defaults to **30.0 s**, applied as one global in `CameraManager`. A frozen feed drives the policy for thirty seconds before anything raises. |
| `executor.py:448` | `_get_policy_actions` awaits `get_actions` with **no executor-level deadline**. Only `LeRobotGrpcTransport` has one (15 s); GR00T and callback clients set their own or none. |
| `executor.py:237` | `_cleanup` always calls `session.stop()`, which **cancels and drops scheduled waypoints**. `drain()` exists and runs them out, but nothing calls it. |
| `schema.py` | `Mapping` / `BoolMapping` apply to **IO only**. Joints and TCP pass through raw — radians and millimetres, whatever the dataset used. |

---

## 1. Derive from the checkpoint; validate only what cannot be derived

**Status: implemented.** `lerobot/config.py`, `lerobot/schema.py`, `lerobot/client.py`,
`policy_client.py`, `executor.py`.

Settings are read through LeRobot's own `PreTrainedConfig` rather than by parsing `config.json`
field by field, so **every LeRobot policy is supported with no per-policy table to maintain**.
Policies disagree about what the action chunk is called — ACT `chunk_size`, diffusion `horizon`,
fastwam `action_horizon`, vqbet nothing at all — but every policy config must implement the
abstract `action_delta_indices` property, and its length is the chunk length for all of them.
Verified across the 19 registered policies in lerobot 0.6.1: 18 derive a chunk length, and the one
that does not (`gaussian_actor`) genuinely has no action chunk.

Two behaviour changes to know about:

- `PolicyExecutor(n_action_steps=...)` now defaults to `None` ("use the policy's declared horizon")
  rather than `0` ("all steps"). Code that used `LeRobotPolicyClient` *and* omitted the argument
  previously executed the whole predicted chunk and now executes the checkpoint's `n_action_steps`
  — which is what the checkpoint intends, but it is a change. Passing `0` explicitly still means
  all steps, so the documented continuous asynchronous-queue setup is unaffected.
- `LeRobotPolicyClient` no longer assumes `policy_type="act"`. When the checkpoint is readable the
  type is derived; when it is not, `policy_type` must be passed explicitly. Guessing it makes the
  server build the wrong policy class, which is exactly the failure this item exists to prevent.

Two narrower consequences of loading through LeRobot: a JSON file under a name other than
`config.json` is no longer accepted (LeRobot's loader reads a checkpoint directory), and feature
parsing is now LeRobot's, so an unknown feature type fails the load instead of being skipped.

### Problem

A `PolicySchema` whose state width or image set has drifted from the trained model produces
plausible motion that is quietly wrong. Nothing catches it today. The checkpoint already
declares the contract in `input_features` / `output_features` — `{key: {type, shape}}` — and we
already build the matching structure client-side in `LeRobotSchema.features()`. The two are
simply never compared.

But validation alone is the wrong end of the problem. Today the user hand-copies values the
checkpoint already knows, straight out of `lerobot/README.md`:

```text
policy_type=settings.policy_type,
actions_per_chunk=settings.chunk_size,
...
n_action_steps=settings.n_action_steps,
```

Every one of those is a chance to mistype something derivable. Catching the mismatch and then
making the user fix it by hand is worse than never asking for it. **Derive what the checkpoint
knows; validate the rest; require nothing.**

### Change

1. Load the checkpoint through LeRobot's `PreTrainedConfig.from_pretrained`, which yields
   `input_features` / `output_features` as typed `PolicyFeature` objects and the chunk length as
   `len(action_delta_indices)`. `chunk_size` and `n_action_steps` become `int | None` on
   `LeRobotExecutionSettings`, so a policy that declares no chunk still gets schema validation.
2. **Derive by default.** `LeRobotPolicyClient` loads the checkpoint itself and fills
   `policy_type`, `actions_per_chunk` and `n_action_steps` from it; the executor takes
   `n_action_steps` from the client rather than from a separate user argument. These constructor
   arguments become **optional overrides**. Camera frame size is *not* derived — see below.
3. **An explicit value that contradicts the checkpoint is an error, not a silent win.** A
   different `policy_type`, or a chunk length above the checkpoint's, raises. A shorter horizon
   warns, since a receding horizon is legitimate.
4. **Validate what is left, automatically.** `LeRobotSchema.assert_matches(...)` is called
   internally from `LeRobotPolicyClient` **after camera connect and immediately before the setup
   message goes out** — it has to sit there, because `features()` needs the first camera frame to
   know the image shape. There is no public opt-in and nothing for the user to remember to call.
5. Normalize both sides through `lerobot.utils.feature_utils.dataset_to_policy_features()`
   before comparing. Our `features()` emits HWC with `names: [height, width, channels]`; the
   checkpoint stores CHW. That helper does exactly this names-driven conversion, so we compare
   `PolicyFeature` to `PolicyFeature` rather than hand-rolling an axis fixup that will rot.
6. Compare three things: the **key set** (missing and extra keys named individually), the
   **shape** per key, and the **action width** from `output_features["action"]` against the
   width `action_layout()` derives from the schema. Raise one error listing every mismatch, not
   the first.

### What cannot be derived

**`fps` stays a user parameter.** There is no `fps` field in `PolicyConfig` or in ACT's config —
`configuration_act.py` declares `chunk_size` and `n_action_steps` and no rate. Frame rate is a
property of the dataset, not of the policy checkpoint. `playback_speed` goes with it.

**Deriving needs a client-readable checkpoint.** `lerobot/config.py` already states the
constraint: "Server-local absolute paths cannot be inspected by the NOVA client" — which is why
the README carries `SERVER_CHECKPOINT` and `CLIENT_CHECKPOINT_CONFIG` as two separate constants.
The client can derive only when it has a local copy or a Hub id. Otherwise it falls back to the
explicit arguments and **warns once** that neither derivation nor validation could run. That
fallback must never be silent.

### Two honest limits on the validation half

`PolicyFeature` carries `{type, shape}` and no per-dimension names, so this validates keys and
widths — it cannot detect a permuted joint order within a correctly sized state vector. That
gap is what the range check in item 3 covers.

`input_features` is legitimately allowed to be `None` or empty (features inferred from the
dataset at train time), so: **fail loudly when features are present and mismatched, warn once
when absent.** Never silently pass.

### Camera frame size warns rather than resizing

An earlier draft had the client derive the frame size and apply it to the camera source. That was
wrong on two counts. `resize=` scales frames *on read*, after they have crossed the network, so it
pays full-resolution bandwidth and decode cost either way; and it is a plain rescale, so pointing
a 16:9 stream at a 4:3 target distorts the image. Some policies (pi0, pi0_fast, pi05, smolvla,
xvla) resize with padding internally and tolerate it — **ACT does not resize at all** and sees the
distortion directly.

**Settled with the Camera App owner: resolution stays user-controlled.** Cameras cannot always
deliver an arbitrary resolution, so novapolicy never overrides the configured stream. A `VISUAL`
shape mismatch warns and the run continues, distinguishing the two cases because they mean
different things:

- **Resolution differs, aspect ratio matches** — frames rescale cleanly; the cost is bandwidth,
  decode time and some detail.
- **Aspect ratio differs** — rescaling cannot correct it; the image is stretched and a policy
  without internal padding sees geometry that never appeared in training.

Every other mismatch — a missing or unexpected observation key, a wrong state width, a wrong action
width — still raises.

### Tests

`test_lerobot_config.py` for parsing, including the absent-features case and chunk-length
derivation across act/diffusion/tdmpc/pi05/molmoact2/vqbet. `test_lerobot_schema.py`
for the comparison: matching schema, wrong state width, missing camera key, extra camera key,
CHW/HWC round-trip, action-width mismatch. `tests/api/test_lerobot_client.py` for derivation: no
explicit arguments at all, an explicit argument agreeing with the checkpoint, an explicit
argument contradicting it, and a server-only checkpoint falling back with a warning.

---

## 2. Declare what happens when data goes stale — and on the way out

**Status: implemented.** `types.py`, `schema.py`, `cameras/manager.py`, `executor.py`.

One behaviour change: `camera_max_age_s` drops from **30.0 s to 1.0 s**. A frozen feed now surfaces
in about a second instead of driving the policy for half a minute. Setups running a slow WebRTC
stream that previously limped along will start failing — that is the point, but it needs a release
note.

Two decisions worth recording, both deviations from this plan as first written:

- **The response is executor-wide, not per action channel.** The *trigger* is per channel — one
  camera can go stale on its own — but one policy drives the whole cell, so holding one arm while
  aborting another is incoherent. Per-channel freshness, one declared response.
- **`HOLD` is refused for an inference deadline.** `LeRobotPolicyClient.get_actions` runs its gRPC
  call in `asyncio.to_thread`; a timeout cancels the await but not the thread. Retrying would
  re-enter a client whose previous call may still complete and advance `_timestep`. An inference
  deadline therefore always ends the run.

The plan also called for a rosetta-style `Align(tolerance_ms=...)`. Dropped: `CameraSource.read`
already takes `max_age_s`, so a second name for the same quantity buys nothing.

**Touches:** `executor.py`, `schema.py`, `cameras/manager.py`

### Problem

The current behaviour isn't unsafe, it's *undeclared*. A stale frame raises out of the loop and
`_cleanup` stops every session, so the robot does come to rest — but the 30-second default means
a frozen camera can feed thirty seconds of stale observations to a policy that keeps commanding
motion first. Meanwhile a hung inference server has no executor-level deadline at all, and
teardown always drops the accepted lookahead even when the run ended normally.

### Change — four separable pieces

1. **Retie camera staleness to the control rate.** Replace the global 30 s with a per-channel
   `Align(tolerance_ms=...)` on `Observation.image(...)`, defaulting to a small multiple of the
   frame period at the configured fps. Keep `camera_max_age_s` as the fallback for channels that
   don't declare one.
2. **Add an inference deadline in the executor.** Wrap the `get_actions` await in
   `_get_policy_actions` with `asyncio.timeout()`, expressed in frame periods rather than
   seconds so it scales with fps. This makes the guarantee uniform across LeRobot, GR00T and
   callback clients instead of depending on each transport's own timeout.
3. **Declare the response.** An `on_stale` on each action channel: `hold` (keep the last
   commanded target and retry), `controlled_stop` (drain the accepted lookahead, then end the
   run), `abort` (today's behaviour — raise). Whichever fires goes into `ExecutionResult.reason`.
4. **Fix teardown to match the reason.** Map stop-reason to method: a stop condition or timeout
   ends normally and should `drain()` the accepted waypoints; an error or e-stop should `stop()`
   and drop them. Right now everything takes the drop path.

### Scope this honestly

Under `SequentialExecution` the robot is already at a measured standstill while inference runs,
so the deadline mostly changes the error message. The declared behaviour earns its keep under
`ContinuousExecution`, where the lookahead is live and what happens next is a real choice. Say
that in the docs rather than overselling it.

### Do not adopt rosetta's `zeros`

It's sane for velocity and effort channels. `novapolicy` streams absolute joint positions, where
zero commands an industrial arm to its zero pose. The vocabulary is `hold | controlled_stop | abort`.

### Tests

`tests/api/test_executor.py`: a policy client that sleeps past the deadline, asserting each
`on_stale` branch and the resulting `reason`. A camera stub with a controllable frame age. A
teardown test asserting `drain()` on a stop condition and `stop()` on an error.

### Behaviour change to call out in the changelog

Tightening the camera default from 30 s will make some currently-passing setups start failing.
That is the point, but it needs a release note, not a silent bump.

---

## 3. Units: declared, invertible, and checked against the checkpoint's own statistics

**Status: implemented.** `ops.py` (new), `schema.py`, `executor.py`, `lerobot/config.py`,
`lerobot/schema.py`, `lerobot/client.py`.

No behaviour change: `ops=` defaults to empty, and the range check only runs when a checkpoint
ships normalization statistics the client can read.

Four decisions worth recording, all deviations from this plan as first written:

- **Round-trip verification collapsed to constructor validation.** Rosetta probes operators because
  it loads them from config strings; ours are library-owned classes. `Rad2Deg` is exact by
  construction and `Scale` validates its factor in `__init__`, so the only load-time check left is
  the direction one: a forward-only operator on a writable channel raises.
- **`Clamp` applies in both directions.** Clamping a *command* before it reaches the robot is the
  safety-relevant direction, and a pure pass-through inverse would silently give a user who wrote
  `ops=[Clamp(-1, 1)]` nothing. Clamping is monotone, so its position survives the reversal.
- **TCP takes two operator lists.** Position is millimetres and orientation radians; one list
  cannot serve both, so `Observation.tcp` takes `position_ops` and `orientation_ops`.
- **The seam is schema-level, as recommended.** `PolicySchema.apply_inverse_ops(chunk)` runs in the
  executor immediately before `_apply_relative_mode`. That ordering is load-bearing and has a test:
  a delta in degrees added to a state in radians is the error the operators exist to prevent.

One honest limit on the statistics check: bounds catch values that are too *large* for the training
range — degrees where radians were expected. The reverse sits comfortably inside a degree range and
cannot be caught this way.

**Touches:** `schema.py`, `executor.py`, `lerobot/config.py`

### Problem

The LeRobot policy server normalizes and unnormalizes server-side, from the checkpoint's own
stats — verified in `policy_server.py`, which builds pre- and post-processors at load. So we must
**not** build a normalization layer. What is genuinely unguarded is *units*: those stats are in
the recording robot's units, and NOVA hands over radians and millimetres. A dataset captured in
degrees normalizes perfectly cleanly and moves the arm to the wrong place.

### Part A — an invertible operator list

1. Generalize `Mapping` into an ordered `ops=[...]` on `Observation.joint_positions` and
   `Observation.tcp`. Ship a deliberately small vocabulary: `Rad2Deg`, `Scale(factor)` for
   mm↔m, `Clamp(min, max)`.
2. Classify each op by direction, following rosetta: **bijective** (`Rad2Deg`, `Scale`) and
   **bidirectional** (`Clamp`) may sit on a channel that is also an action; forward-only ops may
   not, and the schema rejects them at construction. Verify bijective ops by round-trip at load,
   so a wrong inverse fails at import rather than on the robot.
3. Ops apply forward on observation and inverse on action, in reverse order. They do not apply
   to `Observation.computed` or `Observation.constant` — those are opaque by definition.

**The real design decision is the inversion seam, not the vocabulary.** Today IO inversion
(`mapping.to_hardware`) lives inside `LeRobotSchema.decode_arrays` — inside a client. Putting
joint and TCP inversion there too would duplicate it in every backend we add.

*Settled:* forward ops in `PolicySchema.build_observation`, inverse through one schema-level
`apply_inverse_ops(chunk)` called from the executor right before `_apply_relative_mode`.

The follow-up this plan proposed — migrating IO's `Mapping` to the same seam — was investigated
and **rejected**. The claim that it was "duplicated per backend forever" was wrong: the conversion
already lives in one shared place (`Mapping`), and both clients call it with the same object from
`schema.io_action_keys`. What differs per backend is only where the value sits on the wire — a flat
array slice for LeRobot, a named `(B, T, 1)` array for GR00T — which no seam can unify.

Moving it would also break a contract: `ActionChunk.ios` carries **hardware** values, which is why
a hand-built chunk can write `True` directly (see `examples/replay_lerobot_dataset_ur3.py`).
Converting at the schema seam would double-convert those, silently. `Mapping` and `ValueOp` stay
separate: one crosses a type boundary at the client, the other transforms units at the schema.
Both docstrings now say so.

### Part B — the empirical check that proves the units are right

Declared units are an assertion by the person writing the schema. The checkpoint can settle it:
normalization stats ship in the pretrained directory as `policy_preprocessor.json` plus
safetensors state files, giving per-dimension statistics for `observation.state` and `action`.

1. At connect time, take the first live observation and compare each state dimension against the
   training distribution: **min/max where `MIN_MAX` stats are present, mean ± k·σ where they are
   `MEAN_STD`**, since `NormalizationMode` spans `MIN_MAX`, `MEAN_STD`, `IDENTITY` and two
   quantile variants. Skip dimensions with no usable stats.
2. A dimension far outside its training range means wrong units, wrong joint order, wrong robot
   or wrong TCP — in one check. Report the offending dimension index, the live value and the
   trained range, then refuse to start.

**Make part B opt-in.** It needs `policy_preprocessor.json` and the normalizer safetensors
client-side; the README today promises only that `config.json` is readable by the client. Default
to on when the files are there, warn once and continue when they aren't, and document the extra
files as recommended rather than required.

### Out of scope

Rotation-representation conversion — rotation vector to quaternion or Euler. It is not a scalar
op, it interacts with the TCP frame, and no current checkpoint needs it. Ship the scalar ops first.

### Tests

`tests/unit/` for op round-trips and the rejection of a forward-only op on an action channel.
`tests/api/test_lerobot_client.py` for the full path: schema with `Rad2Deg` → observation in
degrees → action back in radians. Stats fixtures for both normalization modes, plus in-range and
out-of-range first observations.

---

## What a configured channel looks like when all three land

```python
schema = PolicySchema(
    observations=[
        Observation.joint_positions(
            "arm",
            source=arm,
            ops=[Rad2Deg()],  # bijective - inverted on the action path
            on_stale=OnStale.CONTROLLED_STOP,  # declared, not emergent
        ),
        Observation.image(
            "scene",
            source=cameras.device(DEVICE),
            align=Align(tolerance_ms=200),  # per channel, ~3 frames at 15fps
        ),
    ]
)

policy = LeRobotPolicyClient(
    server_address=LEROBOT_SERVER,
    pretrained_name_or_path="./my_policy",  # dir, not just config.json
    fps=FPS,  # not in the checkpoint - stays explicit
)
# policy_type, actions_per_chunk and n_action_steps are read off the
# checkpoint - for any LeRobot policy, via action_delta_indices. Pass one
# to override it; a value that contradicts the checkpoint is reported.

executor = PolicyExecutor(schema, policy, execution=SequentialExecution())

# Validation runs inside the client, right before the setup message:
# key set, shapes, action width, then the first live observation against
# training stats. It fails here, not three waypoints into a move.
result = await executor.run()
```

## Sequencing

Ordered so each item ships independently. Item 1 first because it connects pieces that already
exist; item 3 last because it changes the public schema surface.

| Item | Why this order | Size | Breaking? |
| --- | --- | --- | --- |
| 1 · Derive + validate ✓ | Standalone. Parsing, layout derivation and the validation hook all exist; this wires them together and removes four hand-copied values. Highest value per line changed. | small–medium | No — explicit arguments become optional overrides, so existing call sites are unaffected |
| 2 · Stale & teardown ✓ | Adds config surface and touches the executor loop, but no schema redesign. Independent of item 1. | medium | Yes — camera default tightens 30s → 1s |
| 3 · Units + stats check ✓ | Last: it changes `PolicySchema`'s public API and moves the inversion seam. Item 1 should be proven first, since the range check builds on the same checkpoint-loading path. | large | No, if `ops=` defaults empty |

## Open decisions

- ~~**An explicit argument that contradicts the checkpoint — warn or fail?**~~ *Settled as
  proposed:* `policy_type` raises on disagreement; a chunk length above the checkpoint's raises;
  a shorter horizon is accepted with a warning. Camera frame size warns rather than raising.
- ~~**Does `on_stale` belong per channel or per motion group?**~~ *Settled: neither.* Freshness is
  declared per channel, the response executor-wide — see item 2.
- ~~**Where does the inversion seam live?**~~ *Settled: schema-level.* `apply_inverse_ops` runs
  from the executor. Migrating IO's `Mapping` to it was investigated and rejected — see item 3.
- **Is requiring the full checkpoint directory client-side acceptable?** Shipped as
  degrade-quietly: statistics are used when present, skipped when not. Still worth knowing whether
  real deployments ever have more than `config.json` client-side — if not, the check never runs.
- **How hard should the camera default bite?** Shipped at 1.0 s, down from 30.0 s. Tighten
  further per channel with `Observation.image(..., max_age_s=...)` where a camera warrants it.

---

Anchors verified in-tree: `lerobot/config.py`, `lerobot/schema.py`, `schema.py`, `executor.py`,
`cameras/manager.py`, `cameras/webrtc.py`, `jogging/waypoint_session.py`, and lerobot's
`policy_server.py`, `configs/policies.py`, `configs/types.py`, `utils/feature_utils.py`,
`utils/constants.py`.
