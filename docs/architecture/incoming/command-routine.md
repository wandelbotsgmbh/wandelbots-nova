# CommandRoutine — format analysis and SDK impact

Status: **research / pre-draft.** Nothing implemented. This document exists to
decide what the SDK must grow before the downstream services (trajectory-cursor,
TouchUp, omniservice) can be reworked onto the new format.

Date: 2026-07-28
SDK baseline analysed: `main` @ `4c2fc20` (5.9.0)

---

## 0. Provenance and verification status

The format originates in service-manager MR
`https://code.wabo.run/wandelos/service-manager/-/merge_requests/2642`.

**The MR itself could not be read.** `code.wabo.run` resolves but all TCP/ICMP
traffic times out — the host is VPN-gated. The analysis below is therefore based
on the local checkout at `/Users/q/0code/service-manager`, branch
`feat/trajectory-format-components` @ `b21c51d7d` *"feat: add trajectory format
teaching components"* (Dirk Sonnemann, 2026-07-23), which introduces
`api/v2/teaching/components/schemas/CommandRoutine.yaml` plus 39 sibling schemas.

**Unverified:** that this commit *is* MR 2642, and that the MR has no newer
revisions. Re-check on VPN before acting on §2 (the spec-feedback section).

Superseded predecessor: branch `feat/trajectory-format-openapi` @ `5e4a4c450`
(`api/v2/trajectory/…`, `Trajectory`/`TrajectoryItem`/`WaypointRef`). The
narrative pre-draft lives in `nova-developer-tools/apps/trajectory-format/docs/spec.md`.

---

## 1. What CommandRoutine is

A **pre-planning** representation: an ordered list of motion commands and
execution-overlay commands, with polymorphic targets. It compiles *down* to
`JointTrajectory` + `StartMovementRequest` overlay — it is not a wire format for
execution.

Root schema — `teaching/components/schemas/CommandRoutine.yaml`:

```
CommandRoutine
├─ id, dataset_id, name           required
├─ description
├─ motion_group          MotionGroupReference | null    {type:"id", id}
├─ motion_group_setup    MotionGroupSetup | null         required, nullable
├─ tcp                   string | null
├─ start_joint_position  DoubleArray | null              radians
├─ default_motion_settings  MotionSettings
├─ commands              Command[]  minItems:1           required
├─ metadata              {string→string}
└─ created_at / updated_at
```

### 1.1 Two-mode conditional validation

`CommandRoutine.yaml:64-92` uses JSON-Schema `if/then/else`:

| `motion_group_setup` | Consequence |
|---|---|
| `null` | I/O-only routine. `motion_group`, `tcp`, `start_joint_position` must be null; `commands` must **not** contain a motion command. |
| present | `start_joint_position` becomes **required**; `commands` must contain ≥1 motion command. |

### 1.2 Command union

`Command.yaml` — `oneOf` discriminated on `type`:

| `type` | Schema | Layer |
|---|---|---|
| `motion_command` | `CommandRoutineMotionCommand` | planning |
| `set_io` | `SetIOCommand` | overlay |
| `wait_for_io` | `WaitForIOCommand` | overlay |
| `pause_on_io` | `PauseOnIOCommand` | overlay |
| `wait_for_time` | `WaitForTimeCommand` | overlay |
| `marker` | `MarkerCommand` | overlay |

Motion commands own the integer locations. Overlay commands are positioned by an
optional `at` trigger relative to the nearest preceding motion.

### 1.3 Motion commands

`CommandRoutineMotionCommand.yaml` is a **bare `oneOf` with no discriminator**:

- `ExplicitCommandRoutineMotionCommand` — requires `target` + `path_type`
- `GeneratedCommandRoutineMotionCommand` — requires `target` + `generator`

Both carry `type: "motion_command"`; they are separable only by which of
`path_type` / `generator` is present. Both also take optional `motion_settings`,
`intent` (free-form authoring annotation), `metadata`.

`PathType.yaml` discriminates on `path_definition_name`, **with the target
hoisted out** — no `target_pose` inside the path variant:

`PathLine` · `PathCartesianPTP` (requires `kinematic_configuration`) ·
`PathJointPTP` · `PathCircle` (`via_pose`) · `PathCubicSpline` (`via_points`) ·
`PathDirectionConstrainedCartesianPTP` · `PathDirectionConstrainedJointPTP`

`MotionGenerator.yaml` = `algorithm` (`CollisionFreeAlgorithm`, required) +
`constraint` (`DirectionConstraint`, optional).

### 1.4 Targets — `PoseRef.yaml`

Discriminated on `type`:

| Variant | Payload | Self-contained? |
|---|---|---|
| `inline_pose` | `allOf ConfiguredPose` + `tcp` override | yes |
| `joint_position` | `joints` (DoubleArray, rad) | yes |
| `local_pose` | `pose_id` within the enclosing `DatasetBundle` | **no — needs the bundle** |
| `dataset_pose` | `cell` + `dataset_id` + `pose_id`, optional `resolved_pose` cache | **no — needs a teaching-API client** |

`ConfiguredPose` = `pose` + `kinematic_configuration` + `coordinate_system_id`.
`KinematicConfiguration` = `kinematic_branch` + `axis_ranges` — vendor-neutral,
and a proper replacement for the ad-hoc `abb_confdata` hack currently in
`nova-developer-tools/apps/trajectory-cursor/trajectory_cursor/ik_scoring.py`.

### 1.5 Triggers — `AtTrigger.yaml`

Discriminated on `type`; `AtReference` = `previous | next`.

| Trigger | Fields |
|---|---|
| `path_fraction` | `value`, `minimum: 0`, `exclusiveMaximum: 1` |
| `distance` | `millimeters` ≥ 0, `reference` |
| `time` | `seconds` ≥ 0, `reference` |

These mirror, almost exactly, the classes on the SDK's **unmerged** branch
`feat/path-triggers-distance-offset` @ `5560279` (`nova/actions/path_trigger.py`:
`PathParameterTrigger` / `DistanceTrigger` / `TimeTrigger`, `TriggerReference`).
Strong evidence the MR was authored against that branch. Two discrepancies:

- naming: format `path_fraction` vs SDK `path_parameter`
- range: format `[0, 1)` vs SDK docstring "1.0 = at the next motion"

### 1.6 Settings inheritance — three-state

`MotionSettings.yaml` = `blending` (`Blending | null`) + `limits_override`
(`LimitsOverride | null`).

**`null` = suppress inherited. Absent = inherit from `default_motion_settings`.**
This is three-state, not two, and will be modelled wrong by default in Pydantic
(`Optional[X] = None` collapses null and absent). Needs `model_fields_set`.

### 1.7 IO expressions

`IOExpression.yaml` is a **recursive boolean tree**: `condition` | `all_of` |
`any_of` | `not`. `IOConditionExpression` = `io` (`IOValue`) + `comparator` +
`negate` + `io_origin`.

### 1.8 Envelope and endpoint

`DatasetBundle.yaml` = `Dataset` + `poses: DatasetPose[]` +
`command_routines: CommandRoutine[]`, exposed **read-only** at
`GET /teaching/datasets/{dataset}/bundle` (tag `Teaching`, scope
`can_read_teaching`).

**The MR adds no write endpoint for CommandRoutine.** Consumers must accept a
routine in their own request body; they cannot pass an ID.

---

## 2. Spec feedback to raise on the MR

Ordered by how much they will cost us downstream.

1. **`Command` discriminator points at a non-concrete schema.**
   `mapping.motion_command → CommandRoutineMotionCommand.yaml`, which is itself
   an undiscriminated `oneOf`. Every generated client (Python, Go, TS) will
   handle this badly or refuse it. Ask for a real inner discriminator, e.g.
   `motion_kind: explicit | generated`. **Raise this before merge.**
2. **`if/then/else` + `contains`/`not` will not survive codegen.** Each consumer
   hand-reimplements §1.1 and they drift. Consider two named schemas
   (`IOOnlyCommandRoutine` / `MotionCommandRoutine`).
3. **OAS 3.1 only** (`type: 'null'`, `if/then/else`, numeric `exclusiveMaximum`).
   Any consumer still on 3.0.x cannot vendor these schemas as-is.
4. **`id` / `dataset_id` are required** — awkward for a not-yet-persisted routine
   sent as a request body. A `CommandRoutineCreate` variant is needed.
5. **`path_fraction` range `[0,1)`** contradicts the SDK's `1.0 = at next motion`.
   Pick one.
6. **Three-state `MotionSettings`** (§1.6) should be called out normatively, or it
   will be implemented as two-state everywhere.
7. `InlinePoseReference` uses `allOf` + a discriminator property; several
   generators flatten this incorrectly.

---

## 3. SDK gap matrix

Verified against `main` @ `4c2fc20`. **Two gaps I reported earlier were already
closed on `main`** — noted explicitly to stop them being re-raised.

| Format capability | SDK on `main` | Gap |
|---|---|---|
| `motion_command` PathLine | `lin()` / `linear()` | none |
| PathCartesianPTP | `ptp()` / `cartesian_ptp()` | `kinematic_configuration` not plumbed |
| PathJointPTP | `joint_ptp()` | none |
| PathCircle | `circular()` | check `via_pose` mapping |
| PathCubicSpline | `spline()` | check `via_points` mapping |
| PathDirectionConstrained* | — | **no equivalent** |
| `generator` (collision-free) | `collision_free(target, algorithm=)` | `DirectionConstraint` has no slot |
| `set_io` | `io_write()` → `CombinedActions.to_set_io()` (`nova/actions/container.py:149`) | **not reachable from the cursor** — see §3.1 |
| `wait_for_io` / `pause_on_io` | `api.models.StartOnIO` / `PauseOnIO` = `{io, comparator, io_origin}` | **single condition only; no boolean tree** — see §3.2 |
| `wait_for_time` | `wait()` → `WaitAction` in `nova/actions/mock.py` | `to_api_model()` returns a plain dict; **probably not executable** |
| `marker` | — | **no signal, no event type** — see §3.3 |
| `at` triggers | absent on `main`; present on unmerged `5560279` | **branch must land first** — see §3.4 |
| `MotionSettings.limits_override` | `nova.types.motion_settings.MotionSettings` | field-by-field mapping + three-state merge |
| `blending` | `blending_auto` / `blending_radius` | `BlendingPosition` has 4 fields, SDK has ~2 |
| `local_pose` / `dataset_pose` | — | **no teaching-API client, no bundle concept** |

### 3.1 `set_outputs` never leaves the cursor — real gap

`TrajectoryCursor`'s `Intent.to_commands()`
(`nova/cell/movement_controller/trajectory_cursor.py:386-415`) builds
`StartMovementRequest` with `direction`, `target_location`, `start_on_io`,
`pause_on_io` — but **not `set_outputs`**.

The non-cursor path does it correctly:
`nova/cell/movement_controller/move_forward.py:107-112` calls
`context.combined_actions.to_set_io()` and passes `set_outputs=set_io_list`.

So today, a routine's `set_io` commands are silently dropped when executed
through a cursor. This is the single most important overlay kind. **Fix first.**

### 3.2 IO conditions are single-comparison only

`api.models.StartOnIO` / `PauseOnIO` expose exactly `{io, comparator, io_origin}`.
The format's `IOExpression` is a recursive `all_of`/`any_of`/`not` tree. There is
no way to express a compound condition against the current API.

Options: (a) restrict SDK support to the single-`condition` case and reject the
rest; (b) push a baseline API change. **Decision needed.**

### 3.3 Markers have no transport

`trajectory_cursor.py:322-337` defines only `motion_started` / `motion_stopped`
signals and `MotionEventType.{STARTED,STOPPED}`. Markers need either a
`marker_reached` signal or a new event type, fired when the cursor crosses a
marker's resolved location — including on backward travel, whose semantics are
undecided.

### 3.4 Path triggers are not on `main`

`nova/actions/path_trigger.py`, `path_trigger_resolver.py`,
`path_trigger_api_compat.py` and `io_write(at=…)` exist only on
`feat/path-triggers-distance-offset` @ `5560279`. `main`'s `io_write()`
(`nova/actions/io.py:30-35`) has no `at` parameter, and `api.models.SetIO` has
fields `{io, location, io_origin}` — no `distance_offset`.

**Landing that branch is a prerequisite for `at` trigger support.**

### 3.5 Already fixed on `main` — do not re-raise

- `Intent` **does** carry `start_on_io` / `pause_on_io` and passes them through
  (`trajectory_cursor.py:383-384`, `:411-412`); `forward()` / `backward()` accept
  them (`:666-667`, `:714-715`).
- `TrajectoryCursor.__init__` now filters `self.actions` to motions only and
  keeps everything on `self._raw_actions` (`trajectory_cursor.py:520-533`), so
  the end-location validation at `:536-545` is correct for mixed action lists.

---

## 4. Proposed SDK scope

**Position:** the SDK owns the CommandRoutine *library/framework tooling* —
parsing, validation, resolution and compilation to actions. Services become thin
consumers that hand a routine to the SDK and get a cursor back.

### 4.1 Module layout (proposal, not decided)

```
nova/teaching/            # new package
  models.py               # CommandRoutine et al. — generated or hand-written (§5, Q1)
  resolver.py             # PoseRef → Pose/joints; PoseResolver protocol
  compiler.py             # CommandRoutine → list[Action] + tcp + start_joints
  validation.py           # the if/then/else rules codegen cannot express
```

Deliberately **not** in `nova/actions/` — the routine is a teaching-domain
concept and `nova.actions` should not depend on teaching schemas.

Proposed entry point:

```python
def compile_routine(routine: CommandRoutine, resolver: PoseResolver) -> CompiledRoutine
# CompiledRoutine: actions: list[Action], tcp: str | None, start_joints: list[float] | None
```

`PoseResolver` is a protocol so callers choose their strategy: inline-only, a
`DatasetBundle` map, or a live teaching-API client.

### 4.2 Changes to existing SDK code

| Change | File | Priority |
|---|---|---|
| Add `set_outputs` to `Intent` + `to_commands()`, resolved from the action list | `nova/cell/movement_controller/trajectory_cursor.py:369-415` | **P0** — blocks `set_io` |
| Land path triggers from `5560279` onto `main` | `nova/actions/path_trigger*.py` | **P0** — blocks `at` |
| Marker signal / `MotionEventType.MARKER` | `trajectory_cursor.py:322-337` | P1 |
| Expose routine `intent` + `current_command_index` on `MotionEvent` | `trajectory_cursor.py:339-358` | P1 |
| Command-level addressing alongside action-level (`current_command`) | `trajectory_cursor.py:427-449`, `:596` | P1 |
| Rename/alias `PathParameterTrigger` → `path_fraction`; reconcile range | `nova/actions/path_trigger.py` (on branch) | P2 |
| Plumb `kinematic_configuration` through cartesian PTP | `nova/actions/motions.py` | P2 |

**Not proposed:** a `TrajectoryCursor.from_command_routine()` constructor. The
cursor is a transport/execution object; pulling teaching schemas into it couples
two layers that are currently independent. Keep compilation in `nova/teaching/`
and keep the cursor taking `actions` + overlay.

### 4.3 Downstream consumer shape (trajectory-cursor service)

Once §4.1/§4.2 exist, the service reduces to:

- new endpoint `PUT …/cursors/from-command-routine` (do **not** overload
  `PUT …/cursors` — its `joint_trajectory` is required, and a `oneOf` body
  compounds spec issue §2.1)
- request body `{command_routine, poses[]?, tcp?}` — `poses[]` supplies
  `local_pose` resolution without a teaching-API round trip
- handler: `compile_routine()` → `mg.plan(actions, tcp, start_joints)` →
  `_load_planned_motion()` → `TrajectoryCursor(..., actions=actions)`
- its OpenAPI must move `3.0.3 → 3.1.0`

Details of that service's rework belong in its own repo; recorded here only to
pin down what the SDK API has to make possible.

---

## 5. Open questions

1. **Where do the models come from?** Generate from the service-manager spec into
   `nova/api/` once merged, hand-write in `nova/teaching/models.py`, or vendor the
   YAML? Generation is blocked on the MR merging *and* on spec issue §2.1.
2. **How far do we support `IOExpression`?** Single condition only, or push for a
   baseline API change (§3.2).
3. **Is `WaitAction` actually executable**, or is `wait_for_time` unimplementable
   without server support? Needs a live check.
4. **Marker transport** — blinker signal only, or does the SDK also own a NATS
   publication contract?
5. **Backward-travel semantics** for overlay commands: re-fire `set_io`? re-emit
   markers? The format is silent.
6. **v1 scope.** Proposal: `inline_pose` + `joint_position` targets,
   `PathLine`/`PathCartesianPTP`/`PathJointPTP`, `set_io` + `marker`. Reject
   generated motions, direction-constrained paths, `dataset_pose`, and compound
   IO expressions with a clear error. Confirm.
7. **Does MR 2642 match `b21c51d7d`?** Verify on VPN (§0).

---

## 6. Suggested sequencing

1. Verify §0 provenance on VPN; raise §2.1 and §2.2 on the MR **before it merges**.
2. Land `feat/path-triggers-distance-offset` onto `main` (§3.4).
3. Fix `set_outputs` in the cursor (§3.1) — small, self-contained, independently
   testable, unblocks the most valuable overlay kind.
4. Decide Q1 and Q6, then build `nova/teaching/` (§4.1) against the frozen subset.
5. Markers + command-level addressing (§4.2 P1).
6. Only then rework trajectory-cursor (§4.3).

Steps 2 and 3 are useful on their own merits and carry no CommandRoutine risk.
