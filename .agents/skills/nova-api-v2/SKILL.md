---
name: nova-api-v2
description: >-
  Authoritative reference for working with the released Wandelbots NOVA API v2 in the
  wandelbots-nova Python SDK. Use whenever a task touches the NOVA API, the generated
  `wandelbots_api_client` models (`nova.api.models.*`), motion planning requests
  (`plan_trajectory`, `MotionGroupSetup`, `LimitSet`, `LimitsOverride`), robot limits, or
  torque/velocity/acceleration/jerk limits. Reach for this skill before guessing field names
  or assuming a field exists — the installed client reflects the RELEASED API, while nightly
  specs may contain unreleased fields. Especially relevant for controller-reported limits
  (e.g. KUKA torque) and any "does field X exist / how is it forwarded to the planner?" question.
---

# NOVA API v2

Domain knowledge for the **released** Wandelbots NOVA API v2 as used by the `wandelbots-nova`
SDK. The goal is to keep you from guessing field names or inventing fields that only exist in
unreleased/nightly specs.

## Source of truth (in priority order)

1. **The installed client** `wandelbots_api_client` (pinned in `pyproject.toml`, e.g.
   `wandelbots_api_client~=26.4.0`). `nova.api` lazily re-exports
   `wandelbots_api_client.v2_pydantic`; all models live in a single generated file
   `wandelbots_api_client/v2_pydantic/models/models.py`. This matches the released API exactly.
2. **Docs UI:** https://portal.wandelbots.io/docs/api/v2/ui/ — base URL `/api/v2`. Shows the
   API version (e.g. `v2.4.0`).

Prefer the installed models over any nightly/internal spec. A nightly spec may advertise fields
that are **not yet released** and therefore are not in the client the SDK ships with.

### How to verify a field exists (do this instead of guessing)

```bash
# Confirm versions
./.venv/bin/python -c "import wandelbots_api_client as c; print(c.__version__)"

# Inspect a model's fields directly
./.venv/bin/python -c "from nova import api; print(api.models.JointLimits.model_fields.keys())"
```

You can also grep the generated models file:

```bash
grep -n "^class JointLimits" .venv/lib/python*/site-packages/wandelbots_api_client/v2_pydantic/models/models.py
```

If a field is not in the installed model, treat it as **not available** in the released API,
regardless of what a spec or ticket claims.

## Limits & torque (motion planning)

This is the most common source of confusion. The released v2 API distinguishes **global**
motion-group limits from **per-segment** limit overrides, and they do NOT carry the same fields.

- The controller reports limits in `MotionGroupDescription.operation_limits`
  (`auto_limits` / `manual_limits` / `manual_t1_limits` / `manual_t2_limits`), each a `LimitSet`.
- Per-joint limits live in `LimitSet.joints[i]` (`JointLimits`):
  `position`, `velocity`, `acceleration`, `jerk`, `torque`.
- **`torque` is a global per-joint limit only.** There is no per-segment torque field.
- `MotionGroupSetup.global_limits` (sent to `plan_trajectory`) is built from `auto_limits` in
  `nova/utils/collision_setup.py::motion_group_setup_from_motion_group_description`. So any
  controller-reported `joints[i].torque` (e.g. from a KUKA controller) is **already forwarded to
  the planner unchanged** via `global_limits` — no SDK change is needed for forwarding.
- Per-segment `MotionCommand.limits_override` (`LimitsOverride`) has **no torque field** — only
  velocity / acceleration / jerk for joints and TCP. Therefore torque **cannot be overridden per
  motion**; it is exclusively a motion-group-global limit.

### Consequence for `MotionSettings`

`nova.types.motion_settings.MotionSettings` is **per-motion**. Because the API only supports
torque as a global limit, exposing a per-motion torque setting would be a footgun: it could only
take effect on the collision-free planning path (which folds settings into
`global_limits` via `update_motion_group_setup_with_motion_settings`) and would be **silently
ignored on the standard collision-checked path** (which uses `LimitsOverride`). Do not add
per-motion torque unless the released API gains a per-segment torque field.

## Planning data flow (where to look)

- Setup is built in `nova/utils/collision_setup.py::motion_group_setup_from_motion_group_description`.
- Collision-checked planning: `MotionGroup._plan_with_collision_check` → `plan_trajectory`
  (uses `LimitsOverride` per `MotionCommand`, plus `global_limits`).
- Collision-free planning: `MotionGroup._plan_collision_free` → `plan_collision_free`
  (folds `MotionSettings` into `global_limits`, not via `LimitsOverride`).
- Per-segment overrides are capped to the global maxima before planning by
  `nova/utils/motion_group_settings.py::clamp_motion_commands_to_global_limits` (this prevents the
  planner from producing a trajectory the hardware down-scales at execution time).

## Verifying behavior with a quick experiment

When unsure whether a value survives setup-building and serialization, construct the models and
serialize the request rather than reasoning about it:

```python
from nova import api
from nova.utils.collision_setup import motion_group_setup_from_motion_group_description as build

desc = api.models.MotionGroupDescription(
    motion_group_model=api.models.MotionGroupModel("UniversalRobots_UR5e"),
    operation_limits=api.models.OperationLimits(
        auto_limits=api.models.LimitSet(
            joints=[api.models.JointLimits(velocity=3.0, acceleration=10.0, torque=150.0)],
            tcp=api.models.CartesianLimits(velocity=250.0),
        )
    ),
)
setup = build(desc)
print(setup.global_limits.joints[0].torque)  # -> 150.0, i.e. forwarded
```

## Pitfalls

- Don't trust a ticket/spec that says a field exists until you've confirmed it in the installed
  model. Released ≠ nightly.
- `global_limits` (request input) vs `operation_limits` (controller-reported description) are
  different objects — the setup builder copies `auto_limits` into `global_limits`.
- Physical limit enforcement happens at execution time (the motion player time-scales the path);
  the planner treats `limits_override` as a per-segment replacement, not a clamp.
