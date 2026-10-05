# Motion (MOT)

## MOT-001 · Separate plan / execute / stop timeouts
`severity: error` · `scope: nova` · `detect: static`

**Rule.** Every plan, execute and stop call is wrapped in its own timeout. Execute timeout covers
the longest legitimate motion, plan timeout the worst-case planning, stop timeout the shortest of
all.

**Detect.** `plan(`, `execute(`, `plan_and_execute(`, `stop(` awaited without `asyncio.timeout`,
`asyncio.wait_for` or an SDK timeout argument.

NOVA: the SDK has **no timeout parameter** on `plan` / `execute` / `plan_and_execute`. Wrap them
centrally (see `nova-sdk-mapping.md`, MOT-001).

---

## MOT-002 · TCP is explicit, named and validated at startup
`severity: error` · `scope: nova` · `detect: static`

**Rule.** Reference TCPs by name from configuration. Validate that every TCP used by any job exists
on the controller at startup and fail fast otherwise. No string/number literals like `tcp="3"`.

**Why.** A wrong TCP moves the flange, not the tool tip, to the target: an offset of the full tool
length.

**Detect.** `tcp="<literal>"`, `tcp=<int>`, TCP chosen by list index, missing startup validation of
TCP names (`await mg.tcp_names()`).

---

## MOT-003 · Units are explicit and never mixed
`severity: error` · `scope: robotics` · `detect: static+review`

**Rule.** Document and enforce one unit per quantity. NOVA uses **mm** for positions, **radians**
(rotation vector) for orientation and joints, **mm/s** for TCP velocity. Suffix variables (`_mm`,
`_rad`, `_deg`, `_s`, `_ms`) and convert at the boundary only.

**Detect.** `math.radians`/`degrees` scattered in job code; poses with values like `0.5` (meters?)
next to `500`; tolerances without unit suffix; velocities divided by unexplained constants.

---

## MOT-004 · Staged approach and retract around parts
`severity: warning` · `scope: robotics` · `detect: review`

**Rule.** Pick and place follow: free-space move to an approach pose above/before the target →
linear approach at reduced speed → slow linear final segment → actuate → linear retract along the
same axis → free-space move. Approach offsets are expressed relative to the target pose.

```python
from nova.actions import cartesian_ptp, joint_ptp, linear
from nova.types import Pose

actions = [
    joint_ptp(home, settings=FAST),
    cartesian_ptp(target @ Pose((0, 0, -150, 0, 0, 0)), settings=FAST),  # approach
    linear(target @ Pose((0, 0, -20, 0, 0, 0)), settings=NORMAL),
    linear(target, settings=SLOW),  # contact
]
retract = [
    linear(target @ Pose((0, 0, -150, 0, 0, 0)), settings=NORMAL),
    joint_ptp(home, settings=FAST),
]
```

---

## MOT-005 · Linear moves near parts, joint/PTP moves in free space
`severity: warning` · `scope: robotics` · `detect: review`

**Rule.** Joint/PTP paths are not straight in Cartesian space. Within the approach distance of a
part, fixture or other obstacle use linear motion only.

**Detect.** `ptp` / `cartesian_ptp` / `joint_ptp` directly to a pick/place pose without a preceding
approach pose.

NOVA: in cluttered free space prefer `collision_free(...)` with a collision setup over hand-placed
via-points.

---

## MOT-006 · Speeds and accelerations come from named profiles
`severity: warning` · `scope: robotics` · `detect: static`

**Rule.** Define named profiles (`FAST`, `NORMAL`, `SLOW`, `WITH_PART`, `CONTACT`) centrally. Jobs
reference profiles, never numbers.

**Detect.** `MotionSettings(` / `tcp_velocity_limit=` / `velocity=` with a numeric literal outside
the config module.

---

## MOT-007 · Poses come from a dataset or frame system
`severity: warning` · `scope: robotics` · `detect: static`

**Rule.** Taught positions live in a dataset (named waypoints) or a frame hierarchy (fixture frame +
relative offsets). Job code must not contain absolute poses. Prefer frames, so that re-teaching one
fixture moves all dependent poses.

**Detect.** `Pose((` with 3 non-zero literal translation values in job modules (relative offsets
like `Pose((0, 0, -100, 0, 0, 0))` are fine); literal joint arrays in `joint_ptp(...)`.

NOVA: `nova.datasets` (`ProgramPreconditions(dataset=...)`, `ctx.dataset`, `DatasetPose.as_world()`).

---

## MOT-008 · Pose-offset frame is explicit
`severity: warning` · `scope: nova` · `detect: review`

**Rule.** `target @ offset` applies the offset in the **target/tool** frame; `offset @ target`
applies it in the **base/reference** frame. Choose deliberately and name helpers accordingly
(`offset_in_tool`, `offset_in_base`). Getting this wrong tilts approach directions when the target
is rotated.

---

## MOT-009 · Motion group is selected explicitly
`severity: warning` · `scope: nova` · `detect: static`

**Rule.** Select the motion group by configured identifier and validate it. `controller[0]`
silently picks the wrong group on multi-group controllers (external axes, positioners, second
arm).

NOVA: `controller.motion_group("0@<controller>")`; validate against
`await controller.motion_groups()`.

---

## MOT-010 · Blending is intentional
`severity: warning` · `scope: robotics` · `detect: review`

**Rule.** Use blending in free-space via-points to keep the robot moving. Use zero blending at
contact, pick/place and measurement poses. A single global blending value for all moves is a
smell.

**Why.** Zero blending everywhere adds a full stop per via-point. Blending at a contact pose cuts
the corner and misses the part.

NOVA: `MotionSettings(blending=api.models.BlendingPosition(...) | api.models.BlendingAuto(...))`.
`blending_radius` / `blending_auto` are deprecated. Blending is not supported for
`collision_free` motions.

---

## MOT-011 · Payload is configured, not just documented
`severity: warning` · `scope: robotics` · `detect: static+review`

**Rule.** Tool and part masses and centres of gravity must be configured on the controller/planner
and switched when a part is picked. A payload documented only in a comment does not affect the
motion.

**Detect.** Comments containing `kg`, `mass`, `payload`, `CoG` with no corresponding API call.

NOVA: `plan(..., payload_override=...)`, `mg.payloads()`, `mg.active_payload_name()`.

---

## MOT-012 · Singularity strategy is explicit where needed
`severity: info` · `scope: nova` · `detect: static`

**Rule.** Cells with wrist-flip-prone geometry (palletizing, top-down picking) set an explicit
singularity handling strategy on plan calls, and pin the minimum SDK version that supports it.

NOVA: `plan(..., singularity_handling=api.models.SingularityHandling.PALLETIZING_WRIST)`
(experimental).

---

## MOT-013 · Robot limits are read from the controller
`severity: info` · `scope: nova` · `detect: static`

**Rule.** Read maximum joint velocities/accelerations from the motion group description; derive
profiles as fractions of them. Do not hardcode limits copied from a datasheet.

NOVA: `(await mg.get_description()).operation_limits.auto_limits` (`LimitSet` with `joints`,
`tcp`, …).
