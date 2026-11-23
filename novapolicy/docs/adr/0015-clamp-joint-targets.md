# 0015 Joint targets are clamped inside the robot's limits

Status: accepted. Recorded 2026-11-23 from the code (`joint_limits.py`); the decision is older.
The contract is M7; the module is `JointClamp`.

**Context.** NOVA enforces joint limits by pausing the motion: one out-of-range waypoint ends the
run with `joint_limit (joints: [n])`. A policy that occasionally asks for slightly more than a
joint has is fatal, and a residual policy asks for exactly that whenever the robot is not quite
where its reference expects. The reference deployment stack for these policies hands the real
joint ranges to the policy core, which clips every target, so an impossible request becomes a
saturated one.

**Decision.** Clamp on the robot side, where the limits are known: every joint target is held
`margin_rad` inside the bounds. Each joint's first clip is logged: a joint that saturates is out
of distribution or mis-mounted. The limits come from the motion group description's
`operation_limits.auto_limits.joints`, the only place NOVA exposes them (`get_setup` reports no
position limits; `operation_limits` has no `joint_limits` field).

**Margin measured.** The pause checks the measured position, so the margin has to absorb the
robot overshooting its target. On a manus elbow, a target 0.5° inside the bound was overshot by
0.48° and tripped the limit by 0.02°. 2° (`DEFAULT_MARGIN_RAD`) leaves room, at the cost of that
much travel at each end. Overshoot depends on the controller and the approach speed: re-tune per
robot; a margin too small shows as the same `joint_limit` pause.

**Consequences.** Degraded cases send unclamped, as before clamping existed, and never stop a run
from starting: a robot that reports no limits, a failure to read them, or one unreported or
degenerate bound (the whole set is dropped rather than clamping some joints against guesses). A
step whose width disagrees with the limits passes untouched: the width is checked upstream, and
guessing an alignment would command the wrong joints. TCP waypoints are not clamped; their limit
is the workspace.
