# 0005 IO and computed actions run when the policy's first step is due

Status: accepted, 2026-11-20. Interim until NOVA's action chunks carry timestamped IO.

**Context.** A chunk's IO and computed actions fired at five different moments: at send (most
paths), when a lead chunk's first step was due, or after the loop blocked until a bridged chunk
reached its policy waypoint zero. With the lead, IO at send fired 100–250 ms before the motion
it belonged to.

**Decision.** The session takes them with the chunk (`update_chunk(when_due=...)`) and runs them
when the policy's first step is due on its clock, on every path, without blocking the loop. A
chunk replaced before it is sent runs neither (its motion never ran); `stop()` cancels those not
yet due; a hook that raises fails the session.

**Consequences.** IO keeps its place relative to the arm, one network delay late (a few ms in the
cluster). When NOVA accepts IO in action chunks, the session puts it into the request and the
delayed write goes away.
