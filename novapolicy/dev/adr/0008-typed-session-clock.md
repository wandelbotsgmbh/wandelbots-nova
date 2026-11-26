# 0008 Session-clock timestamps are typed

Status: accepted, 2026-11-20

**Context.** Every timing bug on this branch mixed clocks: client monotonic time, the session
clock, state timestamps and placeholder timestamps were all plain numbers.

**Decision.** `ServerMs` (a `NewType` of `int` in `transport/clock.py`) marks session-clock values
in the session's timing API, placement and the executor's observation times; `ty` rejects a value
from another clock or a bare `int` there. Arithmetic stays `int`; a value enters the session clock
in one visible `ServerMs(...)`. Public `ActionChunk` fields stay plain `int`; `first_timestamp_ms` names its clock.

**Consequences.** No runtime cost. A probe confirmed `ty` flags a `MonotonicS` or a literal passed
where `ServerMs` is expected.
