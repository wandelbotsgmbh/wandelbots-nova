# PLC / IO (IO)

## IO-001 · Handshakes are full request/ack cycles
`severity: error` · `scope: robotics` · `detect: static+review`

**Rule.** Use four-phase handshakes: set request → wait for ack high (timeout) → reset request →
wait for ack low (timeout). The same applies to device control words (e.g. gripper control word →
status bit).

```python
import asyncio
from nova.utils.io import set_bus_io_value, wait_for_bus_io


async def handshake(req: str, ack: str, timeout_s: float = 5.0) -> None:
    await set_bus_io_value({req: True})
    async with asyncio.timeout(timeout_s):
        await wait_for_bus_io([ack], on_change=lambda c: c[ack].new_value is True)
    await set_bus_io_value({req: False})
    async with asyncio.timeout(timeout_s):
        await wait_for_bus_io([ack], on_change=lambda c: c[ack].new_value is False)
```

**Observed.** Skipping the second half caused the gripper to ignore new parameters (plausibility
error codes).

---

## IO-002 · Every IO wait has a timeout and a predicate
`severity: error` · `scope: robotics` · `detect: static`

**Rule.** No unbounded wait on an IO signal. Each wait has a timeout that raises a descriptive error
naming the signal and the expected value. The timeout must not be optional, and it must not default
to 0 (meaning "infinite").

**Detect.** `wait_for_bus_io(` / `wait_for_bool_io(` without an enclosing timeout; `while not
read(...)` loops without deadline; timeout parameters with `0`/`None` defaults.

NOVA: neither `nova.utils.io.wait_for_bus_io` nor `wait_for_bool_io` has a timeout parameter.
Wrap them (see `nova-sdk-mapping.md`, IO-002).

---

## IO-003 · Read related signals atomically
`severity: warning` · `scope: robotics` · `detect: static`

**Rule.** When a decision depends on several signals, read them in **one** call and evaluate
against that snapshot. Sequential single-signal reads can mix states from different PLC cycles.

NOVA: `nova.utils.io.get_bus_io_value([a, b, c])` instead of several `controller.read(...)`.

---

## IO-004 · Validate the IO map at startup
`severity: warning` · `scope: robotics` · `detect: review`

**Rule.** At startup, verify that every signal the code references exists in the configured IO map
with the expected direction and type. Fail fast with a list of missing signals.

---

## IO-005 · Signal names come from generated symbols
`severity: warning` · `scope: robotics` · `detect: static`

**Rule.** Generate a symbol module from the PLC tag export (TIA/UDT/XML) and reference signals via
those symbols. String literals for signal names are typos that only surface at runtime. Regenerate
as a build step.

**Detect.** String literals matching `^(In|Out)\.` (or the project's prefix) outside the generated
module and config; string literals as the IO key of `io_write`, `read`, `write`,
`wait_for_bool_io`, `get_bus_io_value`, `set_bus_io_value`, `wait_for_bus_io`.

---

## IO-006 · Deduplicate triggers and job selection
`severity: warning` · `scope: robotics` · `detect: review`

**Rule.** When job selection is level-based (a "ready" flag stays high), the dispatcher must not
enqueue the same job again while it is pending or running. Duplicates are rejected visibly
(logged), not dropped silently.

---

## IO-007 · Consistent signal naming convention
`severity: info` · `scope: robotics` · `detect: static`

**Rule.** Example convention: `<Dir>.<Domain>[.<Station>].<Signal>`, with `Dir ∈ {In, Out}` from the
robot's view, `Domain ∈ {Sys, Grp, App}`, PascalCase signals and semantic suffixes (`Rdy`, `Act`,
`Done`, `HS`, `Ok`).
