# 0007 Setup objects are frozen out of garbage collection

Status: accepted, 2026-11-20

**Context.** About every 7 s a full collection traversed every object set up for the run and
stopped the event loop for 135–240 ms, the policy loop and the state stream together. A chunk
policy lost its place on the timeline each time.

**Decision.** After the policy is prepared, `gc.collect()` then `gc.freeze()`; `gc.unfreeze()`
when the run ends. The collector stays on.

**Consequences.** The longest gap between policy calls was 36–43 ms over 16 s and 83 ms once
over 150 s. `gc.unfreeze()` also releases what the interpreter or a host froze; only a
pre-forking server that freezes for copy-on-write would notice.
