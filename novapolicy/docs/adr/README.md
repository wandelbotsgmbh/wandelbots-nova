# Decision records

Why `PolicyExecutor` works the way the [contracts](../contracts.md) describe. One decision per
file: context, decision, consequences, and the alternative that was measured against it. A
superseded record stays, marked as such, so the history is kept. Measurements are on a virtual
UR10e, with the client inside the NOVA cluster unless a record says otherwise.

| # | Decision |
| --- | --- |
| [0001](0001-map-and-transport.md) | The executor maps and transports; NOVA owns the motion |
| [0002](0002-fixed-lead.md) | A fixed lead, and a loud stop when it is exceeded |
| [0003](0003-one-placement-rule.md) | One placement rule for both modes |
| [0004](0004-drop-late-steps.md) | Late steps are dropped, never retimed or smoothed |
| [0005](0005-side-effects-at-first-step.md) | IO and computed actions run when the policy's first step is due |
| [0006](0006-absolute-tick-grid.md) | The control loop ticks on an absolute grid |
| [0007](0007-freeze-setup-objects.md) | Setup objects are frozen out of garbage collection |
| [0008](0008-typed-session-clock.md) | Session-clock timestamps are typed |
| [0009](0009-opt-in-pre-send-modules.md) | One send pipeline of opt-in modules (defaults superseded by 0010) |
| [0010](0010-built-ins-are-modules.md) | The built-ins are pipeline modules, and the list is everything |
