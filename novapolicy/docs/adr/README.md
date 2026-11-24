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
| [0011](0011-providers-own-their-wire.md) | Providers own their wire protocol; the package owns the codec and NOVA |
| [0012](0012-session-clock-estimate.md) | The session clock is estimated from generation time and never rescaled |
| [0013](0013-live-buffer-replays-measured-targets.md) | The live buffer replays measured targets on a fixed grid |
| [0014](0014-providers-fail-before-motion.md) | Providers check the policy before the robot moves, and fail loudly |
| [0015](0015-clamp-joint-targets.md) | Joint targets are clamped inside the robot's limits |
| [0016](0016-measured-joint-velocities.md) | Joint velocities are read from the raw state stream |
| [0017](0017-playback-clock-for-one-step-chunks.md) | One-step chunks are spaced by a playback clock |
| [0018](0018-liveness-bounds-on-inputs.md) | Stale inputs end the run, with bounds that catch a dead input, not a slow one |
| [0019](0019-session-stream-and-stop.md) | The session sends without waiting for acknowledgements and ends with a stop request |
| [0020](0020-rerun-off-the-loop-in-the-robots-frame.md) | Rerun is written off the control loop, stamped when produced, drawn in the robot's frame |
| [0021](0021-one-step-targets-on-the-cycle-grid.md) | One-step joint targets can be put on NOVA's cycle grid (opt-in) |
