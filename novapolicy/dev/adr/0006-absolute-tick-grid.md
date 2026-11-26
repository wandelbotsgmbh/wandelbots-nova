# 0006 The control loop ticks on an absolute grid

Status: accepted, 2026-11-19

**Context.** `ContinuousExecution(rate_hz=30)` ran at 27 Hz with the Rerun viewer and about 29 Hz
without: the loop slept for the rest of each tick, so an overrun or a late wake-up was lost for
good.

**Decision.** Sleep until the next tick on an absolute grid, as ROS `Rate.sleep`; more than one
period behind, restart the grid instead of firing catch-up ticks.

**Consequences.** 30.0 Hz with the viewer (360 of 360 steps in 12 s, against 326).
