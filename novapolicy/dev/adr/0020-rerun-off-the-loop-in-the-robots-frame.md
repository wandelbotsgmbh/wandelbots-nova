# 0020 Rerun is written off the control loop, stamped when produced, drawn in the robot's frame

Status: accepted. Recorded 2026-11-23 from the code (`rerun/`); the decisions are older. What is
logged and how to read it is in [rerun.md](../../docs/rerun.md).

**Context.** The viewer is optional and must cost a run nothing when it is off and little when it
is on. One tick's worth of tracking and action-chunk entries costs a meaningful fraction of a 10 ms
control tick, occasionally a multiple of one; paid on a jogging loop that also produces the next
chunk, that lengthened ticks past their budget and cost motion smoothness (UR10e jogging run). The
overlays (TCP trails, action chunks, tracking error) are drawn from DH kinematics next to a robot
rendered from its URDF, and both have to agree on where the robot is.

**Decision.**

- **Writes run on a worker thread** (`AsyncLogSink`), from a bounded queue that drops its oldest
  entries on overflow: showing the present matters more than showing everything, and a failed
  entry never stops the loop that submitted it.
- **Entries carry the time they were produced**, so a backlog does not stretch the timeline.
- **State is logged per state packet, not per control tick**, with the commanded side taken at
  that packet's own server timestamp, so both sides of the tracking error are the same instant.
- **Overlays are drawn in the robot's frame**, from the visualizer's URDF placement, one
  visualizer per controller; their kinematics stay in `novapolicy`, so `nova_rerun_bridge` is
  untouched.

How each of these is done, and the pitfalls each avoids, is in [rerun.md](../../docs/rerun.md).

**Consequences.** Logging costs the control loop a queue put. Under overload the viewer shows the
present and drops history, counted at debug level on close. The sink is drained before the
recording disconnects, so the tail of a run is kept.
