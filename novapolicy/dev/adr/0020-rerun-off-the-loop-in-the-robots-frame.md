# 0020 Rerun is written off the control loop, stamped when produced, drawn in the robot's frame

Status: accepted. Recorded 2026-11-23 from the code (`rerun/`); the decisions are older. What is
logged and how to read it is in [rerun.md](../rerun.md).

**Context.** The viewer is optional and must cost a run nothing when it is off and little when it
is on. One tick's worth of tracking and action-chunk entries costs a meaningful fraction of a 10 ms
control tick, occasionally a multiple of one; paid on a jogging loop that also produces the next
chunk, that lengthened ticks past their budget and cost motion smoothness (UR10e jogging run). The
overlays (TCP trails, action chunks, tracking error) are drawn from DH kinematics next to a robot
rendered from its URDF, and both have to agree on where the robot is.

**Decision.**

- **Writes run on a worker thread** (`AsyncLogSink`). The Rerun SDK serialises in Rust and releases
  the GIL, so the cost leaves the producing thread. The queue holds 512 entries, several seconds at
  about 90 entries/s, and drops its oldest on overflow: showing the present matters more than
  showing everything. Close sets a stop flag rather than relying on a sentinel, since a full queue
  may have no room for one; the worker drains, then stops. A write submitted after close runs
  inline, under a lock, because the worker may still be mid-write on the same trails. A failed
  entry is skipped; it never stops the writer or the loop that submitted it, which is why the
  tracking calls also guard their snapshot on the caller's thread.
- **Entries carry the time they were produced.** The producer pins its elapsed time thread-locally
  (`logtime.py`) and the writer stamps with it, so a backlog does not stretch the timeline. Rerun's
  built-in `log_time` is set from the same instant: written by the worker, a burst of state packets
  would pile into a vertical step on it. An instant from before the session start (a
  zero-initialised value is machine boot on the monotonic clock) is stamped "now" instead of
  dragging the axis days back. Plots show `policy_time`; left unset, a view falls back to the
  selected timeline, in practice `log_time`.
- **Position and orientation error get a view each.** A time-series view shares one Y axis;
  position error runs to tens of millimetres, orientation error to thousandths of a radian, which
  plotted together reads as "no error" whatever its value. Each view lists its norm series
  explicitly, since the norms are siblings of the component groups, and the norm is the series that
  shows a constant lag as constant.
- **State is logged per packet, not per control tick** (`add_state_observer`). States arrive in
  bursts, and sampling the cached one from the control loop re-reads one packet for tens of
  milliseconds and drops the rest, which draws the tracking error as flat shelves. The commanded
  side is what was sent for that packet's own server timestamp (`commanded_at`), so both sides of
  the error are the same instant. The robot mesh moves at most once per sample period of
  generation time, so it moves evenly however packets arrived.
- **A dedicated recording** (`novapolicy`) holds the policy data and its blueprint, so NOVA planning
  keeps its own. The program's own viewer settings decide how the robot is drawn
  (`show_collision`), so a policy run renders like the rest of the program.
- **One visualizer per controller.** A robot is one kinematic chain even where the API splits it
  into motion groups, and an arm cannot be placed without the part it rides on; not one per
  recording either, since a cell can hold several robots (two UR5e on separate controllers is the
  common case). The URDF is the only rendering path; when it cannot be had, the run continues
  without the robot mesh and the overlays still draw.
- **Overlays live in the robot's frame.** A URDF robot sits in Rerun's frame graph, and the 3D view
  drops anything with no path to its root, so each overlay entity declares its own robot's frame
  (on the entity itself: a frame on the parent path does nothing for its children). Each motion
  group's DH table starts at its own base and the controller reports `mounting = [0, 0, 0]`, so the
  overlay `DHRobot`s are re-based where the visualizer places each chain, and reported poses, which
  are relative to the chain's mount, are moved into the robot's frame. Without either, trails and
  markers land about a metre from the TCP they belong to.
- **The commanded TCP is the driven one.** Measured poses come back at the TCP the caller drives,
  so a joint target's TCP position uses that TCP's offset, not the controller's active one; a
  different offset shows a constant tool-sized error that reads as the policy being wrong. Unless
  the logger is asked for a TCP offset (`use_tcp_offset_for_joint_actions`), joint chunks are drawn
  at the flange: a schema with joint actions declares no TCP, and silently applying the active
  one's offset can displace the marker from what is sent.
- **The overlays' kinematics stay in `novapolicy`**, built from the public `DHRobot` API, so
  `nova_rerun_bridge` is untouched.

**Consequences.** Logging costs the control loop a queue put. Under overload the viewer shows the
present and drops history, counted at debug level on close. The sink is drained before the
recording disconnects, so the tail of a run is kept.
