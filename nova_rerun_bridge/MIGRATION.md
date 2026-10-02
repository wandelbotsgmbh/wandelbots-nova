# Migration notes

What changed in the bridge, and what a program has to do about it.

## Robots come from URDF

A robot is rendered from a URDF that `nova2urdf` derives from the DH parameters,
meshes and collision model the API serves. Rerun renders URDF natively, so a pose
costs one transform per joint instead of one per mesh, and the viewer shows named
links and joint limits.

Nothing to do for a program that uses `viewers.Rerun()`: the export happens on
first use and is cached. Two things are worth knowing:

- `nova2urdf` has to be installed. It comes with the `nova-rerun-bridge` extra.
- Point `NOVA_URDF_EXPORT_DIR` at an export to reuse one instead of exporting.

The GLB download path is gone, and with it `download-models` and the mesh-loading
internals of `RobotVisualizer` (`init_geometry`, `discover_joints`, and the other
scene-graph helpers). `nova_rerun_bridge.model_loader` still fetches a GLB from
the API if you want one.

## Collision and safety geometry

Both come out of the URDF now, each drawn see-through in its own colour from the
[NOVA design system](https://wandelbotsgmbh.github.io/nova-design-system/) tokens
(see `scene_colors.py`):

| geometry | colour |
| --- | --- |
| the model's own collision hulls | indigo |
| a tool's collision geometry | teal |
| the safety controller's volumes | violet |
| a zone the robot may not enter | red, filled |
| a zone the robot may not leave | orange, outline only |
| a collision object in a plan | grey, solid |

- `Rerun(show_collision=...)` decides whether the robot's collision hulls are
  drawn: off by default, `True` for all of them, `None` for only the links whose
  model carries no visual mesh. `show_collision_link_chain` and
  `show_collision_tool` are deprecated aliases of it.
- `Rerun(show_safety_zones=...)` and `Rerun(show_safety_link_chain=...)` work as
  before.
- The safety controller's volumes are exported per TCP; the one being driven is
  the one drawn.

## Tool meshes

`Rerun(tcp_tools={"gripper": "gripper.stl"})` exports each mesh onto that TCP's
frame in the URDF, so the tool rides the arm through a trajectory. `tool_asset`
arguments further down (`log_trajectory`, `log_tcp_pose`) are accepted and
ignored; pass the mesh to `Rerun` or to `log_motion` instead.

## Timing

`TimingMode` places a trajectory against the ones logged before it, per motion
group: `CONTINUE` (the default) after that motion group's last trajectory,
`RESET`/`OVERRIDE` at the given offset, `SYNC` at the given offset without moving
the clock, which is how several motion groups line up at one instant.
`NovaRerunBridge.continue_after_sync()` is deprecated and does nothing.

## Packaging

`nova_rerun_bridge/example_data` is no longer shipped in the wheel (12 MB of
meshes). Clone the repository if an example needs it.
