# Running CommandRoutines

A `CommandRoutine` is an ordered list of commands with its own context (motion group, TCP, start
joint position, default motion settings, motion group setup). You can build one in code
(`nova.command_routines`) or load it from a Dataset. `MotionGroup` runs it the same way it runs
action lists:

| Call | Returns |
| --- | --- |
| `await mg.plan(routine)` | `JointTrajectory` |
| `await mg.execute(joint_trajectory, routine.tcp, routine)` | runs a planned trajectory |
| `await mg.plan_and_execute(routine)` | plans, then runs |

The routine's own context is used and cannot be overridden. Per-command motion settings take
precedence over the routine defaults. Planning only needs the start joint position, so move the
robot there first .

## From code

```python
from nova.command_routines import command_routine, move_joint_ptp, move_linear, set_io

routine = command_routine(
    "my-routine",
    motion_group=mg.id,
    tcp="Flange",
    motion_group_setup=await mg.get_setup("Flange"),
    start_joint_position=list(await mg.joints()),
    commands=[move_joint_ptp(target_joints), set_io("tool_out[0]", True), move_linear(pose_ref)],
)
await mg.plan_and_execute(routine)
```

## From a Dataset

Load the Dataset (for example with `ds.local_dataset(...)` in `@nova.program` preconditions) and
resolve the routine against it. Local pose references and poses in a Dataset frame become inline
world poses, so the calls above need no dataset:

```python
from nova.cell.routine_execution import resolve_routine

routine = ctx.dataset.command_routines["my-routine"]
await mg.plan_and_execute(resolve_routine(routine, ctx.dataset))
```

If the routine's TCP or motion group setup do not match your controller, replace them with
`routine.model_copy(update={"tcp": ..., "motion_group_setup": await mg.get_setup(tcp)})`.
The full flow, with the dataset of `examples/datasets.py`, is in
[`examples/command_routine.py`](../examples/command_routine.py). The command support list is
this page's "Supported commands" and "Not supported yet" sections.

## Supported commands

- Motions: `move_linear`, `move_cartesian_ptp`, `move_joint_ptp` (joint position target),
  `move_circular`.
- `set_io`: boolean, integer and analog outputs, controller or bus IO origin. Without `at` it
  switches at its motion boundary. With a time, distance or path-fraction trigger
  (`at_time`, `at_distance`, `at_path_fraction`) it switches at that point along the motion.

## Not supported yet

`wait_for_time`, `wait_for_io`, `pause_on_io`, `marker` and `generated_motion` have no
action-list equivalent. A routine containing one raises `UnsupportedRoutineCommand` naming the
command (for example `Command 3 (wait_for_time) is not supported yet`) during planning, before
the robot moves. Nothing is skipped silently.

## Observing state, stopping, errors

- **Observe:** run the routine as a task and read `mg.stream_state()` next to it. `state_stream_rate_msecs` sets the
  rate of the state stream used during execution.
- **Stop:** cancel the task awaiting the call. No outputs are written after a stop. Without a
  stop the routine runs to completion.
- **Errors**, all raised before the robot moves:
  - `UnsupportedRoutineCommand`: unsupported command or path type.
  - `PoseResolutionError`: a referenced pose cannot be resolved (error names the pose). Pass the
    Dataset when the routine uses local pose references.
  - `ValueError`: the routine references a different motion group.
  - Planning failures surface as the usual planning errors.
