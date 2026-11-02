# Running CommandRoutines

A `CommandRoutine` is an ordered list of commands with its own context (motion group, TCP, start
joint position, default motion settings, motion group setup). You can build one in code
(`nova.command_routines`) or keep it in a Dataset. NOVA plans and runs the whole routine through
its command routine execution API, which `nova.command_routines` calls for you:

| Call | Returns |
| --- | --- |
| `await cr.plan(nova, routine)` | the `PlannedCommandRoutine` with the trajectory NOVA planned, e.g. to draw its path; nothing moves |
| `await cr.execute(nova, routine)` | runs the routine and returns the completed run |

The routine runs with its own context on the motion group it references (`motion_group`). Pass
`controller=...` to run it on the motion group with the same index on another controller.
Per-command motion settings take precedence over the routine defaults. The robot must be at the
routine's start joint position, so move it there first.

## From code

```python
from nova import command_routines as cr
from nova.command_routines import command_routine, move_joint_ptp, move_linear, set_io

routine = command_routine(
    "my-routine",
    motion_group=mg.id,
    tcp="Flange",
    motion_group_setup=await mg.get_setup("Flange"),
    start_joint_position=list(await mg.joints()),
    commands=[move_joint_ptp(target_joints), set_io("tool_out[0]", True), move_linear(inline_pose)],
)
await cr.execute(nova, routine)
```

Poses in a routine you run must be inline poses in the `world` frame (`api.models.InlinePose`).

## From a Dataset

A routine stored in a Dataset on the NOVA instance refers to the Dataset's poses and frames.
Let NOVA resolve it first: local pose references and poses in a Dataset frame become inline world
poses, so the routine runs without the Dataset.

```python
from nova import command_routines as cr
from nova import datasets as ds

routine = await ds.resolve_command_routine(nova, "my-dataset", "my-routine")
await cr.execute(nova, routine)
```

If the routine's TCP or motion group setup do not match your controller, replace them with
`routine.model_copy(update={"tcp": ..., "motion_group_setup": await mg.get_setup(tcp)})`.
The full flow is in [`examples/command_routine.py`](../examples/command_routine.py).

## Supported commands

NOVA runs every command as authored, at its place in the sequence or at its `at` trigger:

- Motions: `move_linear`, `move_cartesian_ptp`, `move_joint_ptp` (joint position target),
  `move_circular`.
- `set_io`: boolean, integer and analog outputs, controller or bus IO origin. Without `at` it
  switches at its motion boundary. With a time, distance or path-fraction trigger
  (`at_time`, `at_distance`, `at_path_fraction`) it switches at that point along the motion.
- `wait_for_time`, `wait_for_io`, `pause_on_io` and `marker`.

## Not supported yet

NOVA rejects these before the robot moves with `UnsupportedRoutineCommand`, naming the command
and its index (for example `Command 0 (motion_command): a joint PTP to a pose target is not
implemented yet`):

- spline and direction-constrained path types,
- a joint PTP to a pose target, a cartesian path to a joint position target, a circle via a
  joint position,
- more than one TCP: poses taught with different TCPs, or with another TCP than the routine's.

## Observing state, stopping, errors

- **Observe:** run the routine as a task and read `mg.stream_state()` next to it.
- **Stop:** cancel the task awaiting `cr.execute`. The SDK stops the run on NOVA; no outputs are
  written after the stop. Without a stop the routine runs to completion.
- **Errors** (all in `nova.command_routines`), raised before the robot moves unless noted:
  - `UnsupportedRoutineCommand`: a feature NOVA cannot execute yet (see above).
  - `CommandRoutineError`: NOVA rejects the routine, e.g. an unresolved pose, an unreachable
    target, a busy motion group or a robot not at the start joint position. The message names the
    command. It is the base class of the other two.
  - `RoutineRunFailed`, raised during the run: it failed (for example a `wait_for_io` timed out)
    or was stopped from elsewhere. `error.run` holds its final state.
  - `DatasetNotFoundError` / `DatasetError` from `ds.resolve_command_routine`: the Dataset or
    routine does not exist, or a pose cannot be resolved (the error names the pose).
