# Running CommandRoutines

A `CommandRoutine` is an ordered list of commands with its own context (motion group, TCP, start
joint position, default motion settings, motion group setup). You can build one in code
(`nova.command_routines`) or keep it in a Dataset. NOVA plans and runs the whole routine through
its command routines API; the SDK sends the routine as it is. `command_routine(...)` builds a `CommandRoutine`, a subclass of the generated
`api.models.CommandRoutine` that resolves, plans and executes itself on a motion group (the
command routines of a loaded `Dataset` are `CommandRoutine`s too):

| Call | Result |
| --- | --- |
| `await CommandRoutine.resolve(nova, dataset, "routine-id")` | a routine stored in a Dataset, its poses resolved to world poses by NOVA |
| `plan = await routine.plan(mg)` | the plan NOVA made for the routine: `joint_trajectory` (e.g. to draw its path) and where each command happens on it; nothing moves |
| `run = await routine.execute(mg)` or `routine.execute(mg, plan)` | starts the routine on NOVA and returns the `CommandRoutineRun` (state, current command) once NOVA has accepted it. NOVA runs the plan and never plans again, so what was drawn is what runs; without a `plan`, the routine is planned first |
| `await CommandRoutine.stop(mg, run)` | stops the run (a run or its id): the robot stops and no further outputs are written |

`execute` does not wait for the robot. NOVA runs the routine on its own; awaiting the end
of a run and following its state will come with NATS.

The routine's own context is used. `plan` and `execute` run it **on the given motion group,
whatever motion group the routine names**; the routine itself is not changed. A routine that names
another motion group is planned with the setup NOVA describes for the given one, not with the
routine's `motion_group_setup`. The methods take no arguments that override the routine's context.
`MotionGroup.plan`, `plan_and_execute` and the other action-list functions do not take a routine.

Per-command motion settings take precedence over the routine's default settings. The robot must be
at the routine's start joint position, so move it there first.

## From code

```python
from nova.command_routines import CommandRoutine, command_routine, move_joint_ptp, move_linear, set_io

routine = command_routine(
    "my-routine",
    tcp="Flange",
    motion_group_setup=await mg.get_setup("Flange"),
    start_joint_position=list(await mg.joints()),
    commands=[move_joint_ptp(target_joints), set_io("OUT#1", True), move_linear(inline_pose)],
)
await routine.execute(mg)
```

Poses in a routine you run must be inline poses in the `world` frame (`api.models.InlinePose`).

## From a Dataset

A routine stored in a Dataset on the NOVA instance refers to the Dataset's poses and frames.
Let NOVA resolve it first: local pose references and poses in a Dataset frame become inline world
poses, so the routine runs without the Dataset.

```python
from nova.command_routines import CommandRoutine

routine = await CommandRoutine.resolve(nova, "my-dataset", "my-routine")
await routine.execute(mg)
```

If the routine's TCP or motion group setup do not match your controller, replace them with
`routine.model_copy(update={"tcp": ..., "motion_group_setup": await mg.get_setup(tcp)})`.
The full flow is in [`examples/command_routine.py`](../examples/command_routine.py).

## What runs

| Scenario | Outcome |
| --- | --- |
| joint PTP, cartesian PTP, line, circle | runs |
| generated motions (collision-free, to joints or to a pose) | runs |
| `set_io` where the robot stands (written by the service) and between two motions (written by the controller) | runs |
| `wait_for_time`, `wait_for_io` (also compound conditions with `all_of`, `any_of`, `not`) | runs; a `wait_for_io` that times out fails the run, naming the command |
| `marker` (with payload) | runs |
| `at` triggers, `pause_on_io` | run |
| routine default settings, per-motion overrides | runs |
| I/O-only routine (`motion_group_setup: None`): bus IOs, a wait, a marker | runs, nothing is planned or moved |

## Not supported yet

NOVA answers these before the robot moves with HTTP 501, which the SDK raises as
`UnsupportedRoutineCommand` naming the command:

- `PathCubicSpline` and the direction-constrained path types,
- a joint PTP to a pose target, a cartesian path to a joint position target, a circle via a joint
  position,
- more than one TCP: poses taught with different TCPs, or with another TCP than the routine's.

NOVA answers these with HTTP 422, raised as `CommandRoutineError`: a local pose reference or a pose
in a Dataset frame (resolve the routine first), a motion routine without `start_joint_position`, a
target out of the workspace (the error names the command), a routine that spans more than one motion
group, and I/O-only routines with a `motion_group`, a `CONTROLLER` IO, a motion, an `at` trigger or
`pause_on_io`.

## Known limits of the client

The generated API client leaves out fields that are `None` when it sends a request. That affects
two things, which cannot be sent today:

- `motion_group_setup: None`, which declares an I/O-only routine: the field is required, and
  NOVA never sees it.
- `blending: None` in a command's motion settings, which clears the routine's default blending.

## Stopping and errors

- **Stop:** `await CommandRoutine.stop(mg, run)` with the run from `execute`. Without a stop the
  routine runs to completion.
- **Errors**, raised before the robot moves:
  - `UnsupportedRoutineCommand` and `CommandRoutineError` (`nova.command_routines`): see above.
    `UnsupportedRoutineCommand` is a `CommandRoutineError`; the message is NOVA's and names the
    command and its index.
  - `DatasetNotFoundError` / `DatasetError` from `CommandRoutine.resolve`: the Dataset or
    routine does not exist, or a pose cannot be resolved (the error names the pose).
