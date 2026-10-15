# Spec: Execute CommandRoutines in the Python SDK (NDX-883)

**Parent:** NDX-883
**Status:** implemented on virtual controller; real-controller validation pending
**Codebase:** wandelbots-nova

## Problem Statement

Robot Programmers using the NOVA Python SDK can build CommandRoutines in code and keep them in Datasets, but they cannot execute them. The SDK stops at the data model. To move a robot, programmers convert routines into action lists by hand or skip CommandRoutines entirely. As a result, users say they cannot fully use Datasets and their tooling, and no further tooling can be built on CommandRoutines.

Note that NDX-844 ("Add CommandRoutine execution to the Python SDK", released in 6.0.0) delivered only the routine builders. It added no execution.

## Solution

The SDK gets a single public way to run a CommandRoutine on its motion group. The routine can come from code or from a loaded Dataset. As a first step, execution reaches **parity with action lists**: the commands that have an action-list equivalent run.

- **What runs:** explicit motions (linear, cartesian PTP, joint PTP, circular) and set_io, with or without time, distance or path-fraction triggers.
- **What is rejected:** routines containing wait_for_time, wait_for_io, pause_on_io, marker or generated_motion. They are rejected before the robot moves, with an error naming the command.
- **What gets used:** the routine's own context. Dataset pose references are resolved.
- **What the programmer can do:** observe execution state and stop the run. Without a stop, it runs to completion.
- **Where it runs:** on the virtual controller and on a real controller.
- **What doesn't change:** existing plan/execute and action-based programs behave exactly as before.

## User Stories

Each story maps to a virtual-controller integration test in `tests/nova/cell/motion_group/test_execute_routine.py`.

1. **Run a routine from code.** As a Robot Programmer, I want to run a CommandRoutine I built in code with one SDK call, using the routine's own context (motion group, TCP, start joint position, setup, default settings; per-command settings win), so that I don't convert it into an action list. Motions (line, cartesian PTP, joint PTP, circle) execute in order. Planning errors surface before the robot moves.
   Test: `TestMotionRoutine::test_motion_routine_runs_to_completion_on_virtual_controller`.
2. **Run a routine with Dataset poses.** As a Robot Programmer, I want local pose references and frames resolved against the Dataset, with a clear error naming the missing pose before the robot moves, so that taught poses drive the robot.
   Test: `TestDatasetPoses::test_dataset_pose_is_resolved_to_world_on_virtual_controller`. The missing-pose error is covered by the gateway-double tests in `tests/cell/test_execute_routine.py`.
3. **Switch outputs along the path.** As a Robot Programmer, I want set_io (boolean, integer, analog; controller or bus origin) to switch at its motion boundary or at a time, distance or path-fraction trigger, so that I get precise process timing. No outputs are written after a stop.
   Test: `TestSetIO::test_set_io_switches_output_during_run_on_virtual_controller`. Triggers and stop are covered by the gateway-double tests.
4. **Run a Dataset routine end-to-end.** As a Robot Programmer, I want an example and docs showing how to load a Dataset and run its routine with motions and set_io (with and without trigger), and which commands are not supported yet, so that I can get started quickly.
   Test: `TestExampleDatasetRoutine::test_dataset_routine_with_set_io_runs_on_virtual_controller`.
5. **Stay in control and safe.** As a Robot Programmer, I want unsupported commands (wait_for_time, wait_for_io, pause_on_io, marker, generated_motion) rejected up front with the command named, to stop a run at any time, and my existing plan/execute and action programs unchanged, so that adoption carries no risk. Stop is cancelling the awaiting task; state is observable through the motion group state stream.
   Tests: `TestUnsupportedCommands::test_unsupported_command_is_rejected_before_motion_on_virtual_controller` (wait_for_time, marker: the error names the command and says 'not supported yet', the robot does not move). Stop is covered by the gateway-double tests, plus the existing plan/execute and action suites. Real-controller run still pending.

## Implementation Decisions

- **Public entry points mirror plan / execute / plan_and_execute.** They live on `MotionGroup`: `plan_routine(routine, dataset=None)` returns a `JointTrajectory`, `execute_routine(routine, joint_trajectory, dataset=None, *, state_stream_rate_msecs=None)` runs it, and `plan_and_execute_routine(routine, dataset=None, *, state_stream_rate_msecs=None)` does both. `dataset` is an optional argument: it is required only when the routine uses local pose references or poses in a dataset frame. `execute_routine` translates the routine again to rebuild the action list, so it must get the same routine and dataset as `plan_routine`. There is no streaming variant. Stop means cancelling the task awaiting the call. Callers cannot override the routine's context through the call. To adapt a routine whose TCP or setup does not match the controller (as with the seed dataset routines), copy it with `model_copy(update=...)` first.
- **Execution reuses the action-list machinery.** The pinned API client (26.7.0) has CommandRoutine models but no endpoint for executing a routine. Because scope is parity with action lists, the SDK translates the routine internally into the existing motion and IO-write model and executes it through the existing plan/execute path. That path goes through the trajectory cursor, the single execute-trajectory implementation per ADR 001. This translation is internal and not a public conversion feature.
- **Command mapping:**
  - Explicit motions map to the equivalent action-list motions.
  - set_io maps to IO writes. Its trigger maps onto the shared AtTrigger vocabulary and the existing path-trigger resolution into `set_outputs`.
- **Validation happens before motion.** The unsupported-command check, pose resolution and planning all complete before the robot moves. wait_for_time, wait_for_io, pause_on_io, marker and generated_motion are rejected, with the error naming the command.
- **Dataset integration.** Routines come from a Dataset's command routines. Local pose references are resolved through the existing Dataset frame/pose resolution.
- **Errors.** `UnsupportedRoutineCommand` (also for inline targets with another TCP, non-joint targets for joint PTP, and other pose reference types) and `PoseResolutionError` (both `ValueError` subclasses in `nova.exceptions`) are raised during translation. A routine referencing another motion group raises `ValueError`.
- **Docs and example.** `docs/command_routines.md` and `examples/command_routine.py` (reuses `examples/example_dataset.json`, virtual KUKA, `OUT#1`) document and show the flow. The dataset's seed routine holds only `wait_for_time`, so the example filters unsupported commands and appends motions and `set_io` in code.
- **Existing code is left alone.** The routine builders from NDX-844 stay unchanged, and the existing MotionGroup plan/execute and actions APIs stay unchanged.

## Testing Decisions

- **What makes a good test.** Tests assert only external behaviour at the public entry point: which motions ran, which outputs were requested and where, stop behaviour and errors. They do not assert internal translation details.
- **Main seam.** All tests go through the single public routine-execution entry point.
  - Behaviour tests use gateway test doubles, following the existing cell tests (the trajectory cursor and trajectory executor tests).
  - Integration tests run on the virtual controller, following the existing motion group integration tests.
- **Coverage.** Each user story ships one behaviour test and one virtual-controller integration test. A recorded manual run on a real controller is still outstanding.
- **Regression.** The existing plan/execute, action and command routine builder test suites must stay green.

## Out of Scope

- Executing wait_for_time, wait_for_io, pause_on_io, marker and generated_motion. These have no action-list equivalent and are deferred to later work.
- Manual pause/resume and stepping by the programmer.
- Synchronized execution across multiple motion groups.
- Exposure via novax (program-run HTTP API).
- Rerun visualization of routine execution.
- Writing or editing routines back to a Dataset.
- Routine-to-action-list conversion as a user-facing feature.
- Running several routines in parallel or chained.
- Changes to NDX-844 or its changelog entry.
- Caller overrides of the routine's own context.

## Further Notes

- **External dependency.** ADR 001 still has pending hardware verification of the IO overlay (`set_outputs`), and routine IO depends on it. It must be confirmed before IO support is considered done.
- **Deferred commands.** Supporting the deferred commands later will need SDK-side handling, because the execution API has no native wait or marker. It will also need the open trajectory-cursor TODO about surfacing non-motion actions as events.
