# Spec: Execute CommandRoutines on a MotionGroup

**Parent:** NDX-883
**Seam:** `plan(routine)` and `plan_and_execute(routine)` of a robot (`AbstractRobot`, used through `MotionGroup`), which take a `CommandRoutine` wherever they take an action list, and `MotionGroup.stop_routine(run)`
**Status:** implemented with unit tests (API double); runs on a NOVA instance and a real controller pending (instance unreachable at the time of writing)
**Codebase:** wandelbots-nova
**Related spec:** [Resolve a CommandRoutine of a Dataset](../pose-resolution/spec.md) (`nova.datasets.resolve_command_routine`), which turns Dataset routines into routines this seam runs

## Problem Statement

Robot Programmers using the NOVA Python SDK can build CommandRoutines in code and keep them in Datasets, but they cannot execute them. The SDK stops at the data model. To move a robot, programmers convert routines into action lists by hand, which loses the routine's own context, or they skip CommandRoutines entirely. As a result they cannot fully use Datasets and their tooling, and no further tooling can be built on CommandRoutines.

NDX-844 ("Add CommandRoutine execution to the Python SDK", released in 6.0.0) delivered only the routine builders. It added no execution.

## Solution

NOVA's command routines API plans and runs a whole CommandRoutine. The SDK sends the routine to it as it is, without converting it into an action list. `MotionGroup` decides by what it receives: an action list takes the existing path, a CommandRoutine goes to the API, on this motion group.

- **What runs:** everything NOVA runs: explicit motions (joint PTP, cartesian PTP, line, circle), generated (collision-free) motions, set_io, wait_for_time, wait_for_io (also compound conditions), `at` triggers, pause_on_io, markers, and I/O-only routines.
- **What is rejected:** what NOVA does not implement yet (cubic splines, direction-constrained paths, a joint PTP to a pose, a cartesian path to a joint position, a circle via a joint position, more than one TCP), before the robot moves, with an error naming the command; and what is invalid (unresolved poses, a missing start joint position, an unreachable target).
- **What gets used:** the routine's own context. Routines with Dataset pose references are resolved first, see the related spec.
- **Where it runs:** on the motion group the call is made on, whatever motion group the routine names.
- **What the programmer can do:** plan a routine, start it (and get its run), stop the run. Following a run until it ends is not part of this step; it will be built on NATS.
- **What doesn't change:** existing plan/execute and action-list programs behave exactly as before.

## Glossary

- **Routine Context:** the values a CommandRoutine carries itself: motion group reference, TCP, start joint position, default motion settings and motion group setup.
- **Routine Run:** one execution of a routine on NOVA, with a state (`PREPARING`, `RUNNING`, `COMPLETED`, `FAILED`, `STOPPED`), the command it executes or waits on and, if it failed, the error.
- **Execution State:** the observable status of a running routine: moving, waiting (with reason), stopped, completed or failed.

## Requirements

Tags: **[Deferred]** not implemented. **[NOVA]** done by NOVA; the SDK passes the routine through and has no test of its own for it beyond what is sent. **[Pending]** implemented, verification on a NOVA instance or real controller outstanding. Untagged criteria are implemented and covered by the tests named in the Examples.

### R1 Run a routine with its own context

**User Story:** As a Robot Programmer, I want to run a CommandRoutine with one SDK call using the routine's own context, so that I don't convert it into an action list.

- **R1.1** WHEN a CommandRoutine is passed to `plan_and_execute`, THE MotionGroup SHALL start a Routine Run of the whole routine on NOVA and return once NOVA has accepted the run. **[Pending]** end-to-end on a NOVA instance.
- **R1.2** THE MotionGroup SHALL send the routine to NOVA unchanged, with itself as the motion group to run on, whatever motion group the routine references.
- **R1.3** THE MotionGroup SHALL NOT accept arguments that would override the Routine Context (start joint position, motion group setup, payload, singularity handling) or are execution arguments of action lists (`movement_controller`, `start_on_io`, `pause_on_io`, `state_stream_rate_msecs`); it raises `ValueError` naming them. `tcp` may only repeat the routine's TCP.
- **R1.4** WHEN a CommandRoutine is passed to `plan`, THE MotionGroup SHALL return the joint trajectory NOVA planned for it on this motion group, without moving the robot. An I/O-only routine has no trajectory (`ValueError`).
- **R1.5** WHERE a motion command defines its own motion settings, NOVA SHALL apply them instead of the routine's defaults (speed overrides included). **[NOVA]**
- **R1.6** THE MotionGroup SHALL run routines built with the existing routine builders and routines resolved from a Dataset in the same way.
- **R1.7** `MotionGroup.execute`, `stream_execute` and `stream_plan_and_execute` SHALL NOT take a routine: they raise `ValueError` naming `plan_and_execute`.
- **R1.8** `plan_and_execute` SHALL return the started Routine Run for a routine and `None` for an action list.

### R2 Move along every supported path type

**User Story:** As a Robot Programmer, I want every motion command in a routine to execute, so that the robot path matches the authored routine.

- **R2.1** WHEN a routine contains joint PTP, cartesian PTP, line, circle or generated motions, NOVA SHALL move the robot along them. **[NOVA]** **[Pending]**

### R3 Switch outputs

**User Story:** As a Robot Programmer, I want set_io commands to switch outputs at the place the routine defines, so that I get precise process timing.

- **R3.1** WHEN a set_io command is where the robot stands, NOVA SHALL write it; WHEN it is between two motions, the controller SHALL write it on the way. Boolean, integer and analog values and the IO origin (controller or bus) are honoured. **[NOVA]** **[Pending]**

### R4 Reject what cannot run, before motion

**User Story:** As a Robot Programmer, I want routines that cannot run rejected before anything moves, so that adoption carries no risk.

- **R4.1** IF NOVA does not implement a feature of the routine yet (HTTP 501), THEN THE SDK SHALL raise `UnsupportedRoutineCommand` with NOVA's message naming the command and its index. Currently: a cubic spline, a direction-constrained path, a joint PTP to a pose, a cartesian path to a joint position, a circle via a joint position, a pose of another TCP.
- **R4.2** IF NOVA rejects the routine for another reason (HTTP 422, 404, 409: an unresolved pose, a missing start joint position, an unreachable target, an I/O-only routine with a motion group, a controller IO, a motion, an `at` trigger or pause_on_io, a busy motion group, a robot not at the start joint position), THEN THE SDK SHALL raise `CommandRoutineError` with NOVA's message.
- **R4.3** `UnsupportedRoutineCommand` SHALL be a `CommandRoutineError`.

### R5 Wait, markers and I/O-only routines **[NOVA]**

**User Story:** As a Robot Programmer, I want routines to wait for time and IO and to mark points, so that the robot synchronizes with processes and peripherals.

- **R5.1** WHEN execution reaches a wait_for_time command, the run SHALL hold for the command's duration. **[Pending]**
- **R5.2** WHEN execution reaches a wait_for_io command, the run SHALL hold until its IO expression (also all_of, any_of, not) is true. **[Pending]**
- **R5.3** IF a wait_for_io command's timeout elapses first, THEN the run SHALL fail naming the command. **[Pending]**
- **R5.4** A routine without a motion group setup SHALL run without planning or moving a robot. **[Pending]**

### R6 Stop

**User Story:** As a Robot Programmer, I want to stop a running routine, so that I stay in control.

- **R6.1** WHEN `stop_routine` is called with a run or its id, THE MotionGroup SHALL ask NOVA to stop the run; NOVA stops the robot and writes no further outputs of it. NOVA's rejections are raised as for a start.
- **R6.2** **[Deferred]** WHEN the task awaiting `plan_and_execute` is cancelled, THE MotionGroup SHALL stop the run. It returns once NOVA accepted the run, so there is nothing to cancel yet.
- **R6.3** **[Deferred]** THE SDK SHALL let the programmer await the end of a run and follow its Execution State and markers, built on NATS (`{instance}.v2.cells.{cell}.routines.runs.{run}`), not on polling.

### R7 Compatibility

- **R7.1** THE SDK SHALL keep the behaviour of the existing MotionGroup plan/execute and action-list APIs unchanged, verified by their existing test suites.
- **R7.2** A routine SHALL run the same on the virtual controller as on a real controller. **[Pending]** Not yet verified.

## Examples

| ID | Given | When | Then | Test |
| --- | --- | --- | --- | --- |
| E1 | A routine written for `0@other` | `plan_and_execute(routine)` on `0@ctrl` | NOVA is asked to run it on `0@ctrl`; the routine is sent as authored; the started run is returned and not waited for | `TestPlanAndExecute` (unit) |
| E2 | A routine written for `0@other` | `plan(routine)` on `0@ctrl` | NOVA is asked to plan it for `0@ctrl`; nothing is started | `TestPlan` (unit) |
| E3 | A routine on a motion group other than the routine's | it runs | the robot of the motion group the call was made on moves, the named one does not | `TestOtherMotionGroup` (integration) |
| E4 | A routine with default settings (speed 100), a motion with speed 25 and a motion without settings | it runs | NOVA receives exactly these settings | `TestPlanAndExecute`, `TestMotionSettings` (unit) |
| E5 | Each feature scenario of NOVA's routines service (motion types, set_io, wait_for_time, wait_for_io, expressions, markers, spline, generated motions, settings, I/O-only) | it is planned and started | NOVA receives the routine unchanged | `TestScenarios` (unit) |
| E6 | The same scenarios on a virtual controller | they run | each run ends `COMPLETED` (04 once its input is true; 03 is seen waiting) | `TestFeatures` (integration) |
| E7 | Each error scenario | it is planned or started | NOVA's answer (422, 501) is raised as `CommandRoutineError` / `UnsupportedRoutineCommand` naming the command; the robot does not move | `TestScenarios` (unit), `TestErrors` (integration) |
| E8 | A wait_for_io on an input that never becomes true, timeout 2 s | it runs | the start is accepted and the run ends `FAILED` naming the command | `TestErrors` (integration) |
| E9 | A run | `stop_routine(run)` or `stop_routine(run id)` | NOVA is asked to stop that run; a run integration test sees it `STOPPED` | `TestStopRoutine` (unit), `TestFeatures` (integration) |
| E10 | `start_joint_position`, `motion_group_setup`, another TCP, `movement_controller` or `state_stream_rate_msecs` passed with a routine | it is planned or started | `ValueError` naming the argument; NOVA is not called | `TestRejection` (unit) |
| E11 | A routine | `execute`, `stream_execute` or `stream_plan_and_execute` | `ValueError` naming `plan_and_execute` | `TestRejection` (unit) |
| E12 | An action list | `plan` or `plan_and_execute` | the existing path is used; NOVA's routines API is not called | `TestPlan` (unit) |
| E13 | A routine | `plan` or `plan_and_execute` of a simulated robot | `NotImplementedError` says it cannot plan / run routines | `TestRobotsWithoutNova` (unit) |

## Implementation Decisions

- **The robot branches on its input.** `AbstractRobot.plan` and `plan_and_execute` accept a `CommandRoutine` next to action lists: a routine is checked (arguments it cannot take raise `ValueError`) and handed to the abstract hooks `_plan_routine` and `_start_routine`; everything else takes the existing path, unchanged. `MotionGroup` implements the hooks by calling NOVA's command routines API (`planCommandRoutine`, `startCommandRoutineRun`) with the `ApiGateway` it holds, with itself as the motion group to run on. `SimulatedRobot` implements them by raising `NotImplementedError`, because it has no NOVA to talk to. `_normalize_actions`, the one funnel of the other action-list paths, raises `ValueError` for a routine, which is what `execute` and the streams do with one.
- **The calls live in the motion group, errors in `nova.command_routines`.** `MotionGroup._plan_routine`, `_start_routine` and `stop_routine` call `self._api_client.command_routines_api` (`planCommandRoutine`, `startCommandRoutineRun`, `stopCommandRoutineRun`) and map API errors with `error_from_api_exception` from `nova/command_routines/exceptions.py` (`CommandRoutineError`; `UnsupportedRoutineCommand` for 501), as `nova/datasets` maps its errors. There are no module-level run functions: a routine always runs on a motion group, and no `Nova` instance is passed down.
- **The motion group to run on is the request's override.** `planCommandRoutine` and `startCommandRoutineRun` take the `controller` the routine runs on (required for a motion routine; NOVA never derives it from a motion group id) and a `motion_group_override`. The MotionGroup always passes its controller and, unless the routine names it already, itself as the override. NOVA then plans the routine with the setup it describes for this motion group instead of the routine's `motion_group_setup`; a routine that names this motion group keeps its own setup. The routine is left as it is.
- **No translation and no waiting.** The routine is not converted into actions; NOVA plans and runs it, so the SDK's trajectory execution (cursor, movement controllers, state streaming) is not used. `plan_and_execute` returns the run once it is accepted, which is the handle for `stop_routine`. Following a run will use NATS, not polling.
- **Errors.** NOVA's messages name the command and reason and are reused as they are. `ValueError` is raised by the SDK for arguments a routine cannot take.
- **Docs and example.** `docs/command_routines.md` and `examples/command_routine.py`.
- **Existing code is left alone.** The routine builders from NDX-844 and the action-list behaviour stay unchanged.

## Testing Decisions

- **One seam.** Every test calls the motion group's `plan`, `plan_and_execute` or `stop_routine` and asserts only what is observable there: what NOVA was asked, the answer and the errors. Command semantics are NOVA's.
- **Routines in the tests.** Each test file builds the routines it needs in code at its head (unit: the 11 features and 15 errors of NOVA's routines service; integration: the subset that runs on a virtual controller). There is no shared scenario module.
- **Unit:** `tests/cell/test_execute_routine.py` with a double of the API: plan, run, stop, the scenarios, the wire format of the settings, the rejections and the simulated robot.
- **Integration:** `tests/nova/cell/motion_group/test_execute_routine.py` on a virtual KUKA: every feature to `COMPLETED`, every error as documented, another motion group, set_io.
- **Regression:** the existing plan/execute, action and routine builder suites stay green.
- **Manual:** one recorded run on a real controller is still outstanding.

## Work Items

- [01 Tracer: run a code-built motion routine](issues/01-tracer-run-code-built-motion-routine.md)
- [03 set_io in routines](issues/03-set-io-in-routines.md)
- [04 Real-controller validation, example and docs](issues/04-real-controller-validation-and-docs.md) (real-controller run pending)
- [05 Plan, start and stop routines through NOVA's command routines API](issues/05-run-routines-through-nova-execution-api.md)

## Out of Scope

- Awaiting the end of a run, following its state and markers, and stopping it when the awaiting task is cancelled: NATS-based, a later step (R6.2, R6.3).
- Manual pause/resume and stepping.
- Synchronized execution across several motion groups; several routines in parallel or chained.
- Exposure via novax (program-run HTTP API). The Service Manager's Routine Execution service (NDX-938) runs routines over HTTP.
- Running routines with the SDK's trajectory execution (movement controllers, `start_on_io`, `pause_on_io`).
- Changes to NDX-844 or its changelog entry.

## Further Notes

- **External dependency.** The command routines API is experimental and needs `wandelbots-api-client` 26.9.0 (dev build `26.9.0+a3248.03b9fa45.517643`). The SDK cannot be released before that client is.
- **The client drops explicit nulls.** The generated client serializes with `exclude_none`, so `motion_group_setup: null`, which declares an I/O-only routine, never reaches NOVA, and `blending: null`, which clears a default, is lost. Both need a client or SDK fix before I/O-only routines and blending resets work; they are covered by strict xfail tests in `TestMotionSettings`, and the integration errors for I/O-only routines are marked xfail for the same reason.
- **Earlier dev instance.** On 2026-10-28 runs on the dev instance moved the robot but stayed `RUNNING`. Check this when the instance is back.
