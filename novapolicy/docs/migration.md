# Migration

novapolicy is an experimental extra, so its API can still change between releases. This page
lists the changes that need an edit, or that change what an unchanged program does.

## Step rate: `control_freq`

Every client takes the policy's step rate as `control_freq=` in Hz: the training data's frame rate
(`fps` in a LeRobot dataset's `meta/info.json`) or a reinforcement-learning policy's control frequency. It is not
`ContinuousExecution(rate_hz=...)`, which sets how often the policy is asked.

| Before                                       | Now                                                |
| -------------------------------------------- | -------------------------------------------------- |
| `Gr00tPolicyClient(..., dt_ms=50.0)`         | `Gr00tPolicyClient(..., control_freq=20)`          |
| `LeRobotPolicyClient(..., fps=15)`           | `LeRobotPolicyClient(..., control_freq=15)`        |
| `LeRobotPolicyClient(..., playback_speed=x)` | `control_freq=` the dataset's `fps`                |
| `CallbackPolicyClient(fn)`                   | unchanged; `control_freq=` fills chunks without `dt_ms` |

- **The LeRobot default changed from 15 to 30.** A policy recorded at 15 fps that relied on the
  default would now play twice as fast: pass `control_freq=15`.
- The GR00T default is 30 Hz (33.3 ms per step, was 33 ms). A `dt_ms` the server reports still
  wins.
- `dt_ms=`, `fps=` and `playback_speed=` still work, with a `DeprecationWarning`.

## Relative TCP rotations are composed

`Action.tcp(..., mode="relative")` used to add the rotation offset to the current rotation vector
component by component. It now turns the tool about the TCP by the offset, composed as rotations.
Positions are unchanged. For small rotations away from a tool-down pose the result is nearly the
same; near a tool-down pose (rotation vectors close to π) the old sum pointed the tool in the wrong
direction. Nothing to change in code; a policy that relied on the old sum turns differently.

New options on the same action: `accumulate=True` for per-step deltas, and
`use_tool_coordinate_system=True` for offsets in the tool coordinate system (see
[schema.md](schema.md)).

## Pre-send stop conditions see the steps that run

At inference time, `StopContext.target_joints` and `target_ios` now hold only the
`n_action_steps` steps that will be sent, not the whole predicted chunk. A stop condition that
looked at later steps of the horizon no longer sees them.

## GR00T docs moved

`novapolicy/gr00t/README.md` is now [docs/gr00t.md](gr00t.md), together with the guide for setting
up a GR00T policy on a real robot.

## `PolicyExecutor(plant=...)` is removed

The executor no longer simulates a plant between a policy and the robot: it maps observations,
runs the policy and sends its actions to NOVA, and NOVA moves the robot. `novapolicy.plant`
(`VirtualPlant`) and the `plant=` argument are gone; no shipped plant existed. Drop the argument.

## The start move keeps the cell's collision setups

`PolicyExecutor(start_joint_position=...)` used to clear the motion group's collision setups for
the move to the start pose. That move is now planned with them, like any other move: a start pose
inside a collision zone fails to plan instead of being reached without collision checks.

## Continuous chunks are dated by their observation

In `ContinuousExecution`, a chunk the policy returns for the latest observation is now dated like
a one-step target: step `k` is due `WaypointConfig.max_expected_latency_ms + k·dt` after the
moment it was observed (on the server's clock, 100 ms by default), and steps already too close to
send are dropped. Before, step 0
was placed at server "now" minus the inference time, rounded to whole steps. Chunk policies
(GR00T, LeRobot without the async queue, NATS) therefore run about `max_expected_latency_ms`
later than before, and lose no head steps while the latency stays below it. Chunks on their own
timeline keep the old placement: RTC (`seam_backdate_steps`), the timestep queues
(`QueuedPolicyClient`, the LeRobot async queue) and bridged chunks.

Such a chunk's IO actions (`ActionChunk.ios`) are written when its first step is due rather than
at send, so they keep their place relative to the arm. NOVA's action chunks carry no IO, so the
write reaches the controller one network delay after that moment.
