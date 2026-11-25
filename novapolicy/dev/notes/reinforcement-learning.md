# Notes: reinforcement-learning guide

These notes were moved out of the customer guide. They keep development history,
measurements and design rationale. The text is verbatim.

## From docs/reinforcement-learning.md, Check before a real run

| Check | How |
| --- | --- |
| Joint velocities | UR controllers send no measured velocity; expect differenced values. |

## From docs/reinforcement-learning.md, Background

**Continuous, not sequential.** A reinforcement-learning policy was trained with the next action
computed while the joints were still moving. Continuous execution does the same; sequential
execution runs each step to standstill, so the policy sees zero joint velocity at every observation.
On a virtual UR10e a scripted one-step policy ran at 58 Hz of 60 continuously and at 6 Hz
sequentially.

**Latency.** Each waypoint must reach the controller about 67 ms before it is due, so it needs a
lead that also covers the observation's age and the network round trip. Every request re-sends
the earlier waypoints that are not due yet, so the controller always has the next few queued. A
target is due `ActionChunkConfig(max_expected_latency_ms=...)` after the
observation it was computed from (100 ms by default, as MoveIt Servo's `max_expected_latency`;
dated by the observation as UMI and Diffusion Policy do), without a braking hold, so the arm
executes it that much later. Consecutive targets are spaced one policy step apart on a playback
clock that follows the observations slowly, as a streaming jitter buffer does: dating each target
by its own observation would pass the state stream's jitter on as speed jumps. Run the policy on
the NOVA instance, where
100 ms fits; over a remote network raise the value (150 ms over a 40 to 120 ms VPN, where state
packets arrive late and in bursts). Train the policy for it as sim-to-real practice does: a
randomized action delay that covers the value, and its previous action or target in the
observation, as the [worked example](../../examples/execute_leapp_two_point_reach/) does.

**Pulsing on the virtual robot.** NOVA's follower steers toward the next waypoint from a state one
8 ms cycle old, and a waypoint due just after a cycle makes the arm pulse around the policy's
pace. `Timestamps(grid_ms=32)` in the pipeline puts the targets on that cycle grid (the example's
`--grid-ms 32`); on a virtual UR10e at a 150 ms lead it halved the largest path deviation and cut
the speed ripple by about 28% ([ADR 0021](../adr/0021-one-step-targets-on-the-cycle-grid.md)). Test
it on a physical robot, and at a shorter lead, before relying on it there. Swap it into the default
pipeline rather than passing it alone, which would drop the other modules (`JointClamp` among
them):

```python
from novapolicy import ContinuousExecution, PolicyExecutor, Timestamps, default_pipeline

execution = ContinuousExecution(rate_hz=client.control_freq)
pipeline = [
    Timestamps(grid_ms=32) if isinstance(m, Timestamps) else m
    for m in default_pipeline(execution=execution, policy=client)
]
executor = PolicyExecutor(schema, client, execution=execution, pipeline=pipeline, n_action_steps=0)
```

For another policy rate, take the multiple of 8 ms closest to its step.

**Limitations.** Only ONNX exports run; TorchScript and `pt2` exports need LEAPP's torch runtime.
One joint target and one joint action are supported; image inputs are not fed yet.
