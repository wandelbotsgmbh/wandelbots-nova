# Run a trained reinforcement-learning policy on NOVA

You have a reinforcement-learning policy trained in simulation, in Isaac Lab or with your own code.
This guide maps its inputs and outputs onto a `PolicySchema` and runs it on a NOVA robot with
`LeappPolicyClient`.

The policy has to be exported with [LEAPP](https://github.com/nvidia-isaac/leapp). The export
contains the network and the observation and action processing of the training environment
(normalization, offsets, action scale, last action), so none of that is rebuilt on the robot side.
You declare which NOVA value feeds each input, and which output commands the robot.

You need a NOVA instance (its URL in `NOVA_API`, plus `NOVA_ACCESS_TOKEN` for a cloud instance)
and either a robot controller on it or a virtual one. The [complete script](#complete-script)
creates a virtual robot for a first run. Two worked examples:

- [`execute_leapp_two_point_reach/`](../examples/execute_leapp_two_point_reach/): a policy that
  reaches a detected part, with the trained export included.
- [`execute_isaaclab_rl_reach/`](../examples/execute_isaaclab_rl_reach/): Isaac Lab's UR10e reach
  task, with your own export.

## 1. Export the policy

Export with the ONNX backend, using the same physics backend and overrides as training. The client
runs ONNX exports only.

```bash
uv run --extra leapp isaaclab leapp export --rl_library rsl_rl --task <task>
```

Then install the client's extra:

```bash
uv sync --extra "novapolicy-leapp"
```

## 2. List the export's inputs and outputs

```python
from novapolicy.providers.leapp import LeappPipeline

pipeline = LeappPipeline("path/to/export.yaml")
print(pipeline.frequency)
for port in [*pipeline.inputs, *pipeline.outputs]:
    print(port.key, port.connection, port.kind, list(port.shape))
```

It prints the policy's rate in Hz, then one line per port: its `node/name`, its Isaac Lab
connection, its kind and its shape. For an arm policy that observes its joints and a goal pose,
the ports look like this (names and widths depend on your policy):

| Port | `isaaclab_connection` | `kind` | Shape |
| --- | --- | --- | --- |
| input `<node>/robot_joint_pos` | `state:robot:joint_pos` | `joint/position` | [1, joints] |
| input `<node>/robot_joint_vel` | `state:robot:joint_vel` | `joint/velocity` | [1, joints] |
| input `<node>/<command>` | `command:<command>` | | [1, 7] |
| output `<node>/<action term>` | `write:robot:set_joint_position_target...` | `joint/position` | [1, joints] |

Feedback states such as the last action are not listed: the export carries them itself. Each
listed input needs exactly one observation in the schema.

## 3. Map each input to an observation

| Export input | Observation | Units |
| --- | --- | --- |
| `state:<robot>:joint_pos`, `kind: joint/position` | `Observation.joint_positions("joint_pos", source=mg, action=False)` | rad, as NOVA |
| `state:<robot>:joint_vel`, `kind: joint/velocity` | `Observation.joint_velocities("joint_vel", source=mg)` | rad/s, as NOVA |
| `command:<name>` holding a pose (7 values: position and quaternion) | `Observation.pose("<name>", source=..., orientation_type=OrientationType.QUATERNION, position_ops=[Scale(0.001)])` | NOVA mm → m, rotation vector → quaternion `(x, y, z, w)` |
| any other value the robot cannot measure (a constant of the training setup, a non-pose command) | `Observation.constant("<name>", value=[...])` or `Observation.computed(fn, keys=["<name>"])` | as trained |

Feed joint positions as NOVA reports them, even if the policy was trained on positions relative to
a default pose: the export subtracts that pose itself. `action=False` keeps the observation from
also declaring an action; the joint action is declared on its own (step 4).

The observation names are the mapping. The client feeds each export input from the observation
with the same name. Name each observation after the last part of the input's connection
(`joint_pos`, `joint_vel`, the command name), or after the input's full port name from step 2.
When two inputs share a connection name, as two robots' `joint_pos` do, use the port names
(`left_robot_joint_pos`, `right_robot_joint_pos`).

The values must match what the policy saw in training:

- **Order and width.** Joint observations list the motion group's joints in NOVA's order. Check it
  against the training robot's joint order. The width has to equal the port's shape, or the first
  inference reports the mismatch.
- **Units.** Joints need no conversion. Positions in NOVA are mm; most policies use metres, hence
  `Scale(0.001)`. Current Isaac Lab orders quaternions `(x, y, z, w)`, as
  `OrientationType.QUATERNION` does. Isaac Lab 2.x used `(w, x, y, z)`.
- **Frame.** Isaac Lab's pose commands are in the robot's base frame. NOVA poses are in the
  controller's world. If that is not the robot's base (a mounted robot), convert in the source.

A pose command, such as a goal, takes its value from the source of its `Observation.pose`: a fixed
pose, an async function, a detector running at its own rate, or an object in an Isaac Sim scene.
The policy reads it on every step, so it follows a goal that changes during the run. How to write
a source, and how to turn a goal in metres and a quaternion into a NOVA `Pose`, is in
[Object poses](schema.md#object-poses).

## 4. Map the output to the action

The export's joint output is the final joint target: the action term's scale and offset are
already applied. Declare it as an absolute joint action:

```python
from novapolicy import Action

actions = [Action.joint_positions("arm", target=mg)]
```

The client takes the output of kind `joint/position`, or the output with a
`write:<robot>:set_joint_position_target...` connection. If the export has several, name the
joint action like the output it should command. An output named like a key of
`Action.computed(..., keys=[...])` reaches that hook in `chunk.values`. Other outputs, such as
exported gains, are not used.

## 5. Run

Run in this order, each step only when the one before looks right:

1. **Dry run.** Build one observation and run one inference, without commanding anything. Compare
   the printed observation with the training environment's: units, joint order, frame, quaternion
   sign. A policy that outputs absolute targets can command one far from the current joints. The
   arm moves there within its velocity limits, without a warning, so compare the dry run's joint
   target with the current joints.
2. **Virtual robot.** Run the policy on a virtual controller and compare where the robot ends with
   where the policy ends in simulation.
3. **Real robot**, supervised, with the controller's safety zones configured.

`validate_schema`, `connect`, `get_actions`, `build_observation` and the `start_sources` /
`stop_sources` pair are coroutines: `await` them. A `validate_schema` that is not awaited checks
nothing. `executor.run()` connects and validates by itself; call them yourself only for a dry run.

`PolicyExecutor(start_joint_position={mg: joints})` moves the robot to the training start pose
before the policy starts. Use a stop condition to refuse inputs the policy was not trained for.
Stop conditions run on every tick, before every action is sent and while the run starts up, so
the robot does not move on such an input. Set `LOG_LEVEL=WARNING` to hide the executor's per-step
log lines.

### Complete script

A template: a dry run, then the policy, on a virtual robot that NOVA creates and deletes. Fill in
the constants for your policy and robot, and add one observation per further export input, as
mapped in step 3. The [worked example](../examples/execute_leapp_two_point_reach/) is a filled-in,
runnable version.

```python
"""Run a trained reinforcement-learning policy (LEAPP export) on a NOVA robot.

NOVA_API=http://<nova-host> uv run python run_policy.py
"""

from nova import ProgramContext, ProgramPreconditions, api, program, run_program
from nova.cell import virtual_controller
from novapolicy import (
    Action,
    ContinuousExecution,
    LeappPolicyClient,
    Observation,
    PolicyExecutor,
    PolicySchema,
)

# --- Your policy and robot ----------------------------------------------------------------
EXPORT = "path/to/export.yaml"  # the LEAPP export's .yaml
TCP = "Flange"  # the TCP the policy was trained for
START_JOINTS = None  # the joints training started from (rad); None: start where the robot is
VIRTUAL = True  # False: run on the existing controller CONTROLLER, which moves the real robot
CONTROLLER = "policy-test-robot"  # with VIRTUAL, the virtual controller NOVA creates and deletes
MANUFACTURER = api.models.Manufacturer.UNIVERSALROBOTS  # the virtual robot's model
ROBOT_TYPE = "universalrobots-ur10e"

VIRTUAL_ROBOT = ProgramPreconditions(
    controllers=[virtual_controller(name=CONTROLLER, manufacturer=MANUFACTURER, type=ROBOT_TYPE)],
    cleanup_controllers=True,
)


@program(preconditions=VIRTUAL_ROBOT if VIRTUAL else None)
async def run_policy(ctx: ProgramContext) -> None:
    controller = await ctx.cell.controller(CONTROLLER)
    async with controller[0] as mg:  # the robot's first motion group
        schema = PolicySchema(
            observations=[
                Observation.joint_positions("joint_pos", source=mg, action=False),
                Observation.joint_velocities("joint_vel", source=mg),
                # one observation per further export input (step 3), e.g. a goal pose command
                # not a policy input: makes the run use TCP, so state poses are its pose
                Observation.tcp("tcp", source=mg, tcp=TCP),
            ],
            actions=[Action.joint_positions("arm", target=mg)],
        )
        client = LeappPolicyClient(EXPORT)

        # 1. Dry run: one observation, one inference, nothing commanded.
        await schema.start_sources()
        try:
            await client.connect([mg.id])
            await client.validate_schema(schema)
            states = {mg.id: await mg.get_state(TCP)}
            print("observation:", await schema.build_observation(states))
            chunk = await client.get_actions(states, schema)
            print("joint target:", chunk.joints[mg.id][0])
        finally:
            await schema.stop_sources()

        # 2. The policy, from the training start pose if one is given.
        executor = PolicyExecutor(
            schema,
            client,
            execution=ContinuousExecution(rate_hz=client.control_freq),
            n_action_steps=0,  # one action per inference, replaced by the next one
            start_joint_position={mg: START_JOINTS} if START_JOINTS else None,
            stop_conditions=[],  # add checks for inputs outside the trained range (step 5)
            timeout_s=10.0,
        )
        result = await executor.run()
        rate = result.steps / result.duration_s
        print(f"{result.reason} after {result.steps} steps ({rate:.1f} Hz)")


if __name__ == "__main__":
    run_program(run_policy)
```

With `START_JOINTS` set, the run first moves the robot to the pose training started from. The dry
run before it sees the robot where it is. Without it, the policy starts from the current position,
which works when that is close to what the policy saw in training.

## Execution settings

**Use continuous execution.** A reinforcement-learning policy was trained with the next action
computed while the joints were still moving. `ContinuousExecution(rate_hz=client.control_freq)`
does the same. `SequentialExecution` runs each step to standstill, so the policy sees zero joint
velocity at every observation and runs far below its rate.

**Latency.** Each target is due `ActionChunkConfig(max_expected_latency_ms=...)` after the
observation it was computed from. The default is 100 ms, which fits when the script runs on the
NOVA instance. Over a remote network, raise it to about 150 ms:

```python
from novapolicy import ActionChunkConfig

executor = PolicyExecutor(
    schema,
    client,
    execution=ContinuousExecution(rate_hz=client.control_freq),
    motion=ActionChunkConfig(max_expected_latency_ms=150),
    n_action_steps=0,
)
```

The arm executes each target that much after its observation. If you train your own policy, train
it for that delay: randomize the action delay in training to cover the value, and put the previous
action or target in the observation. The
[two-point reach example](../examples/execute_leapp_two_point_reach/) is trained this way.

**Targets on NOVA's cycle grid.** If the arm pulses around the policy's pace, put the targets on
NOVA's 8 ms cycle grid with `Timestamps(grid_ms=...)`. Use the multiple of 8 ms closest to the
policy's step: 32 for a 30 Hz policy. It is off by default. Swap it into the default pipeline
rather than passing it alone, which would drop the other modules, `JointClamp` among them:

```python
from novapolicy import ContinuousExecution, PolicyExecutor, Timestamps, default_pipeline

execution = ContinuousExecution(rate_hz=client.control_freq)
pipeline = [
    Timestamps(grid_ms=32) if isinstance(m, Timestamps) else m
    for m in default_pipeline(execution=execution, policy=client)
]
executor = PolicyExecutor(schema, client, execution=execution, pipeline=pipeline, n_action_steps=0)
```

[pipeline.md](pipeline.md) describes the default pipeline and its modules.

## Check before a real run

| Check | How |
| --- | --- |
| Every input fed | `await client.validate_schema(schema)` names any input without an observation. |
| Units, order and frame | Compare the dry run's observation with the training environment's. |
| Rate | Compare `result.steps / result.duration_s` with `client.control_freq`. |
| Latency | Run on the NOVA instance. Over a remote network raise `max_expected_latency_ms` to 150 ms. |
| Joint velocities | Where the controller sends no measured velocity, expect differenced values. |
| Response | Run on a virtual robot first and compare where it ends with the training environment. |
| Inputs in range | A stop condition refuses goals or commands outside the trained range. |
| Safety | Configure the controller's safety zones. Stop conditions and the stale-input deadlines are software checks. |

If the policy fails on the robot, start the robot from the same observation as in simulation and
compare the responses step by step. That separates an input mismatch from a response mismatch.

## Troubleshooting

| Symptom | What to do |
| --- | --- |
| `Schema does not satisfy the LEAPP export: No observation feeds ...` | Add an observation with one of the names the message lists. |
| `... is joint/position: feed it with Observation.joint_positions(...)` | That input needs the observation kind named in the message. |
| `The schema has 0 joint actions; exactly one is supported` | Declare one `Action.joint_positions`. |
| `No single joint target among the export's outputs` | Name the joint action like the output it should command. |
| `The export outputs absolute joint targets` | Remove `mode="relative"` from the joint action. |
| `<port> takes N values, observation ... has M` | The observation's width differs from the port. Check the joints and the pose's `orientation_type`. |
| The export fails to load and asks for the ONNX backend | Export again with the ONNX backend. |
| The run is far below `client.control_freq` | Use `ContinuousExecution`, and run the script on the NOVA instance. |
| The arm lags or overshoots over a remote network | Raise `max_expected_latency_ms` to 150 ms. |
| The arm pulses around the policy's pace | Put the targets on the cycle grid with `Timestamps(grid_ms=...)`. |
| The arm moves somewhere unexpected | Compare the dry run's observation with training: units, joint order, frame, quaternion order. |

## Reference

### LeappPolicyClient

`LeappPolicyClient(pipeline, *, control_freq=None)`

| Argument | Meaning |
| --- | --- |
| `pipeline` | the export's `.yaml` file |
| `control_freq` | step rate in Hz; read from the export's `pipeline.configs.frequency`, else 30 |

Each export input is fed from the observation named like its port (`robot_joint_vel`), or like the
last part of its Isaac Lab connection (`state:robot:joint_vel` → `joint_vel`, `command:ee_pose` →
`ee_pose`) when no other input shares that name. The joint target is the output of kind
`joint/position` or with a `write:<robot>:set_joint_position_target...` connection. Where that is
ambiguous, it is the output named like the joint action. Feedback inputs (such as the last action)
carry from step to step and start from the export's initial values at every run. The client needs
the `novapolicy-leapp` extra (`onnxruntime`, `PyYAML`, `safetensors`), not torch.

`validate_schema` refuses a schema when:

- no observation feeds an export input;
- a `joint/position` or `joint/velocity` input is fed by another kind of observation;
- the joint action is `mode="relative"` (the export outputs absolute targets);
- the export has no or several joint targets and none is named like the joint action;
- the schema has no joint action, or more than one;
- an `Action.computed` key matches no export output.

### Joint velocities

`Observation.joint_velocities(key, source, *, ops=None)` gives `key_1 .. key_n` in rad/s: the
controller's measured `joint_velocity` where its state stream sends it, else the difference of the
state packets on their timestamps across at least 20 ms.

## Sources

- [Isaac Lab policy deployment](https://isaac-sim.github.io/IsaacLab/main/source/policy_deployment/index.html):
  [reach](https://isaac-sim.github.io/IsaacLab/main/source/policy_deployment/04_reach/reach_policy.html),
  [gear assembly](https://isaac-sim.github.io/IsaacLab/main/source/policy_deployment/02_gear_assembly/gear_assembly_policy.html)
  (sim-to-real consistency, debugging) and
  [LEAPP export](https://isaac-sim.github.io/IsaacLab/main/source/policy_deployment/05_leapp/exporting_policies_with_leapp.html)
- [LEAPP](https://github.com/nvidia-isaac/leapp): export format and `InferenceManager`
- [Wandelbots Isaac Sim extension API](https://docs.wandelbots.com/simulation/extension-api.md)
