# novapolicy

> **Experimental**: this package is under active development and not ready for production use.
> Expect breaking changes between releases.

novapolicy runs trained robot policies (imitation learning, VLAs such as NVIDIA GR00T,
reinforcement learning from Isaac Lab) on industrial robots through
[Wandelbots NOVA](https://wandelbots.com). Each cycle it builds one observation from the robot's
joints, IO and camera images, hands it to the policy, and streams the policy's answer, an action
chunk of joint or TCP targets, to the robot. The policy only maps observations to actions; NOVA
handles cameras, robot state, motion and safety. The same interface fits every policy, so a
trained model is a swappable part. novapolicy also jogs robots directly from live targets, for
teleoperation.

https://github.com/user-attachments/assets/de8a1bb6-9f35-4953-aa32-a8792d2b8244

![NOVA assembles an observation, the policy returns an action chunk, NOVA executes it on the robot](docs/images/policy-blackbox.png)

## Installation

novapolicy ships with the `wandelbots-nova` package (Python 3.12 or later). Install the extra for
the policy backend you use:

| Extra | Use when you need | Adds |
| --- | --- | --- |
| `novapolicy` | the executor, schema, jogging, WebRTC cameras, your own policy callback | `aiortc`, `Pillow`, `requests` |
| `novapolicy-gr00t` | an NVIDIA GR00T inference server, over ZeroMQ | the core plus `msgpack`, `pyzmq` |
| `novapolicy-lerobot` | a LeRobot async-inference server, over gRPC | the core plus `lerobot[async]` |
| `novapolicy-leapp` | a reinforcement-learning policy exported with LEAPP (Isaac Lab) | the core plus `onnxruntime`, `PyYAML`, `safetensors` |

```bash
uv add wandelbots-nova --extra novapolicy
uv add wandelbots-nova --extra novapolicy-gr00t  # or -lerobot, -leapp
```

Extras can be combined (`--extra novapolicy-gr00t --extra novapolicy-lerobot`). If a client
raises a missing-extra error, install the matching extra.

## Quickstart

This NOVA program runs a policy written as an async Python function on a virtual UR10e. NOVA
creates the robot before the program and deletes it afterwards, so no real robot moves. Set
`NOVA_API` to your NOVA instance.

```python
from nova import ProgramContext, ProgramPreconditions, api, program, run_program
from nova.cell import virtual_controller
from novapolicy import (
    ActionChunk,
    CallbackPolicyClient,
    Observation,
    PolicyExecutor,
    PolicySchema,
    SequentialExecution,
)


@program(
    preconditions=ProgramPreconditions(
        controllers=[
            virtual_controller(
                name="quickstart-ur10e",
                manufacturer=api.models.Manufacturer.UNIVERSALROBOTS,
                type="universalrobots-ur10e",
            )
        ],
        cleanup_controllers=True,  # delete the virtual robot when the program ends
    )
)
async def quickstart(ctx: ProgramContext):
    mg = (await ctx.cell.controller("quickstart-ur10e"))[0]

    async def my_policy(obs) -> ActionChunk:
        """Nudge each joint by a small offset (two steps, 50 ms apart)."""
        arm = [obs[f"arm_{i}"] for i in range(1, 7)]
        return ActionChunk(
            joints={mg.id: [[j + 0.01 for j in arm], [j + 0.02 for j in arm]]}, dt_ms=50.0
        )

    schema = PolicySchema(observations=[Observation.joint_positions("arm", source=mg)])
    executor = PolicyExecutor(
        schema, CallbackPolicyClient(my_policy), execution=SequentialExecution(), timeout_s=10.0
    )
    result = await executor.run()
    print(f"Done: {result.reason}, {result.steps} steps, {result.duration_s:.1f}s")


if __name__ == "__main__":
    run_program(quickstart)
```

For a robot in your cell, leave out the preconditions and use its controller name. See
[NOVA programs](../docs/programs.md) for preconditions and running programs.

## Core concepts

### PolicySchema

The schema decouples the policy from the hardware. It turns robot state, IO and camera images
into a flat dict of named features, and turns the policy's answer back into NOVA targets. The
policy never reads motion groups, controllers or IO keys.

```python
from novapolicy import BoolMapping, Observation, PolicySchema

schema = PolicySchema(
    observations=[
        Observation.joint_positions("left", source=mg_left),
        Observation.joint_positions("right", source=mg_right),
        Observation.io(
            "left_gripper", source=mg_left, io="digital_out[0]", mapping=BoolMapping(on=100.0)
        ),
    ]
)
# The policy sees {"left_1": 0.1, ..., "left_6": 0.3, "right_1": 0.2, ..., "left_gripper": 0.0}
```

A writable observation, such as joint positions or an IO with a mapping, is also an action: the
policy's output for it is sent back to the robot. Observations can also be joint velocities, a
TCP pose, object or goal poses, constants and computed values; units such as degrees or metres
are declared once and apply in both directions. See [docs/schema.md](docs/schema.md).

#### Cameras

Camera streams run in the Camera App on the NOVA instance. The schema reads frames from a
running stream over WebRTC; images arrive as `numpy.ndarray` (H×W×3, uint8, RGB).

```python
from novapolicy import Observation, PolicySchema, WebRTCCameras

cameras = WebRTCCameras(api_url="http://<nova-host>:8011/webrtc-streamer", resize=(224, 224))
schema = PolicySchema(
    observations=[
        Observation.joint_positions("arm", source=mg),
        Observation.image("wrist", source=cameras.device("315122271048")),
    ]
)
```

#### Rotation representation and coordinate systems

NOVA reports TCP poses as `[x, y, z, rx, ry, rz]` (mm, rotation vector) in the controller's world.
A policy trained on another rotation representation or frame declares it on the TCP entry:

```python
from novapolicy import Action, Observation, OrientationType

Observation.tcp(
    "eef", source=mg, orientation_type=OrientationType.QUATERNION, coordinate_system="table"
)
Action.tcp("eef", target=mg, orientation_type=OrientationType.QUATERNION, coordinate_system="table")
```

`orientation_type` takes any of NOVA's `OrientationType` values. `coordinate_system` names a
coordinate system of the controller; the executor fetches it at start, and
`await schema.load_coordinate_systems()` does it for a dry run.

#### Checking units against the checkpoint

When a client can read the checkpoint's normalization statistics, it checks the first live
observation against the training range. Values far outside it, the sign of a unit mismatch, raise
before the robot moves; values somewhat outside it log a warning. LeRobot reads the statistics
from the checkpoint and skips the check for a checkpoint shipped as `config.json` alone. For GR00T,
pass them with `Gr00tPolicyClient(..., statistics="checkpoint/statistics.json")`; name the
embodiment with `embodiment=` if the one matching the server's state keys is not the one you want.

### Policy clients

A `PolicyClient` connects the executor to a policy. Pick the one for your policy:

| Client | For | The policy runs | Guide |
| --- | --- | --- | --- |
| `CallbackPolicyClient(fn)` | your own async Python function | in-process | [Quickstart](#quickstart) |
| `Gr00tPolicyClient(host, port=...)` | an NVIDIA GR00T inference server | on the server, over ZeroMQ | [docs/gr00t.md](docs/gr00t.md) |
| `LeRobotPolicyClient(server_address, pretrained_name_or_path)` | LeRobot policies (ACT, SmolVLA, pi0, ...) | on a LeRobot async-inference server, over gRPC | [providers/lerobot/README.md](providers/lerobot/README.md) |
| `LeappPolicyClient(pipeline)` | reinforcement-learning policies exported with LEAPP (Isaac Lab) | in-process, with onnxruntime | [docs/reinforcement-learning.md](docs/reinforcement-learning.md) |

A policy returns an `ActionChunk`: one or more future steps per motion group, `dt_ms` apart.
Pass `control_freq=` to a client to set its step rate; without one, a client that needs a rate
uses 30 Hz. Wrap a client that returns one chunk per observation in `QueuedPolicyClient` to
replace its plan step by step in continuous execution. For another policy service, write your
own client: [docs/adding-a-provider.md](docs/adding-a-provider.md).

### PolicyExecutor

`PolicyExecutor(schema, policy, ...)` runs one episode: observe, ask the policy, send the actions,
repeat. The policy never ends a run; the executor ends it on `timeout_s`, a stop condition,
`stop()`, an e-stop or a stale input. Two execution modes:

- `SequentialExecution()` (the default): the robot finishes each chunk and comes to rest before
  the next inference.
- `ContinuousExecution(rate_hz=...)`: the policy runs while the robot moves, and each new chunk
  replaces the queued one. Without `rate_hz`, as fast as inference allows.

A stop condition is a fast, synchronous check that runs on every tick. Returning `True` ends the
run normally; its name appears in `result.reason`:

```python
from novapolicy import PolicyExecutor, SequentialExecution, StopContext


def stop_on_io(ctx: StopContext) -> bool:
    """Stop the policy when digital_in[3] goes high."""
    return bool(ctx.io_values and ctx.io_values.get("digital_in[3]"))


executor = PolicyExecutor(
    schema, policy, execution=SequentialExecution(), stop_conditions=[stop_on_io]
)
result = await executor.run()  # result.reason == "stop condition: stop_on_io"
```

Stop conditions must not block; use `Observation.computed()` for async data. Execution modes,
timing, stale inputs and how a run ends: [docs/executor.md](docs/executor.md).

Limits for moves NOVA plans, such as the move to the start pose, are `MotionSettings`, as
everywhere in NOVA. Streamed policy steps run within the controller's own limits.

> The executor's checks are software guards, not a safety system. For production, rely on the
> safety zones and protective stops configured on the robot controller.

### Jogging and teleoperation

`jog_joints` and `jog_tcp` move a robot to targets you set, without a policy. Feed
`set_target(...)` from any input, such as a leader arm, keyboard, gamepad, spacemouse or VR
controller. For a hand-held device, add `ServoHorizon`, which follows the targets smoothly and
brakes onto the newest one:

```python
from novapolicy import ServoHorizon, default_pipeline, jog_tcp

async with jog_tcp(mg, tcp="Flange", pipeline=[ServoHorizon(), *default_pipeline()]) as jogger:
    async for state in jogger:
        jogger.set_target(read_hand_pose())  # on every new pose from the device
```

See [docs/jogging.md](docs/jogging.md).

### The send pipeline

Every target the executor or a jogger sends passes an ordered list of modules: they bridge a
distant first step, ease in and out, clamp to joint limits, give the robot a horizon to brake
within, and drop late waypoints. Each use has sensible defaults; pass `pipeline=[...]` to add,
remove or configure modules, or to add your own. See [docs/pipeline.md](docs/pipeline.md).

## Guides

| Guide | Covers |
| --- | --- |
| [docs/schema.md](docs/schema.md) | Every observation and action type, IO mappings, units, relative and TCP actions, object poses, computed observations and actions |
| [docs/executor.md](docs/executor.md) | Sequential and continuous execution, asynchronous inference, timing, stale inputs, how a run ends |
| [docs/pipeline.md](docs/pipeline.md) | The send pipeline: defaults, every built-in module, adding and writing modules |
| [docs/jogging.md](docs/jogging.md) | Jogging without a policy: joint and TCP targets, chunks, live targets, dual-arm |
| [docs/gr00t.md](docs/gr00t.md) | `Gr00tPolicyClient` and setting up a GR00T policy on a real robot |
| [providers/lerobot/README.md](providers/lerobot/README.md) | `LeRobotPolicyClient` and LeRobot async inference |
| [docs/reinforcement-learning.md](docs/reinforcement-learning.md) | Running a reinforcement-learning policy exported with LEAPP (Isaac Lab) |
| [docs/adding-a-provider.md](docs/adding-a-provider.md) | Writing a `PolicyClient` for another policy service or runtime |
| [docs/rerun.md](docs/rerun.md) | Optional real-time 3D visualization of a run |
| [docs/migration.md](docs/migration.md) | Changes that need an edit when upgrading |

## Examples

- [`execute_custom_policy_on_dual_arm.py`](examples/execute_custom_policy_on_dual_arm.py): two
  UR5e robots with cameras, IO and stop conditions.
- [`execute_gr00t_dual_arm.py`](examples/execute_gr00t_dual_arm.py): a dual-arm GR00T policy
  with four cameras.
- [`execute_gr00t_single_arm_pick_and_place/`](examples/execute_gr00t_single_arm_pick_and_place/):
  a GR00T pick-and-place trained in Isaac Lab, on a UR10e with a Zimmer gripper, sequential or
  continuous.
- [`execute_isaaclab_rl_reach/`](examples/execute_isaaclab_rl_reach/): an Isaac Lab
  reinforcement-learning reach policy (LEAPP export) on a UR10e, one action per step.
- [`execute_leapp_two_point_reach/`](examples/execute_leapp_two_point_reach/): a
  reinforcement-learning policy (LEAPP export) that moves a UR10e to a part detected at one of two
  points.
- [`computed/`](examples/computed/): `Observation.computed` and `Action.computed` on a virtual
  UR5e with a mock policy.
- [`http_policy.py`](examples/http_policy.py): a new policy client in one class, for a policy
  served over HTTP, with a mock server.
- [`jogging/`](examples/jogging/): jogging without a policy (single and dual arm, joint and TCP,
  live targets and chunks); [`jogging_keyboard.py`](examples/jogging/jogging_keyboard.py) jogs the
  TCP from the keyboard, [`jogging_quest.py`](examples/jogging/jogging_quest.py) follows a mocked
  Meta Quest controller with `ServoHorizon`.
- [`pick_and_place_umi_ur10e.py`](examples/pick_and_place_umi_ur10e.py): a pick-and-place written
  out by hand, to prove a cell before a policy runs on it.

Install the cells for the examples with [`setup_cell.sh`](examples/setup_cell.sh): `dual-arm` for
the two-UR5e examples, `umi` for the UR10e with the UMI gripper and its `umi_corrected` TCP.

## Contributing

Architecture, design decisions and developer notes are in [dev/README.md](dev/README.md).
