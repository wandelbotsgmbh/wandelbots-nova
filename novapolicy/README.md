# policy

> **⚠️ EXPERIMENTAL**: This package is under active development and not ready for production use. Expect breaking changes between releases.

Execute learned policies (imitation learning, reinforcement learning) on industrial robots via [Wandelbots NOVA](https://wandelbots.com).

https://github.com/user-attachments/assets/de8a1bb6-9f35-4953-aa32-a8792d2b8244

## Overview

NOVA owns everything physical. Each cycle it assembles one **observation**, the camera
images, the current joint configuration, and the relevant I/O states, and hands it to the
policy. The policy returns an **action chunk** (joint targets), which NOVA streams to the
jogging interface and servos onto the robot. NOVA handles cameras, robot state, execution,
safety, and closing the loop every cycle; the policy only maps observation → action chunk.

That interface is the same whether the policy is an imitation-learning model, a VLA like
GR00T, a reinforcement-learning policy trained in Isaac Lab, or a world model, so any trained model is a swappable
module.

![NOVA assembles an observation, the policy returns an action chunk, NOVA executes it on the robot](docs/images/policy-blackbox.png)

**Robot control lives on the IPC, not on the (potentially remote) GPU server running the policy.**
A small policy, such as a reinforcement-learning policy exported to ONNX, can also run in-process on the IPC, with no
GPU server.

```mermaid
flowchart LR
    subgraph GPU["GPU Server"]
        Policy["Policy Model\n(stateless)"]
    end

    subgraph IPC["IPC (at the robot)"]
        Executor["PolicyExecutor"]
        Motion["NOVA Motion API"]
        Cameras["WebRTC cameras"]
        Robot["Robot"]
    end

    subgraph CamServer["Camera Server"]
        WebRTC["WebRTC streams"]
    end

    Policy <-->|"ZMQ / HTTP / custom"| Executor
    WebRTC <-->|"WebRTC"| Cameras
    Executor --> Motion --> Robot
    Cameras --> Executor
```

The policy is a **stateless pure function**: `obs → actions`. It never controls lifecycle.
The executor decides **when** to start and **when** to stop, and runs the software guards.

## Install

`novapolicy` is split into optional extras so users only install the policy transport they need.

| Extra | Use when you need | Adds |
| --- | --- | --- |
| `novapolicy` | core `PolicyExecutor`, schema, waypoint jogging, WebRTC cameras, custom callback policies | `aiortc`, `Pillow`, `requests` |
| `novapolicy-gr00t` | NVIDIA GR00T ZeroMQ policy client | core policy deps + `msgpack`, `pyzmq` |
| `novapolicy-lerobot` | LeRobot async gRPC inference client/server integration | core policy deps + `lerobot[async]` |
| `novapolicy-leapp` | reinforcement-learning policies exported with LEAPP (Isaac Lab), see [docs/reinforcement-learning.md](docs/reinforcement-learning.md) | core policy deps + `onnxruntime`, `PyYAML`, `safetensors` |

Install exactly one policy-client extra if you know which backend you use:

```bash
# Core policy execution + WebRTC cameras only
uv add wandelbots-nova --extra novapolicy

# GR00T client support
uv add wandelbots-nova --extra novapolicy-gr00t

# LeRobot async-inference client support
uv add wandelbots-nova --extra novapolicy-lerobot

# reinforcement-learning policies exported with LEAPP (Isaac Lab)
uv add wandelbots-nova --extra novapolicy-leapp
```

Extras can also be combined, for example when developing or testing multiple policy backends:

```bash
uv add wandelbots-nova --extra novapolicy-gr00t --extra novapolicy-lerobot
```

The GR00T, LeRobot and LEAPP clients keep their heavy dependencies optional. If a missing-extra
error is raised at runtime, install the matching backend extra above.

## Quick Start

A local policy callback is an async function wrapped by `CallbackPolicyClient`: observations in, an action chunk out.
The example is a NOVA program that runs on a virtual UR10e, which NOVA creates before the program
and deletes after it, so it moves no real robot; set `NOVA_API` to your NOVA instance.

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
        """Nudge each joint by a small offset (two steps, 50ms apart)."""
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

For a robot of your cell, leave out the preconditions and use its controller name. See
[NOVA programs](../docs/programs.md) for preconditions and running programs.

### Policy clients

| Client | For | The policy runs | Step rate | Execution | Docs |
| --- | --- | --- | --- | --- | --- |
| `CallbackPolicyClient(fn)` | your own async Python function | in-process | the chunk's `dt_ms`, or `control_freq=` | sequential or continuous | Quick Start above |
| `Gr00tPolicyClient(host, port=...)` | an NVIDIA GR00T inference server | on the GR00T server, over ZMQ | `control_freq=`, default 30 Hz | sequential; continuous through `QueuedPolicyClient` | [docs/gr00t.md](docs/gr00t.md) |
| `LeRobotPolicyClient(server_address, checkpoint)` | LeRobot policies (ACT, SmolVLA, pi0, ...) | on a LeRobot async-inference server, over gRPC | `control_freq=` = the dataset's `fps`, default 30 Hz | sequential or continuous | [providers/lerobot/README.md](providers/lerobot/README.md) |
| `LeappPolicyClient(export)` | reinforcement-learning policies exported with LEAPP (Isaac Lab) | in-process, with onnxruntime | from the export | `ContinuousExecution(rate_hz=client.control_freq)`, `n_action_steps=0` | [docs/reinforcement-learning.md](docs/reinforcement-learning.md) |

A client given no step rate where it needs one uses 30 Hz. Use `CallbackPolicyClient` to adapt an async callable that maps a feature `dict` to an `ActionChunk`; service integrations implement `PolicyClient` directly ([adding a provider](docs/adding-a-provider.md)). An `ActionChunk` carries one or more future steps per motion group (with `dt_ms`, and an optional `first_timestamp_ms` anchor for overlapping chunks). The executor maps and transports: it builds the observations, converts the actions back to NOVA and streams them as timestamped action chunks, and it stops a run on an e-stop, a stale input or a stop condition. How the arm follows the waypoints is NOVA's job ([contracts](docs/contracts.md)).

## PolicySchema

Decouples the policy's **observations** from hardware topology. The policy sees a flat dictionary of named features built from the schema; it doesn't read motion groups, controllers, or hardware IO keys to interpret its inputs.

```python
from novapolicy import BoolMapping, Observation, PolicySchema

schema = PolicySchema(
    observations=[
        Observation.joint_positions("left", source=mg_left),
        Observation.joint_positions("right", source=mg_right),
        Observation.io(
            "left_gripper", source=mg_left, io="digital_out[0]", mapping=BoolMapping(on=100.0)
        ),
        Observation.io(
            "right_gripper", source=mg_right, io="digital_out[0]", mapping=BoolMapping(on=100.0)
        ),
    ]
)
```

This produces observations like:

```python
{
    "left_1": 0.1, "left_2": -1.5, ..., "left_6": 0.3,
    "right_1": 0.2, ..., "right_6": -0.1,
    "left_gripper": 0.0,      # closed
    "right_gripper": 100.0,   # open
}
```

Observations can also be joint velocities, a TCP pose, the pose of an object or goal (fixed, from a
detector, or from an Isaac Sim scene), constants and computed values: see the table of all
observation types in [docs/schema.md](docs/schema.md#observations).

The policy returns an `ActionChunk` keyed by motion-group id. Targets are sent as an `ActionChunkRequest` — joint targets as `JOINTS` waypoints, TCP targets as `POSE` waypoints — and IO values get written to hardware (use `BoolMapping`/`Mapping` on the matching observation so a learned policy's scaled outputs map back to hardware values).

### Cameras

Cameras are managed by the **Camera App** on the NOVA instance: the user starts and stops camera streams via the Camera App UI. The policy client only opens a WebRTC session to receive frames from an already-running stream. Pass `resize=(width, height)` to scale every frame to the size your policy expects on read.

```python
from novapolicy import Observation, WebRTCCameras

# Point to the camera server running on your NOVA instance.
# Frames are resized to the policy's expected input size on read.
cameras = WebRTCCameras(api_url="http://<nova-host>:8011/webrtc-streamer", resize=(224, 224))

schema = PolicySchema(
    observations=[
        Observation.joint_positions("arm", source=mg),
        Observation.image("flange", source=cameras.device("315122271048")),
        Observation.image("left", source=cameras.device("314522065367")),
    ]
)
```

Images arrive as `numpy.ndarray` (H×W×3, uint8, RGB) in the observation dict.

### Units

NOVA speaks radians and millimetres. A dataset does not necessarily. LeRobot's policy server
applies the checkpoint's own normalization, so **do not** normalize here — but those statistics are
in the *recording* robot's units, so a dataset captured in degrees normalizes perfectly cleanly and
moves the arm to the wrong place.

Declare the conversion once and it applies in both directions: forward on the observation,
inverted on the action.

```python
from novapolicy import Clamp, Observation, Rad2Deg, Scale

schema = PolicySchema(
    observations=[
        # Dataset recorded in degrees; NOVA reports radians.
        Observation.joint_positions("arm", source=mg, ops=[Rad2Deg()]),
        # Position is millimetres and orientation radians, so they convert apart.
        Observation.tcp(
            "eef",
            source=mg,
            action=True,
            position_ops=[Scale(0.001)],  # mm -> m
            orientation_ops=[Rad2Deg()],
        ),
    ]
)
```

| Operator | Direction | Notes |
| --- | --- | --- |
| `Rad2Deg()` | bijective | Exact both ways. |
| `Scale(factor)` | bijective | `Scale(0.001)` is mm → m. A zero or non-finite factor is rejected at construction. |
| `Clamp(low, high)` | bidirectional | Bounds the observation forward *and* the command on the way back — limiting a command is the safety-relevant direction. |

Operators run front-to-back on the observation and back-to-front on the action, so
`[Scale(0.001), Clamp(-1, 1)]` clamps in metres in both directions. They are element-wise, apply to
joints and TCP but not to `computed` or `constant` observations, and an operator that declares
itself forward-only is rejected on a channel the policy writes — the executor would have nothing to
send back.

The inversion happens in the executor, **before** relative targets resolve: a delta in degrees
added to a state in radians is exactly the error this exists to prevent.

These operators are element-wise scalars. Rotation representation and coordinate system are
declared separately, below.

### Rotation representation and coordinate systems

NOVA reports TCP poses as `[x, y, z, rx, ry, rz]` (mm, rotation vector) in the controller's world.
A policy trained on another rotation representation or another frame declares it on the TCP entry,
with NOVA's own terms; the conversion is then the same for every policy client, on the observation
and on the action:

```python
from novapolicy import Action, Observation, OrientationType

Observation.tcp(
    "eef",
    source=mg,
    orientation_type=OrientationType.QUATERNION,  # eef_qx, eef_qy, eef_qz, eef_qw
    coordinate_system="table",  # a coordinate system of the controller
)
Action.tcp("eef", target=mg, orientation_type=OrientationType.QUATERNION, coordinate_system="table")
```

`orientation_type` takes any of NOVA's `OrientationType` values: rotation vector, quaternion
`[x, y, z, w]` or one of the intrinsic/extrinsic Euler conventions (named `_e1.._e3`). Element-wise
operators on quaternion components are rejected. `coordinate_system` expresses poses, and relative
offsets, in that coordinate system (references are followed to the world); the executor fetches them
at start, and `await schema.load_coordinate_systems()` does it for a dry run.

### Checking units against the checkpoint

Declared units are an assertion by whoever wrote the schema. When the client can read the
checkpoint's normalization statistics (`policy_preprocessor.json` and its safetensors, alongside
`config.json`), it holds the first live observation against them and reports what it finds:

- **Far outside** the training range — ten spans or more — raises before the robot moves. That is
  what a unit mismatch looks like; a pose does not land there.
- **Somewhat outside** warns. Starting from a pose the demonstrations never visited is normal.

A checkpoint shipped as `config.json` alone simply skips the check. GR00T servers do not serve their
statistics, so pass the checkpoint's `statistics.json` to the client for the same check:
`Gr00tPolicyClient(..., statistics="checkpoint/statistics.json")` (the embodiment whose state keys
match the server's is used, or name it with `embodiment=`). One asymmetry worth knowing:
bounds catch values that are too *large* for the training range — degrees where radians were
expected. The reverse sits comfortably inside a degree range and cannot be caught this way.

### Stale inputs

A camera can freeze and a policy service can hang. Freshness is declared per channel; a frame
older than its bound, or an inference past `inference_timeout_s`, ends the run with
`RuntimeError("Stale policy input ...")`.

```python
from novapolicy import Observation, PolicyExecutor

schema = PolicySchema(
    observations=[
        Observation.joint_positions("arm", source=mg),
        # This camera runs slower than the rest, so it declares its own bound.
        Observation.image("wrist", source=cameras.device(DEVICE), max_age_s=0.5),
    ]
)

executor = PolicyExecutor(
    schema,
    policy,
    camera_max_age_s=1.0,  # default bound for channels that declare none
    inference_timeout_s=30.0,  # liveness guard on one get_actions call; 0 disables
)
```

### How a run ends

How the run ended decides what happens to motion the controller already accepted:

- **Normal end** — a stop condition, the execution timeout or an external `stop()` — drains the
  accepted waypoints before the sessions close. They were
  owed to the caller.
- **Error or e-stop** drops them. Cancelling immediately is the point on a failure path.

> These are **software** guards running in the executor loop, not a safety system.

### Stop conditions

Policies run open-ended: they don't signal "finished". A stop condition is a fast,
synchronous check that runs on every tick; returning `True` ends the run normally
(its name appears in `result.reason`). The typical use in an industrial cell is an
IO stop signal: end the episode when an operator button or PLC sets an input:

```python
from novapolicy import SequentialExecution, StopContext


def stop_on_io(ctx: StopContext) -> bool:
    """Stop the policy when digital_in[3] goes high."""
    return bool(ctx.io_values and ctx.io_values.get("digital_in[3]"))


executor = PolicyExecutor(
    schema,
    policy,
    execution=SequentialExecution(),
    stop_conditions=[stop_on_io],
)
result = await executor.run()
# result.reason == "stop condition: stop_on_io"
```

Stop conditions must be fast (no network calls). Use `Observation.computed()` for async data.

Limits for moves NOVA plans, such as the move to the start pose, are `MotionSettings`, as everywhere
in NOVA. Streamed policy steps run within the controller's own limits; for production, rely on the
safety zones and protective stops configured on the robot controller.

## Teleoperation

There's no built-in teleop device, but the standalone jogging layer is the
building block: feed `set_target(...)` from whatever input you have (a leader
arm, keyboard, gamepad, spacemouse) in your own script. See
[docs/jogging.md](docs/jogging.md).

## Further reading

| Doc                                  | Covers                                                                                                                                                                 |
| ------------------------------------ | ---------------------------------------------------------------------------------------------------------------------------------------------------------------------- |
| [docs/jogging.md](docs/jogging.md)   | Standalone jogging: `jog_joints` / `jog_tcp`, joint/TCP modes, chunked targets, dual-arm, and error handling                                                           |
| [docs/architecture.md](docs/architecture.md) | How the parts fit: mapping, the send pipeline and its modules, execution modes, adding a policy provider |
| [docs/executor.md](docs/executor.md) | Advanced: explicit sequential/continuous execution modes, asynchronous inference, and the client/server timestamp protocol                                |
| [docs/contracts.md](docs/contracts.md) | What the executor promises, as numbered and tested invariants for mapping (M) and timing (T); the reasons in [docs/adr/](docs/adr/) |
| [docs/schema.md](docs/schema.md)     | Every observation type at a glance, then IO mappings, units, relative and TCP actions, object poses (incl. Isaac Sim prims), computed observations/actions                    |
| [docs/gr00t.md](docs/gr00t.md)       | `Gr00tPolicyClient` for [NVIDIA Isaac GR00T](https://github.com/NVIDIA/Isaac-GR00T) inference servers over ZMQ, and setting up a GR00T policy on a real robot (what the actions mean, checking the cell, run script, execution mode, limits) |
| [providers/lerobot/README.md](providers/lerobot/README.md) | LeRobot async inference, checkpoint-derived `chunk_size` / `n_action_steps`, and remote-checkpoint configuration                                                        |
| [docs/reinforcement-learning.md](docs/reinforcement-learning.md)             | Run a trained reinforcement-learning policy (Isaac Lab / LEAPP export): mapping its inputs and outputs to a schema, goal and command inputs, checks before a real run |
| [docs/rerun.md](docs/rerun.md)       | Optional real-time 3D visualization of execution                                                                                                                       |
| [docs/migration.md](docs/migration.md) | Changes that need an edit when upgrading                                                                                                                             |

### Examples

▶ [`execute_custom_policy_on_dual_arm.py`](examples/execute_custom_policy_on_dual_arm.py): two UR5e robots with cameras, IOs, and stop conditions\
▶ [`execute_gr00t_dual_arm.py`](examples/execute_gr00t_dual_arm.py): dual arm with GR00T ZMQ + 4 cameras\
▶ [`execute_gr00t_single_arm_pick_and_place/`](examples/execute_gr00t_single_arm_pick_and_place/): an IsaacLab-trained GR00T pick-and-place on a real UR10e with a Zimmer gripper, plus a training-episode replay\
▶ [`execute_isaaclab_rl_reach/`](examples/execute_isaaclab_rl_reach/): an Isaac Lab reinforcement-learning reach policy (LEAPP export) on a UR10e at 60 Hz, one action per step, with `LeappPolicyClient`\
▶ [`execute_leapp_two_point_reach/`](examples/execute_leapp_two_point_reach/): a trained reinforcement-learning policy (LEAPP export) that moves a UR10e to a part detected at one of two points\
▶ [`jogging/`](examples/jogging/): standalone jogging (single/dual arm, joint/TCP, chunked), no policy\
▶ [`pick_and_place_umi_ur10e.py`](examples/pick_and_place_umi_ur10e.py): the same pick-and-place the choreo3 checkpoints learn, written out by hand — a way to prove a cell before a policy runs on it

Cells for the examples are installed with
[`setup_cell.sh`](examples/setup_cell.sh): `dual-arm` for the two-UR5e examples,
`umi` for the UR10e carrying the UMI gripper and its `umi_corrected` TCP.
