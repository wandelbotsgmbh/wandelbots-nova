# Isaac Lab reach policy on a UR10e

Runs a policy trained on Isaac Lab's [`IsaacContrib-Deploy-Reach-UR10e`][task] task on a NOVA
UR10e: the flange tracks a target pose. The general steps, and how a policy's inputs and outputs
map onto a schema, are in [docs/reinforcement-learning.md](../../docs/reinforcement-learning.md).
This page covers what is specific to this task.

## What you need

A policy trained on `IsaacContrib-Deploy-Reach-UR10e` and exported with LEAPP, using the ONNX
backend and the same physics backend and overrides as training ([LEAPP export guide][leapp]):

```bash
uv run --extra leapp isaaclab leapp export --rl_library rsl_rl --task IsaacContrib-Deploy-Reach-UR10e
```

Copy the export directory (the `.yaml`, the `.onnx` model and the initial values) to
`exported/reach_ur10e/` next to this README, or pass `--export path/to/the.yaml`. Then install the
extras from the repository root:

```bash
uv sync --extra "novapolicy-leapp" --extra "nova-rerun-bridge"  # Rerun: 3D view of the run
```

## Run it

From the repository root:

```bash
# one observation and one inference on a throwaway virtual UR10e, nothing commanded
NOVA_API=http://<nova-host> uv run python \
  novapolicy/examples/execute_isaaclab_rl_reach/reach_ur10e.py --virtual

# the policy runs until the flange is within 5 mm and 2 deg of the target, or 12 s
NOVA_API=http://<nova-host> uv run python \
  novapolicy/examples/execute_isaaclab_rl_reach/reach_ur10e.py --virtual --execute \
  --target 850 -225 200 3.1416 0 0
```

`--controller NAME` instead of `--virtual` uses an existing controller and asks before it moves
it.

The target is a NOVA pose: mm and a rotation vector, inside the trained workspace (x 640..1140,
y -350..-100, z 100..300 mm, tool down). To take it from an Isaac Sim scene instead, point at the
Wandelbots NOVA extension. It may run on another machine; the port is in Isaac Sim's Wandelbots
NOVA menu.

```bash
ISAAC_SIM_API=http://isaac-host:8011/omniservice/api/v2 uv run python \
  novapolicy/examples/execute_isaaclab_rl_reach/reach_ur10e.py --virtual --execute \
  --prim /World/Target --relative-to /World/UR10e/base
```

| Flag | Default | |
| --- | --- | --- |
| `--virtual` / `--controller NAME` | one is required | a throwaway virtual UR10e, or an existing controller (asks before moving) |
| `--execute` | off: dry run | move the robot |
| `--target X Y Z RX RY RZ` | `850 -225 200 3.1416 0 0` | the target flange pose: mm and rotation vector (rad) |
| `--isaac-sim URL` | `$ISAAC_SIM_API` | read the target from a prim of an Isaac Sim scene |
| `--prim` | `/World/Target` | the prim whose pose is the target |
| `--relative-to` | none | express the prim relative to this prim, such as the robot base |
| `--export` | `exported/reach_ur10e/reach_ur10e.yaml` | the LEAPP export's `.yaml` |

## How the task maps onto the schema

| Isaac Lab | Export input / output | Schema |
| --- | --- | --- |
| `joint_pos` | `state:robot:joint_pos` | `Observation.joint_positions("joint_pos", ...)` |
| `joint_vel` | `state:robot:joint_vel` | `Observation.joint_velocities("joint_vel", ...)` |
| `pose_command` | `command:ee_pose` | `Observation.pose("ee_pose", ..., orientation_type=QUATERNION, position_ops=[Scale(0.001)])` |
| `RelativeJointPositionAction(scale=0.0625)` | `write:robot:set_joint_position_target_index` | `Action.joint_positions("arm", ...)`, absolute |

- **Rate.** The task runs at 60 Hz (`sim.dt` 1/120 s × `decimation` 2). The export records it, and
  the example runs the executor at `client.control_freq`.
- **Target frame.** Isaac Lab gives this task's pose command in the UR `base` frame, which is turned
  180° about z from `base_link`. NOVA reports UR poses in the controller's base frame, the same
  `base`. On a mounted robot, whose NOVA world is not its base, convert the target first.
- **Quaternion order.** Current Isaac Lab orders the command's quaternion `(x, y, z, w)`, as
  `OrientationType.QUATERNION` does. A policy trained with Isaac Lab 2.x used `(w, x, y, z)`.
- **Joint velocities.** UR controllers send no measured joint velocity, so they are differenced
  from the state stream.
- **Action.** The export applies the action term (current position + 0.0625 rad × output), so the
  schema declares an absolute joint action with no scale.
- **Start pose.** The example moves to Isaac Lab's UR10e start pose first:
  `[π, -π/2, π/2, -π/2, -π/2, 0]`.

The policy was trained against simulated PD joints, not a position-controlled arm. Run it on a
virtual robot first. NOVA's own checks (self-collision, joint limits) stop the arm when a step
would break them.

## Troubleshooting

| Symptom | What to do |
| --- | --- |
| `FileNotFoundError` for the export | Copy the export to `exported/reach_ur10e/`, or pass `--export`. |
| The export asks for the ONNX backend | Export again with the ONNX backend. |
| `Schema does not satisfy the LEAPP export` | The export has other inputs than this task. Check that it was exported from `IsaacContrib-Deploy-Reach-UR10e`. |
| The run ends with `timeout` short of the target | Check that the target lies inside the trained workspace and points the tool down. |
| The flange goes to a mirrored or rotated target | Check the target frame and the quaternion order (see above). |
| `Stopped by the controller` | NOVA stopped a step that would break a joint limit or collide. Choose a target inside the trained workspace. |

[leapp]: https://isaac-sim.github.io/IsaacLab/main/source/policy_deployment/05_leapp/exporting_policies_with_leapp.html
[task]: https://github.com/isaac-sim/IsaacLab/tree/main/source/isaaclab_tasks/isaaclab_tasks/contrib/deploy/reach/config/ur_10e
