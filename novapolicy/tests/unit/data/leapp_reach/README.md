# Reach-shaped LEAPP export (test fixture)

A small export shaped like Isaac Lab's manager-based reach export: inputs tagged with
`isaaclab_connection` (`state:robot:joint_pos`, `state:robot:joint_vel`, `command:ee_pose`), a
`last_action` feedback state, and one output, the absolute joint target
(`write:robot:set_joint_position_target`, current position + 0.0625 x action), at 60 Hz.

The network is random (seed 0); `test_leapp_client.py` checks the client against the outputs
LEAPP's own `InferenceManager` gives for it. Regenerate with LEAPP 0.8.1 and torch:

```bash
uv venv -p 3.12 venv-leapp && uv pip install -p venv-leapp/bin/python leapp==0.8.1
venv-leapp/bin/python build_reach_export.py out/  # writes out/reach_ur10e/
```

```python
"""Build a small reach-like LEAPP export, shaped like Isaac Lab's manager-based export.

Inputs carry `isaaclab_connection` like Isaac Lab's annotator: joint positions and velocities of
`robot`, the `ee_pose` command; a `last_action` feedback state; one output, the absolute joint
position target written with `set_joint_position_target` (current + 0.0625 * action).
"""

import sys
import torch
import leapp
from leapp import annotate, GraphConfigs
from leapp.utils.semantics import TensorSemantics, joint_position, joint_velocity

torch.manual_seed(0)
out_dir = sys.argv[1]
mlp = torch.nn.Sequential(torch.nn.Linear(25, 16), torch.nn.ELU(), torch.nn.Linear(16, 6))
mean = torch.linspace(-0.5, 0.5, 25)
std = torch.linspace(0.5, 1.5, 25)


def with_connection(sem: TensorSemantics, connection: str) -> TensorSemantics:
    sem.extra = {"isaaclab_connection": connection}
    return sem


def step(q, qd, cmd, last_action):
    q, qd, cmd = annotate.input_tensors(
        "reach_policy",
        [
            with_connection(joint_position("robot_joint_pos", q), "state:robot:joint_pos"),
            with_connection(joint_velocity("robot_joint_vel", qd), "state:robot:joint_vel"),
            with_connection(TensorSemantics(name="ee_pose", ref=cmd), "command:ee_pose"),
        ],
    )
    last = annotate.state_tensors("reach_policy", {"last_action": last_action})
    obs = torch.cat([q, qd, cmd, last], dim=-1)
    action = mlp((obs - mean) / std)
    target = q + 0.0625 * action
    annotate.update_state("reach_policy", {"last_action": action})
    (target,) = annotate.output_tensors(
        "reach_policy",
        [
            with_connection(
                joint_position("joint_pos_target", target), "write:robot:set_joint_position_target"
            ),
        ],
        export_with="onnx",
    )
    return target, action


leapp.start("reach_ur10e", save_path=out_dir)
with torch.no_grad():
    q = torch.tensor([[0.0, -1.57, 1.57, -1.57, -1.57, 0.0]])
    qd = torch.zeros(1, 6)
    cmd = torch.tensor([[0.85, -0.225, 0.2, 1.0, 0.0, 0.0, 0.0]])
    last = torch.zeros(1, 6)
    target, action = step(q, qd, cmd, last)
leapp.stop()
leapp.compile_graph(visualize=False, graph_configs=GraphConfigs(frequency=60.0))

# Reference values for the tests: two consecutive steps, the second one sees the first action.
with torch.no_grad():
    a1 = mlp((torch.cat([q, qd, cmd, torch.zeros(1, 6)], -1) - mean) / std)
    t1 = q + 0.0625 * a1
    a2 = mlp((torch.cat([q, qd, cmd, a1], -1) - mean) / std)
    t2 = q + 0.0625 * a2
print("T1", t1.tolist()[0])
print("T2", t2.tolist()[0])
```
