# Computed action/observation examples

Isolated, runnable examples for the `computed` hooks in `PolicySchema`. Each uses a
single virtual UR5e and a trivial mock policy, so they run without a model server.

| File | Shows |
|---|---|
| `computed_action.py` | Both hooks together: `Observation.computed` injecting an external sensor reading into each observation, and `Action.computed` logging the policy's action chunk. |

`Observation.computed(fn)` registers `async fn(obs) -> dict`, called every step
*before* the policy; the returned keys are merged into the observation.
`Action.computed(fn)` registers `async fn(chunk: ActionChunk) -> None`, called every step
*after* the policy with the chunk it returned. The action hook drives nothing
on the robot itself — use it for logging, metrics, or external hardware.

Run:

    NOVA_API=http://<instance-ip> PYTHONPATH=. python novapolicy/examples/computed/computed_action.py
