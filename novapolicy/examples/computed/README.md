# Computed observation and action example

Shows the `computed` hooks of `PolicySchema` on one virtual UR5e with a mock policy. No model
server is needed.

| File | Shows |
| --- | --- |
| `computed_action.py` | `Observation.computed` adds an external sensor reading to each observation. `Action.computed` logs the action chunk the policy returned. |

Use these hooks for data and devices that NOVA does not expose: a force sensor, a PLC, a
database, a gripper on its own bus.

## How the hooks work

- `Observation.computed(fn)` registers `async fn(obs) -> dict`. It is called on every step,
  before the policy. The returned keys are merged into the observation.
- `Action.computed(fn)` registers `async fn(chunk: ActionChunk) -> None`. It is called on every
  step, after the policy, with the chunk the policy returned. It does not move the robot. Use it
  for logging, metrics or external hardware.

Pass `keys=[...]` to either hook when a policy client should check that every model input is fed,
or should put action outputs into `chunk.values`. See
[Computed observations and actions](../../docs/schema.md#computed-observations-and-actions).

## Run

From the repository root:

```bash
uv sync --extra novapolicy
NOVA_API=http://<nova-host> PYTHONPATH=. uv run python novapolicy/examples/computed/computed_action.py
```

The script creates a virtual UR5e controller named `ur5e`, moves it home and runs the mock policy
for 5 s at 20 Hz. Every 25th step prints a `[computed action]` line with the joint target. The
virtual controller stays in the cell after the run; delete it in the NOVA UI if you do not need it.

## Adapt it

- Replace `read_sensor` with the read of your sensor driver. Return a dict with the keys the policy
  expects.
- Replace `report_action` with your sink: a file, a time-series database, a PLC register.
- Replace `CallbackPolicyClient(mock_policy)` with your policy client.

## Troubleshooting

| Symptom | What to do |
| --- | --- |
| The script cannot reach NOVA | Check `NOVA_API`, and set `NOVA_ACCESS_TOKEN` for a cloud instance. |
| `ModuleNotFoundError: novapolicy` | Run from the repository root with `PYTHONPATH=.`. |
| No `[computed action]` lines | The run ended before step 25. Check the printed `reason`. |
