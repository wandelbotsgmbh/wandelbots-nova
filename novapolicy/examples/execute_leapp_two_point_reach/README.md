# Two-point reach: run a reinforcement-learning policy that reaches a detected part

Runs a trained reinforcement-learning policy, exported with
[LEAPP](https://github.com/nvidia-isaac/leapp) the way Isaac Lab exports its policies, on a UR10e
with `LeappPolicyClient` and the `PolicyExecutor`. The guide for running reinforcement-learning
policies, and the checks before a real run, is
[docs/reinforcement-learning.md](../../docs/reinforcement-learning.md).

## What the policy does

The policy reaches a detected part: tool down, 100 mm above wherever the part is detected. It was
trained on two part positions:

| Part position | Part (mm, robot base frame) | Flange target |
| --- | --- | --- |
| A | (-750, -350, 400) | (-750, -350, 500) |
| B | (-750, 50, 400) | (-750, 50, 500) |

The detection is an observation like any other.
`Observation.pose("ee_pose", source=PartTarget(detector))` gives the policy the flange target for
the part as the detector reports it, on every step. If the part moves, the robot follows it. A part
detected more than 30 mm from A and B stops the run before the robot moves toward it, with the
stop condition `part_outside_trained_positions`: the policy was not trained to go there.

It works like Isaac Lab's reach task. It observes the joint positions, the joint velocities and the
target pose, and outputs absolute joint targets at 30 Hz, one per step. Joint positions are fed as
NOVA reports them; the export subtracts the start pose itself.

Each target is the previous target moved by at most 2° per joint (60°/s at 30 Hz), whatever the
network outputs. The export carries the previous target from step to step, and the policy observes
it. The policy was trained with the delay between an observation and the arm executing its
target, so it brakes early enough not to overshoot.

## Run it

The trained export is in `exported/reach/`. From the repository root:

```bash
uv sync --extra "novapolicy-leapp" --extra "nova-rerun-bridge"  # Rerun: 3D view of the run

# one observation and one inference on a throwaway virtual UR10e, nothing commanded
NOVA_API=http://<nova-host> uv run python \
  novapolicy/examples/execute_leapp_two_point_reach/run_reach.py --virtual

# a scripted detector reports the part at A, B, A for 4 s each
NOVA_API=http://<nova-host> uv run python \
  novapolicy/examples/execute_leapp_two_point_reach/run_reach.py --virtual --execute --parts A,B,A

# the part detected in an Isaac Sim scene (Isaac Sim may run on another machine)
NOVA_API=http://<nova-host> uv run python \
  novapolicy/examples/execute_leapp_two_point_reach/run_reach.py --virtual --execute \
  --isaac-sim http://isaac-host:8011/omniservice/api/v2 --prim /World/Part \
  --relative-to /World/UR10e/base --duration 12

# over a remote network: a longer lead, and the targets on NOVA's cycle grid
NOVA_API=http://<nova-host> uv run python \
  novapolicy/examples/execute_leapp_two_point_reach/run_reach.py --virtual --execute \
  --max-expected-latency-ms 150 --grid-ms 32
```

With `--controller NAME` instead of `--virtual` it uses an existing controller and asks before it
moves it.

After a run, the script prints the reason it ended, the rate it ran at and, per part position, how
far the flange ended from its target.

| Flag | Default | |
| --- | --- | --- |
| `--virtual` / `--controller NAME` | one is required | a throwaway virtual UR10e, or an existing controller (asks before moving) |
| `--execute` | off: dry run | move the robot |
| `--parts` | `A,B,A` | scripted detections, 4 s each |
| `--isaac-sim URL`, `--prim`, `--relative-to` | `$ISAAC_SIM_API`, `/World/Part`, none | detect the part in an Isaac Sim scene |
| `--duration` | 4 s per scripted part | run time in s |
| `--max-expected-latency-ms` | 100 | how far ahead each target goes; use 150 over a remote network |
| `--grid-ms` | off | put the targets on NOVA's cycle grid, a multiple of 8 ms (32 for this policy) |
| `--export` | `exported/reach/reach.yaml` | the LEAPP export's `.yaml` |

### Latency and the cycle grid

Run the script on the NOVA instance, for example as an app. Each target is due
`--max-expected-latency-ms` after the observation it was computed from. The default of 100 ms fits
there. Over a remote network, such as a VPN, state packets arrive late and in bursts; use
`--max-expected-latency-ms 150` there.

`--grid-ms 32` puts the one-step targets on NOVA's 8 ms cycle grid, through
`Timestamps(grid_ms=32)` in the default pipeline. Use it if the arm pulses around the policy's
pace. See [Execution settings](../../docs/reinforcement-learning.md#execution-settings).

## Plug in your own detector

A camera or pose estimator plugs in as the detector: anything with `start()`, `stop()` and a
`read()` that returns the part's latest `Pose`. See
[Object poses](../../docs/schema.md#object-poses).

## Troubleshooting

| Symptom | What to do |
| --- | --- |
| `stop condition: part_outside_trained_positions ...` after 0 policy steps | The part was detected more than 30 mm from A and B. Place it at a trained position. |
| `Unknown part positions` | `--parts` takes only `A` and `B`, separated by commas. |
| The arm lags or overshoots its targets | The lead is too short for the network. Raise `--max-expected-latency-ms` to 150, or run on the NOVA instance. |
| The arm pulses on the way between parts | Add `--grid-ms 32`. |
| The rate printed after the run is far below 30 Hz | Run the script on the NOVA instance. |
| `Stopped by the controller` | NOVA stopped a step that would break a joint limit or collide. |
| No pose from Isaac Sim | Check the extension URL and the prim path. |
