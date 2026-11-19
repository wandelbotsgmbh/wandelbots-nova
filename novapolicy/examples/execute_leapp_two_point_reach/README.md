# Two-point reach: run a reinforcement-learning policy that reaches a detected part

Runs a trained reinforcement-learning policy, exported with
[LEAPP](https://github.com/nvidia-isaac/leapp) the way Isaac Lab exports its policies, on a UR10e
with `LeappPolicyClient` and the `PolicyExecutor`. The guide for running reinforcement-learning
policies and the checks before a real run is
[docs/reinforcement-learning.md](../../docs/reinforcement-learning.md).

The policy reaches a detected part: tool down, 100 mm above wherever the part is detected. It was
trained on two part positions:

| Part position | Part (mm, robot base frame) | Flange target |
| --- | --- | --- |
| A | (-750, -350, 400) | (-750, -350, 500) |
| B | (-750, 50, 400) | (-750, 50, 500) |

The detection is an observation like any other: `Observation.pose("ee_pose", source=PartTarget(detector))`
gives the policy the flange target for the part as the detector reports it, on every step. If
the part moves, the robot follows it. A part detected more than 30 mm from A and B stops the run
before the robot moves toward it (`part_outside_trained_positions`): the policy was not trained to
go there.

It works like Isaac Lab's reach task: it observes the joint positions, the joint velocities and the
target pose, and outputs absolute joint targets at 30 Hz, one per step. Feed the joint positions as
NOVA reports them; the export subtracts the start pose itself, and starts from it.

NOVA moves the arm to every joint target, so each target has to be one the arm can follow: each one
is the previous target moved by at most 2° per joint (60°/s at 30 Hz), whatever the network
outputs. The export carries the previous target from step to step and the policy observes it, so
it knows which commands are still on their way to the arm. It was trained against a model of how
NOVA's controller follows the targets, fitted to a recorded run on a virtual UR10e: each target
takes effect 67 to 233 ms after the observation it was computed from, and the arm follows it with
limited acceleration (about 400°/s²), so the policy brakes early enough not to overshoot.

## Run it

The trained export is in `exported/reach/`.

```bash
uv sync --extra "novapolicy-leapp" --extra "nova-rerun-bridge"  # Rerun: 3D view of the run

# one observation and one inference on a throwaway virtual UR10e, nothing commanded
NOVA_API=http://<nova-host> uv run python run_reach.py --virtual

# a scripted detector reports the part at A, B, A for 4 s each
NOVA_API=http://<nova-host> uv run python run_reach.py --virtual --execute --parts A,B,A

# the part detected in an Isaac Sim scene (Isaac Sim may run on another machine)
NOVA_API=http://<nova-host> uv run python run_reach.py --virtual --execute \
  --isaac-sim http://isaac-host:8011/omniservice/api/v2 --prim /World/Part --relative-to /World/UR10e/base --duration 12
```

Run it on the NOVA instance (as an app): each target is due `--max-expected-latency-ms` (100 ms)
after the observation it was computed from, which fits there (see
[Latency](../../docs/reinforcement-learning.md#background)). On a virtual UR10e (NOVA 26.8.1), with
the part detected at A, B, A, it ran at 29 Hz and ended 1.3 to 2.5 mm from its targets when run in
the cluster, no joint past its end by more than 0.4°. Over a VPN with a 40 to 120 ms ping the state
packets arrive late and in bursts: there it needed `--max-expected-latency-ms 250` (150 and 200
stopped as over budget), ended 1.3 to 2.5 mm from its targets and went at most 1.9° past its end.
The flange path between A and B is a joint-space path, 26 to 50 mm off the straight line. A part detected 80 mm from A ended the
run with `stop condition: part_outside_trained_positions` after 0 policy steps. With
`--controller NAME` instead of `--virtual` it uses an existing controller and asks before it moves
it.

A camera or pose estimator plugs in as the detector: anything with `start()`, `stop()` and a
`read()` that returns the part's latest `Pose`.
