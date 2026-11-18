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
target pose (19 values), and outputs absolute joint targets at 30 Hz, one per step. Feed the joint
positions as NOVA reports them; the export subtracts the start pose itself.

NOVA moves the arm to every joint target, so each target has to be one the arm can follow: this
export's action processing puts every target at most 2° per joint from the measured joints (60°/s
at 30 Hz), whatever the network outputs. A policy whose targets jump to the goal makes the arm move
as fast as the controller allows.

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

Run it on the NOVA instance (as an app): each target goes `--max-expected-latency-ms` (100 ms)
ahead, which fits there; over a remote network pass 150, and the arm then follows that much later
(see [Latency](../../docs/reinforcement-learning.md#background)). This policy was trained for at
most 33 ms. On a virtual UR10e (NOVA 26.8.1), with the part detected at A, B, A, it ran at 29 Hz and
ended 3 to 30 mm from its targets when run in the cluster (100 ms), and 22 to 77 mm over a VPN
with a 62 ms ping (150 ms); with the default 100 ms over that VPN it stops as over budget. Training
with a randomized action delay of that size is what closes the gap. A part detected 80 mm from A ended the
run with `stop condition: part_outside_trained_positions` after 0 policy steps. With
`--controller NAME` instead of `--virtual` it uses an existing controller and asks before it moves
it.

A camera or pose estimator plugs in as the detector: anything with `start()`, `stop()` and a
`read()` that returns the part's latest `Pose`.
