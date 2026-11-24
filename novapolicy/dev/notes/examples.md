# Notes: examples

These notes were moved out of the customer guide. They keep development history,
measurements and design rationale. The text is verbatim.

## From examples/execute_gr00t_single_arm_pick_and_place/README.md

Table row:

| `episode_settings.json`, `episode_settings.py` | Measured from training episode 75: rate, home, the deltas' coordinate system, the gains and the gripper values; `gr00t_ur10e.py` loads it |

### Where the settings come from

[`episode_settings.json`](../../examples/execute_gr00t_single_arm_pick_and_place/episode_settings.json) is measured, not typed. Episode 75's recorded
joints, run through the robot's forward kinematics, give the path the training robot moved;
comparing it with the recorded actions gave what each action means (here TCP deltas in the world
frame), the scale between action units and realized motion (the training controller realized only
part of each command), the gripper polarity and values, the rate and the home joints.
[`episode_settings.py`](../../examples/execute_gr00t_single_arm_pick_and_place/episode_settings.py) loads it, and `RobotSettings.action(mg)` builds the
matching `Action`. Two values cannot be measured and were set by hand: `tcp`, the point on the
tool the training poses refer to, and `gripper_closed`, the fully closed gripper state (the
fingers stop on the cube first).

What is left as constants at the top of [`gr00t_ur10e.py`](../../examples/execute_gr00t_single_arm_pick_and_place/gr00t_ur10e.py) is what no episode
contains: the policy server, the prompt, the image size, the cameras and the gripper's address. See [the GR00T setup
guide](../../docs/gr00t.md#setting-up-a-policy-on-a-real-robot) for the whole workflow.

## From examples/execute_leapp_two_point_reach/README.md

NOVA moves the arm to every joint target, so each target has to be one the arm can follow: each one
is the previous target moved by at most 2° per joint (60°/s at 30 Hz), whatever the network
outputs. The export carries the previous target from step to step and the policy observes it, so
it knows which commands are still on their way to the arm. It was trained against a model of how
NOVA's controller follows the targets, fitted to a recorded run on a virtual UR10e: each target
takes effect 67 to 233 ms after the observation it was computed from, and the arm follows it with
limited acceleration (about 400°/s²), so the policy brakes early enough not to overshoot.

Run it on the NOVA instance (as an app): each target is due `--max-expected-latency-ms` (100 ms)
after the observation it was computed from, which fits there (see
[Latency](reinforcement-learning.md)). On a virtual UR10e (NOVA 26.8.1), with
the part detected at A, B, A, it ran at 29 Hz and ended 1.3 to 2.5 mm from its targets when run in
the cluster, no joint past its end by more than 0.4°. Over a VPN with a 40 to 120 ms ping the state
packets arrive late and in bursts: there use `--max-expected-latency-ms 150`. At that lead every
run reached its parts within 2.5 mm
([ADR 0021](../adr/0021-one-step-targets-on-the-cycle-grid.md)).
The flange path between A and B is a joint-space path, 26 to 50 mm off the straight line. A part detected 80 mm from A ended the
run with `stop condition: part_outside_trained_positions` after 0 policy steps. With
`--controller NAME` instead of `--virtual` it uses an existing controller and asks before it moves
it.

`--grid-ms 32` puts the one-step targets on NOVA's 8 ms cycle grid (`Timestamps(grid_ms=32)` in
the default pipeline); off by default. On a virtual UR10e at a 150 ms lead it halved the largest
path deviation, 5.8 mm against 11.1 mm over five runs each
([ADR 0021](../adr/0021-one-step-targets-on-the-cycle-grid.md)). It is not yet measured
on a physical robot or at a shorter lead.

## From the docstring of examples/jogging/jogging_keyboard.py

```text
Each pose goes out as a live target on the default pipeline. On a virtual UR10e over a remote
network the TCP starts moving about 130 ms after a press and holds 50 mm/s within about 10 mm/s.
A 500 ms ``LiveBuffer`` (see jogging_joints.py) holds the speed tighter but adds its window to the
delay; re-sending a chunk of future poses every iteration (``set_chunk``) made the arm swing around
the set speed at about 11 Hz.
```

## From the docstring of examples/jogging/jogging_quest.py

```text
``QuestMock`` stands in for the headset: controller poses at 72 Hz, arriving with a few ms of
jitter, along a scripted motion of fast flicks, sudden reversals and quick wrist rotations. The
``within`` part stays under the UR10e's speeds (TCP about 1.5 m/s, wrist about 180 deg/s) but its
reversals need up to 11 m/s2, more than the arm can brake; the ``beyond`` part exceeds the speeds
too. ``--speed 0.5`` plays a script at half speed, a quarter of the acceleration. ``seam`` turns
the tool slowly across the half-turn where its rotation vector flips sign. A real headset only
has to provide ``latest()``: the newest controller pose and grip.

Each pose goes out as a live target (``set_target``) through ``ServoHorizon``: a critically damped
spring-damper (gain 300, lookahead 0.115 s) follows the pose, and the spring's path with the pose
held, braking onto it, goes to NOVA as a 400 ms horizon (ADR 0022). ``--raw`` sends each pose
alone instead, one waypoint the jogger's lead (100 ms) ahead.
Measured on a virtual UR10e, the ``within`` part at full speed:

- Client on the NOVA instance, ``ServoHorizon``: 190 ms behind the controller, 7 mm rms, 25 mm
  at most, no overshoot. ``--raw``: the zigzag sets the arm ringing by about 60 mm.
- Over a remote network, ``ServoHorizon``: 220 ms, 19 mm rms; ``--raw``: 176 ms, 34 mm rms,
  overshooting sudden stops by 20 to 30 mm.
```

## From the docstring of examples/jogging/jogging_live_buffer_comparison.py

```text
* ``buffer``: the 500 ms live buffer replays recent targets as a horizon (the workaround).

If ``lone+braking`` follows as smoothly as ``buffer`` with less lag, the buffer can go.
```
