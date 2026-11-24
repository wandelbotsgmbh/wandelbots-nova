# 0022 Live targets can follow a spring-damper and go out as a braking horizon (opt-in)

Status: accepted, 2026-11-24. `ServoHorizon` in `pipeline/modules/servo_horizon.py`; the Quest
teleoperation example (`examples/jogging/jogging_quest.py`) uses it.

**Context.** Teleoperation sends the hand's pose as a live target, about 70 to 100 times a
second. Sent alone, each is one waypoint the jogger's lead (100 ms) ahead, and NOVA's follower
steers at it with `(waypoint − q) / time left` without knowing what comes next: at a sudden
stop or reversal the arm overshoots and rings. A hand reverses at up to about 11 m/s², more than
the arm can brake, so the arm also falls behind the waypoints' times. A `LiveBuffer` sends more
waypoints, but they are the hand's exact past path, which asks for the same accelerations; its
window adds to the lag. A client-side acceleration limiter, a longer braking delay and a shorter
lead did not help.

What a servo interface needs is a path the arm can follow: one that rounds a turn the arm cannot
take into one it can, and brakes before a stop. A second-order spring-damper does that:
`q'' = gain · (ref − (q + lookahead · q'))`, with ω = √gain and ζ = lookahead · √gain / 2. It
lags a constant-speed target by `lookahead` and, at `lookahead = 2 / √gain`, settles without
overshoot.

**Decision.**

- `ServoHorizon` runs that spring on the client, on live targets, at 2 ms steps: the newest
  target is the reference, the state is kept on the session clock and advanced to the time the
  target is due, then simulated forward with the target held. That path, braking onto the
  target, goes out as the chunk: `horizon_ms` (400) of waypoints `step_ms` (32) apart, the first
  on NOVA's 8 ms cycle (ADR 0021). NOVA gets a path it can follow, with the braking already in
  it.
- Defaults gain 300 and lookahead 0.115 s, critically damped. Stiffer pairs (800 / 0.07,
  2000 / 0.045) lagged less in the client's path but tracked worse on the arm.
- Acceleration and speed limits scale the whole step down, keeping its direction: 8 m/s² and
  1.5 m/s for the TCP position, 40 rad/s² and 3 rad/s for its rotation or each joint.
- A rotation vector past a half-turn is taken as the equivalent closest to the spring's, so the
  tool does not swing the long way round.
- It is a chunk module of the `input` stage, like `LiveBuffer`: a chunk of several steps or a
  policy's lead chunk passes through untouched.

**Measured** on a virtual UR10e, the Quest mock's fast part (flicks, a 0.9 m/s zigzag, wrist
rolls, up to 11 m/s²).

With the client in a pod on the NOVA instance, two runs each:

| | lag | position rms / max | overshoot, median / max | rotation rms |
| --- | --- | --- | --- | --- |
| each pose alone | does not settle | 95 to 101 / 282 to 299 mm | 0.1 to 0.9 / 33 to 61 mm | 9 to 12° |
| `ServoHorizon()` | 188 to 196 ms | 6.8 to 7.0 / 24 to 26 mm | 0 / 0.2 mm | 0.9° |

Each pose alone, the 0.9 m/s zigzag set the arm ringing by about ±60 mm, on for half a second
after the hand had stopped.

Over a remote network, one or two runs each (runs scatter by up to 2×):

| | lag | position rms / max | overshoot, median / max |
| --- | --- | --- | --- |
| each pose alone | 176 ms | 34 / 138 mm | 32 / 125 mm |
| `LiveBuffer(200)` | 352 ms | 28 / 131 mm | 12 / 44 mm |
| `ServoHorizon()` | 220 ms | 19 / 95 mm | 3 / 48 mm |

The targets' jitter over the remote network damped the ringing of each pose alone; it added
about 30 ms of lag to `ServoHorizon` and most of its error.

**Consequences.** Opt-in: the joggers' default pipeline is unchanged. A robot controller that
filters its servo input again adds that filter's lag on top. Where the hand moves harder than the
arm can, error remains (24 mm peak from the instance, 95 mm over a remote network). Run the
client next to NOVA where it can.
