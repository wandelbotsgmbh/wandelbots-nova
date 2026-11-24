# 0022 Live targets can follow UR's servoj law and go out as a braking horizon (opt-in)

Status: accepted, 2026-11-24. `ServoHorizon` in `pipeline/modules/servo_horizon.py`; the Quest
teleoperation example (`examples/jogging/jogging_quest.py`) uses it.

**Context.** Teleoperation sends the hand's pose as a live target, about 70 to 100 times a
second. Sent alone, each is one waypoint the jogger's lead (100 ms) ahead, and NOVA's follower
drives at it with `(waypoint − q) / time left` without braking for what comes next: at a sudden
stop or reversal the arm overshoots by 20 to 30 mm and rings back. A hand reverses at up to
about 11 m/s², more than the arm can brake, so the arm also falls behind the waypoints' times.
A `LiveBuffer` sends more waypoints, but they are the hand's exact past path, which asks for the
same accelerations; its window adds to the lag. A client-side acceleration limiter, a longer
braking delay and a shorter lead did not help.

A customer's Quest teleoperation with ur_rtde's `servoL` worked well. UR's servoj, decompiled
from URControl 5.26.3, steps a commanded state every 2 ms with
`q'' = gain · (ref − (q + lookahead · q'))`, one scale factor holding all joints to their
acceleration and speed limits: a second-order filter with ω = √gain and
ζ = lookahead · √gain / 2. It turns a turn the arm cannot take into one it can, and lags a
constant-speed target by `lookahead`. NOVA's UR driver calls `servoj(q, t=0.008, lookahead 0.1,
gain 300)` on the follower's output, but the follower runs before it.

**Decision.**

- `ServoHorizon` runs that law on the client, on live targets: the newest target is the
  reference, the state is kept on the session clock and advanced to the time the target is due,
  then simulated forward with the target held. That path, braking onto the target, goes out as
  the chunk: `horizon_ms` (400) of waypoints `step_ms` (32) apart, the first on NOVA's 8 ms cycle
  (ADR 0021). NOVA gets a path it can follow, with the braking already in it.
- Defaults gain 300 and lookahead 0.115 s, critically damped (lookahead = 2 / √gain). Stiffer
  pairs (800 / 0.07, 2000 / 0.045) lagged less in the client's path but tracked worse on the arm.
- Limits as servoj applies them, one scale for the whole step: 8 m/s² and 1.5 m/s for the TCP
  position, 40 rad/s² and 3 rad/s for its rotation or each joint.
- A rotation vector past a half-turn is taken as the equivalent closest to the spring's, so the
  tool does not swing the long way round.
- It is a chunk module of the `input` stage, like `LiveBuffer`: a chunk of several steps or a
  policy's lead chunk passes through untouched.

**Measured** on a virtual UR10e, client over a remote network, the Quest mock's fast part (flicks,
a 0.9 m/s zigzag, wrist rolls, up to 11 m/s²). The virtual robot does not run servoj, so its
measured TCP stream went through the decompiled servoj (gain 300, lookahead 0.1, t 8 ms) offline,
per TCP axis, as an estimate of a real UR:

| | lag behind the hand | position rms / max | overshoot at stops, median / max |
| --- | --- | --- | --- |
| each pose alone | 264 ms | 30 / 130 mm | 19 / 114 mm |
| `LiveBuffer(200)` | 448 ms | 25 / 121 mm | 0.5 / 34 mm |
| `ServoHorizon()` | 316 ms | 17 / 86 mm | 0 / 36 mm |

On the virtual robot itself (no servoj): 220 ms, 19 mm rms, overshoot median 3 mm, against
176 ms, 34 mm and 32 mm for each pose alone. One or two runs each; runs scatter by up to 2×.

**Consequences.** Opt-in: the joggers' default pipeline is unchanged. On a real UR the path is
filtered twice, by `ServoHorizon` and by servoj in NOVA's driver, which adds about a lookahead of
lag; a servo streaming mode in NOVA, passing the target to servoj directly, would remove one.
Where the hand moves harder than the arm can, about 85 mm peak error remains, as it would with
ur_rtde. The lag from a client on the NOVA instance instead of a remote network is measured
separately.
