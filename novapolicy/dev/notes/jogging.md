# Notes moved from docs/jogging.md

Material moved out of the customer guide [docs/jogging.md](../../docs/jogging.md): measurements,
design rationale, timing internals and ADR/contract references. The text is kept as it was, with
links fixed for this folder.

## Chunked targets: the keyboard example

From "Chunked targets":

In the keyboard example, a chunk of future poses re-sent every loop
iteration (10 ms) made the arm swing around the set speed at about 11 Hz; it sends live
targets instead.

## Where a chunk lands

Step zero is placed at `elapsed + max_expected_latency_ms` (`ActionChunkConfig`, 100 ms by default; a
constant `LEAD_MS` until 2026-11-25) on the
absolute session timeline, not at "now" resolved at send time. Content generated at trajectory
time `T` therefore always maps to the same timestamp, however often it is re-sent, so
overlapping chunks stitch together instead of the same trajectory point landing at a slightly
different timestamp in every chunk. The lead is constant on purpose: anything time-varying in
the time-to-position mapping moves already-commanded points to new absolute times, and the robot
executes each such shift as a lurch.

Each term of the anchor (the session clock at the loop's first state, `elapsed`, the lead) is
rounded to whole milliseconds on its own, so every chunk starts on one 10 ms grid. Truncating
the float sum started some chunks 1 ms early (8139 instead of 8140): two waypoints 1 ms apart
with different values, which the controller took as a speed jump of hundreds of mm/s.

## Short chunks are padded: braking delay

From "Short chunks are padded":

About
200 ms covered 100-150 ms network stalls in teleoperation.

## Short chunks are padded: bookkeeping

The padding is sent but is not treated as commanded motion: it is excluded
from `scheduled.until_server_ms`, so anything waiting for the chunk to finish ends at your
last step rather than sitting at it for the rest of the window.

## Waypoint kinds: request format

The NOVA Action Chunk Streaming API accepts **timestamped waypoints** in a single
`ActionChunkRequest`. Each waypoint carries either joint positions or a TCP pose,
and the mode picks which.

## Late chunks are trimmed

Surviving waypoints keep their original absolute timestamps, so trimming
never distorts the trajectory — see
[executor.md](../../docs/executor.md#unreachable-waypoints-are-trimmed) for the effect on
step indices.

## Closing the session: drain timeout

It is also
abandoned after `_DRAIN_TIMEOUT_S` if the server stops reporting progress, which
is logged.

## Ending at rest: smootherstep rationale

Plain `smoothstep` (`3x² − 2x³`) zeroes the
velocity but enters and leaves at peak acceleration, which the robot executes as a
stumble; that is the same defect as a linear ramp, one order up.

## Rerun sampling while jogging

With a Rerun viewer, the robot mesh is drawn from the state packets, at most once per
1/30 s of their generation time (`STATE_SAMPLE_PERIOD_S`): more often costs sampling and
serialisation work without adding trajectory fidelity. The Rerun writes themselves are
handed to a worker thread rather than paid on the control loop (see
[rerun notes](rerun.md)).

## LiveBuffer: measurements and timeline

`LiveBuffer` was removed on 2026-11-24 (ADR 0013); this section is kept as a record. The
trajectory clock it describes stays in `novapolicy/jogging/jogger.py`.

Nothing in that horizon is invented: every waypoint is a target that was really measured,
replayed late rather than extrapolated forward. Measured on a virtual UR10e with a sine
target set at about 90 Hz: 0.015° rms against the target with 500 ms, 0.45–0.84° and arm
peaks up to twice the commanded speed without it.

The cost is latency: the robot trails the live target by a little over the window. The
window has to be a few hundred milliseconds, because the window *is* the horizon and the
server caps its speed at whatever it can brake to a stop within; on the UR10e a 150 ms
window stalled a fifth of all samples.

How it keeps one timeline ([ADR 0013](../adr/0013-live-buffer-replays-measured-targets.md)):

- The trajectory clock is quantised to `TIMELINE_GRID_MS`, maps to the session clock with a
  constant lead (`LIVE_LEAD_MS`, 100 ms, also the joggers' `LEAD_MS` for chunks), and never runs
  backwards. Forward jumps are kept: they are the clock correcting after a late state read, and
  limiting them lets the timeline fall behind until every waypoint is in the past.
- The replay grid sits on absolute multiples of its spacing, not on the oldest surviving sample,
  which changes with every prune. The spacing is `ActionChunkConfig.single_step_dt_ms`, never coarser
  than half the buffered span, so a short buffer keeps more than its endpoints.
- The resampled span is rounded down, with an epsilon for float division, so the grid ends at or
  before the newest sample. A point past it could only hold the newest value and would be sent
  again with another one when the next sample arrives, rewriting a waypoint under the controller
  (up to 46 mm in a fast teleoperation move). Only the two-point minimum can reach past it,
  while the buffer is still filling. Timestamps are rounded, not truncated, so every waypoint
  stays on the grid.
- Too little history sends the bare target instead of waiting, which would never move.
- `MAX_LIVE_SAMPLES` caps the buffer besides the window: the trajectory clock holds at zero until
  the robot executes, so before the session starts or while its state stream stalls nothing ages
  out. A 100 Hz caller fills a 500 ms window with about 50 samples.

## ServoHorizon: control law, limits and measurements

In teleoperation the live target is a hand that stops and reverses harder than the arm can
brake. The replayed past of a `LiveBuffer` asks for the same accelerations and adds its window
to the lag. `ServoHorizon` ([ADR 0022](../adr/0022-servo-horizon-for-live-targets.md)) puts a
spring-damper between the target and NOVA instead: the newest target is the spring's
reference, the spring runs on the client in 2 ms steps up to the time the target is due, and
then on with the target held. That path, braking onto the target, goes out as the chunk:
`horizon_ms` (400) of waypoints `step_ms` (32) apart, the first on NOVA's 8 ms cycle.

- The spring is `q'' = gain · (ref − (q + lookahead_s · q'))`. The defaults, `gain=300` and
  `lookahead_s=0.115`, are critically damped (`lookahead_s = 2 / √gain`): it settles without
  overshoot and lags a constant-speed target by `lookahead_s`.
- Limits scale the whole step down, keeping its direction: `linear_acceleration` 8000 mm/s² and
  `linear_speed` 1500 mm/s for the TCP position, `angular_acceleration` 40 rad/s² and
  `angular_speed` 3 rad/s for its rotation or each joint.
- A rotation vector past a half-turn is taken as the equivalent closest to the spring's, so the
  tool does not swing the long way round.
- It applies to `set_target` only: a chunk of several steps, or a target without its own start
  time (before the loop has state), passes through and resets the spring. It is the only module
  of the `input` stage since `LiveBuffer` was removed (2026-11-24).

Measured on a virtual UR10e with the Quest mock's fast part (flicks, a 0.9 m/s zigzag, wrist
rolls, up to 11 m/s²). With the client on the NOVA instance, `ServoHorizon()` tracked within
7 mm rms (24 to 26 mm at most) with no overshoot, about 190 ms behind the hand. Each pose alone
did not settle (95 to 101 mm rms): the zigzag set the arm ringing by about ±60 mm, for half a
second after the hand had stopped. Over a remote network, 19 mm rms against 34 mm for each pose
alone and 28 mm for `LiveBuffer(window_ms=200)`. It is opt-in: the default pipeline is
unchanged. Run the client next to NOVA where it can.

## Timing targets: why the session clock

It must come from the server clock rather than a monotonic one: the robot executes
against the jogger session timer, and wall time drifts from it. Three properties
follow from that:

- **Sampled once per iteration.** Every target you derive from `elapsed` within
  one pass through the loop is stamped from the same instant.
- **Quantised** to a 10 ms grid (`novapolicy.jogging.jogger.TIMELINE_GRID_MS`), so successive chunks
  anchor on one absolute grid instead of each landing at a new phase.
- **Never runs backwards.** Server "now" is extrapolated between state samples, so
  a late sample can resolve below an earlier estimate; since content is a function
  of `elapsed`, a backward step would command the robot to reverse. Forward jumps
  are *not* limited — they are the clock correcting after the event loop was late
  reading the state stream, and rate-limiting them makes the timeline fall behind
  until every waypoint lands in the past.

There is no wait for the robot to report `RUNNING` before the clock starts.
Waiting for it deadlocks the live path: the timeline has to move before any target
spacing can be measured, and the robot has to move before it reports `RUNNING`. A
catch-up jump is not a concern either way, because targets are placed on an
absolute timeline and unreachable waypoints are dropped at send time.

## The send pipeline: stages, contract and EaseIn measurement

Modules before `Timestamps()` work on chunks (`ServoHorizon`, `HoldPadding`);
filters after it choose which timed waypoints go out without retiming them (`DropLate`);
waypoint modules last may change the values only (`EaseIn`). `Timestamps()` may be left out; it
then sits after the last chunk module. Each motion group runs its own copy. See
`novapolicy.pipeline.base`, `novapolicy.pipeline.modules` and contract T13 in
[contracts.md](../contracts.md).

Measured on a sine at about 90 Hz with a 500 ms buffer: peak speed in the first
second 54°/s instead of 85°/s.

## Examples: keyboard latency

From the examples table, `jogging_keyboard.py`: Over a remote network the TCP starts moving
about 130 ms after a press; a 500 ms `LiveBuffer` (removed 2026-11-24) added its window to that
delay.

## Error detection: drained queue and endpoint history

Note what is *not* in either table: there is no state for "the commanded
waypoints ran out". A drained queue keeps reporting `RUNNING` — the robot simply
brakes at the end of the last waypoint. Since NOVA 26.6 `PAUSED_BY_USER` follows
a Pause/Stop request only, and this SDK never sends one, so anything that needs
to know the robot came to rest measures it from the joint positions instead (see
[executor notes](executor.md#detecting-standstill)).

A NOVA instance without action chunk streaming raises `JoggingNotSupportedError` when the
session opens. The endpoint, `executeActionChunks`, exists on api-gateway 26.6 and newer; 26.5
reached the same feature through `executeWaypointJogging`, which 26.6 removed. Older gateways
answer the websocket upgrade with HTTP 404, which is reported as this error rather than as a
connection failure, so the fix (upgrade NOVA) is clear.
