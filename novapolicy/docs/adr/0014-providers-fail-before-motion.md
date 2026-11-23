# 0014 Providers check the policy before the robot moves, and fail loudly

Status: accepted. Recorded 2026-11-23 from the code of the providers (`nats/`, `gr00t/`,
`lerobot/`, `training_range.py`); the decisions are older.

**Context.** A mismatch between a schema and a policy (degrees for radians, a missing key, the
wrong number of axes) shows up as unexpected motion if nothing checks it first, and a provider
that skips an answer it cannot read leaves the robot standing still without a word.

**Decision.**

- **Check at connect, before motion.** NATS negotiates the observation and action spaces at
  connect, so a DOF or layout mismatch fails there. `validate_schema` runs before the robot
  moves; NATS re-checks joint counts every step anyway, since a schema can be right while the
  bound robot has the wrong number of axes. A refreshed spec on reset (a hot model swap) replaces
  the old one, so the checks reflect what runs.
- **Hold the first observation against the training statistics** (`training_range`), once per
  run: the point is a units or ordering mistake, not policing every tick. Outside the
  demonstrated range only warns, because a robot parked where the demonstrations never went is
  normal. `FAIL_SCALE` (5) times larger in magnitude than anything in training raises: that is
  not a pose but a unit (radians read as degrees is about 57×). It is scaled by magnitude, not
  span, or a joint that barely moved in training would flag its own park pose.
- **A deadline aborts the episode.** A NATS request timeout ends it: better a clean failure than
  targets applied late to a moving robot. "Nobody subscribed" (service down, wrong `policy_id`
  or `subject_prefix`) fails at once and is reported apart from "subscribed but too slow", a
  different thing to fix.
- **An unreadable answer raises.** GR00T may leave out an action it was not trained on, so a
  missing key is skipped, but a present key without the expected array raises: that is what a
  server that changed its array serialization looks like, and skipping it once left the robot
  standing still with nothing in the log.
- **Episode state starts over.** NATS resets its finite-differenced velocities and last action per
  episode: a stale value across episodes is a discontinuity the policy never saw in training. The
  first step's last action is the current pose (a hold); later ones are what was commanded.
- **LeRobot settings come from the checkpoint.** The chunk length is `len(action_delta_indices)`,
  an abstract property every policy config implements, because the field names differ (ACT
  `chunk_size`, diffusion `horizon`, fastwam `action_horizon`, vqbet none); a new or renamed
  policy needs no change. Settings are resolved once, since `connect` may run again on a
  reconnect. LeRobot is imported lazily: its policy registry costs seconds and loads torch.
  Statistics are fetched from the Hub like the config, or every Hub model would skip the range
  check. Only a `config.json` path maps to its directory; any other file would silently read the
  `config.json` beside it. A checkpoint the client cannot read, or whose config needs packages
  this environment lacks, falls back to explicit arguments; one LeRobot rejects raises, with
  draccus' exception type kept out of the contract. A client cannot read a server-local path:
  pass `config_path`. The schema check compares only `STATE` and `VISUAL` inputs: a checkpoint
  may declare `LANGUAGE` or `ENV` features a schema never emits, and demanding them would fail
  every task-conditioned checkpoint. A camera frame size that differs only warns: it belongs to
  the camera stream, and hardware cannot always deliver an arbitrary resolution.

**Consequences.** Misconfiguration ends a run before motion or at the first step, with a message
naming the cause, instead of as motion or a silent stall.
