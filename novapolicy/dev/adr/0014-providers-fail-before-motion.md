# 0014 Providers check the policy before the robot moves, and fail loudly

Status: accepted. Recorded 2026-11-23 from the code of the providers (`providers/gr00t/`,
`providers/lerobot/`, `mapping/training_range.py`; the NATS provider, removed later the same day, followed the same rules); the decisions are older.

**Context.** A mismatch between a schema and a policy (degrees for radians, a missing key, the
wrong number of axes) shows up as unexpected motion if nothing checks it first, and a provider
that skips an answer it cannot read leaves the robot standing still without a word.

**Decision.**

- **Check at connect, before motion.** `validate_schema` runs before the robot moves; decoding
  re-checks joint counts every step, since a schema can be right while the bound robot has the
  wrong number of axes.
- **Hold the first observation against the training statistics** (`training_range`), once per
  run: the point is a units or ordering mistake, not policing every tick. Outside the
  demonstrated range only warns, because a robot parked where the demonstrations never went is
  normal. `FAIL_SCALE` (5) times larger in magnitude than anything in training raises: that is
  not a pose but a unit (radians read as degrees is about 57×). It is scaled by magnitude, not
  span, or a joint that barely moved in training would flag its own park pose.
- **A deadline aborts the episode** (the executor's inference timeout, ADR 0018): better a clean
  failure than targets applied late to a moving robot.
- **An unreadable answer raises.** GR00T may leave out an action it was not trained on, so a
  missing key is skipped, but a present key without the expected array raises: that is what a
  server that changed its array serialization looks like, and skipping it once left the robot
  standing still with nothing in the log.
- **Settings come from the checkpoint**, not from arguments that can disagree with it; an
  argument that contradicts it raises (LeRobot: another `policy_type`, a longer chunk). How LeRobot reads them (one property every policy config
  implements, which inputs are compared, fallbacks) is in its
  [README](../../providers/lerobot/README.md#checkpoint-execution-settings).

**Consequences.** Misconfiguration ends a run before motion or at the first step, with a message
naming the cause, instead of as motion or a silent stall.
