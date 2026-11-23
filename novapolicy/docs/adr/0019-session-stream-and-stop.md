# 0019 The session sends without waiting for acknowledgements and ends with a stop request

Status: accepted. Recorded 2026-11-23 from the code (`jogging/waypoint_session.py`); the decision
is older. How a run's teardown follows its reason is in
[executor.md](../executor.md#teardown-follows-the-reason) and
[jogging.md](../jogging.md#closing-the-session).

**Context.** A `WaypointJoggingSession` holds one `executeActionChunks` websocket per motion group:
requests go out, acknowledgements and errors come back. The round trip is tens to hundreds of
milliseconds. Closing the stream is also how a session ends, and NOVA keeps session state across
streams.

**Decision.**

- **Sends are not gated on responses.** Responses are consumed in their own task, so the round
  trip does not set the command cadence. A chunk queued while a send is in progress replaces the
  pending one: stale updates coalesce, the newest wins.
- **Nothing pending, nothing sent.** The loop waits for the next chunk instead of sending hold
  messages, which would alter the motion profile.
- **A session ends with `StopActionChunksRequest`**, then waits up to `_STOP_TIMEOUT_S` (2 s) for
  the state stream to report it no longer `RUNNING` before it closes. Closing without the request
  leaves NOVA reporting the session `RUNNING` with its timer frozen: the next jogging session syncs
  its clock to that timer, and the next trajectory execution fails on the jogging details. Only a
  state after the server acknowledged the stop counts; a paused state before it is no answer.
- **A drain compares against acknowledged progress**, the last state timestamp, not the
  extrapolated estimate, which sails past the schedule on a stalled link. Its timeout
  (`_DRAIN_TIMEOUT_S`, 3 s) is generous next to the horizon it drains, so it trips only when the
  server stopped reporting progress.
- **The final target of a drain is reachable.** With `delay_until_braking_ms` the robot may already
  be past the last target's time, keeping its velocity; it still has to go back and stop there, so
  the target is re-sent without the delay at the earliest moment it can still be reached, and every
  filter lets it pass.

**Consequences.** The command rate is the producer's, whatever the network. A session that ends,
normally or not, leaves NOVA ready for the next session or trajectory. A stop that NOVA never
confirms costs at most 2 s and a warning.
