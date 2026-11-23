# 0011 Providers own their wire protocol; the package owns the codec and NOVA

Status: accepted, 2026-11-23

**Context.** Four policy providers ship with novapolicy, and each reaches its policy differently:
GR00T over ZMQ with msgpack, LeRobot over gRPC with pickle, NATS with its own request/reply
protocol (vendored from the server's repository), LEAPP in-process with onnxruntime. Each provider
also built its own observation from the robot states and decoded its own answer, and the executor
carried code for one provider's features (a timestep queue's timeline, RTC seams).

**Decision.**

- **The wire stays with the provider.** No two providers share one, so a package-level transport
  layer would still need one implementation per protocol and reuse nothing.
- **The package owns the codec:** `PolicySchema` maps NOVA state, IO and images to the policy's
  names and units (`build_observation`, `observation_vector`) and the policy's outputs back to an
  `ActionChunk` (`decode_actions`): units, rotation formats, frames, relative targets, joints split
  across motion groups, IO mappings. A new provider maps names to its wire and nothing else.
  Of the shipped ones, GR00T and LEAPP decode with it; GR00T still assembles its own state arrays
  (its TCP formats), NATS its proprioception blocks, and LeRobot its flat feature layout and
  decoder, which its checkpoint validation is built on.
- **The package owns NOVA's side:** placement, the send pipeline and the action chunk stream.
- **What a provider needs beyond that comes as modules** (`PolicyClient.default_modules`): a timestep
  queue brings its `Smoothing`, `TimestepTimeline` and `Bridge`, and the executor knows nothing
  about queues. Its only feedback is generic: which policy step the robot is executing
  (`synchronize_action_timestep`).

**Verified (2026-11-23, a fresh virtual UR10e).** The executor no longer knows queues: a
`QueuedPolicyClient` keeps one timeline through its `TimestepTimeline` (live test asserts every
chunk after the first sits on it, unbridged) and moved the robot as before (path deviation 0.841°
rms / 4.02° max against 0.84° / 4.02° with the old code); a dummy VLA over NATS ran
continuously and sequentially; sequential execution sent 156 of 156 policy steps with IO on the
first; the RL reach with the shared decoder kept p90 0.3 mm.

**Consequences.** A new provider is a `PolicyClient` with a transport and one `get_actions`:
build the observation with the schema, call the policy, decode with the schema
([adding a provider](../architecture.md#adding-a-policy-provider)). Porting a provider touches no
executor code. RTC was removed rather than moved (its seams needed the executor to backdate
placement); a provider-side equivalent would bring its own module.
