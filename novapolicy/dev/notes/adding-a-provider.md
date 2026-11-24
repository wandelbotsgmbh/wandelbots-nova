# Notes: adding a policy provider

These notes were moved out of the customer guide. They keep development history,
measurements and design rationale. The text is verbatim.

## From docs/adding-a-provider.md, introduction

A provider connects one kind of policy to novapolicy. It owns its wire (HTTP, gRPC, ZMQ, an
in-process runtime) and nothing else: the schema maps NOVA state to the policy's names and units
and the policy's answer back to robot targets, and the executor places and streams those targets
([ADR 0011](../adr/0011-providers-own-their-wire.md)). A provider is one class with one required
method, `get_actions`. [`examples/http_policy.py`](../../examples/http_policy.py) is a complete one in
26 lines of code, written against this contract without changing the package.

## From docs/adding-a-provider.md, step 2

Every provider is the same three lines; only the middle one, your wire, is yours:

```python
from novapolicy import PolicyClient, resolve_control_freq


class MyPolicyClient(PolicyClient):
    def __init__(self, wire, *, control_freq: float | None = None) -> None:
        self._wire = wire
        self._dt_ms = 1000.0 / resolve_control_freq(control_freq, owner="MyPolicyClient")

    async def get_actions(self, states, schema, images=None, io_values=None):
        groups = schema.observation_groups(await schema.build_observation(states, io_values))
        outputs = await self._wire.infer(groups, images)
        return schema.decode_actions(outputs, dt_ms=self._dt_ms, source="MyPolicy")
```

What you do not write: timestamps, bridging from the robot's position, clamping to joint limits,
holds at the end of a chunk, IO timing. The executor and the send pipeline do those for every
provider ([architecture.md](../architecture.md#the-send-pipeline)).

## From docs/adding-a-provider.md, step 3

Decoding also checks every chunk: a joint key with too few columns for its motion groups, or rows
narrower than the action columns, raise instead of moving the robot
([ADR 0014](../adr/0014-providers-fail-before-motion.md)).

## From docs/adding-a-provider.md, step 4

`QueuedPolicyClient` keeps one timeline across overlapping chunks and brings the modules that
timeline needs (`Smoothing`, `TimestepTimeline`, `Bridge`); the executor knows nothing about
queues. It requires `ContinuousExecution`: the executor refuses it with `SequentialExecution`.

## From docs/adding-a-provider.md, step 5

Give the provider a fake wire and check the chunk it returns. This is how
`test_a_provider_is_a_transport_and_the_schema_does_the_mapping` tests the snippet above:

```python
class EchoWire:
    async def infer(self, observation, images):
        return {"arm": [[0.1] * 6, [0.2] * 6]}


chunk = await MyPolicyClient(EchoWire(), control_freq=20).get_actions(
    {mg.id: SimpleNamespace(joints=(0.0,) * 6)}, schema
)
assert chunk.dt_ms == 50.0 and chunk.joints[mg.id] == [[0.1] * 6, [0.2] * 6]
```

## From docs/adding-a-provider.md, shipped providers

The [architecture page](../architecture.md#adding-a-policy-provider) lists their code sizes.

## Provider code sizes

The line-count table for each provider is in [architecture.md](../architecture.md#adding-a-policy-provider).
