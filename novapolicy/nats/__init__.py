"""NATS transport for remote policy inference.

Provides ``NatsPolicyClient`` for policy services that speak the NATS policy
protocol — plain core-NATS request/reply over ``policy.<id>.{info,reset,predict,close}``,
msgpack-encoded with numpy array support.

The protocol is defined in :mod:`novapolicy.nats.protocol`, whose source of
truth lives outside this repository; see that module's warning before editing it.

``examples/nats_mock_server.py`` serves the same protocol with fixed analytic motion, a dummy
policy for debugging a client without a trained policy in the loop.
"""

from novapolicy.nats.client import NatsPolicyClient
from novapolicy.nats.protocol import (
    DEFAULT_SUBJECT_PREFIX,
    PROTOCOL_VERSION,
    ActionSpec,
    FieldSpec,
    PolicyInfo,
    PolicyProtocolError,
)

__all__ = [
    "DEFAULT_SUBJECT_PREFIX",
    "PROTOCOL_VERSION",
    "ActionSpec",
    "FieldSpec",
    "NatsPolicyClient",
    "PolicyInfo",
    "PolicyProtocolError",
]
