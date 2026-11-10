"""NATS transport for remote policy inference.

Provides ``NatsPolicyClient`` for policy services that speak the NATS policy
protocol — plain core-NATS request/reply over ``policy.<id>.{info,reset,predict,close}``,
msgpack-encoded with numpy array support.

The protocol is defined in :mod:`novapolicy.nats.protocol`, whose source of
truth lives outside this repository; see that module's warning before editing it.

:mod:`novapolicy.nats.mock_server` serves the same protocol with fixed analytic
motion, for debugging a client without a trained policy in the loop. It is not
imported here: it is a debugging tool, and nothing on the client path needs it.
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
