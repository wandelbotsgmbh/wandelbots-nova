"""``NatsPolicyClient`` for policy services on the NATS policy protocol.

The protocol, :mod:`novapolicy.nats.protocol`, is a copy of an external source of truth: read
its warning before editing it. ``examples/nats_mock_server.py`` serves it with a dummy policy.
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
