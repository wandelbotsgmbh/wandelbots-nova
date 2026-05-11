"""GR00T ZeroMQ transport for policy inference.

Built against **NVIDIA Isaac GR00T N1.7** (``Isaac-GR00T`` commit ``v1.7.0``).
The ZMQ REQ/REP protocol, msgpack+numpy serialization, and observation/action
array shapes match that version.  If NVIDIA changes the wire format in a
future release, this package will need updating.

Provides ``Gr00tPolicyClient`` for NVIDIA GR00T inference servers,
plus the ``Gr00tMsgSerializer`` and ``Gr00tZmqTransport`` internals.
"""

from policy.gr00t.client import Gr00tPolicyClient
from policy.gr00t.transport import Gr00tMsgSerializer, Gr00tZmqTransport

GROOT_PROTOCOL_VERSION = "1.7"
"""The NVIDIA Isaac GR00T version this implementation targets."""

__all__ = [
    "GROOT_PROTOCOL_VERSION",
    "Gr00tMsgSerializer",
    "Gr00tPolicyClient",
    "Gr00tZmqTransport",
]
