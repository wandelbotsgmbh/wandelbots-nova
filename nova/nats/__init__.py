"""
Nova NATS integration package.

This package provides NATS client and messaging functionality for Nova.
"""

from nova.nats.client import NatsClient
from nova.nats.message import Message

__all__ = ["NatsClient", "Message"]
