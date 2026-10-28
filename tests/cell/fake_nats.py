"""A minimal in-memory stand-in for ``nats.NATS`` as the SDK uses it for bus IOs."""

from __future__ import annotations

import asyncio
from types import SimpleNamespace

from nova.core.nats_state import ReconnectSignal


class FakeSubscription:
    def __init__(self, nats: FakeNats, subject: str, cb):
        self._nats = nats
        self.subject = subject
        self.cb = cb
        self.active = True

    async def unsubscribe(self) -> None:
        self.active = False
        self._nats.subscriptions.remove(self)
        if self._nats.unsubscribe_error is not None:
            raise self._nats.unsubscribe_error


class FakeNats:
    """``subscribe(subject, cb=...)`` + ``publish`` delivering to matching callbacks.

    ``reconnected`` is the same signal the SDK's real client exposes; ``reconnect()``
    fires it. ``unsubscribe_error`` makes every unsubscribe raise, like nats-py does on
    a closed connection.
    """

    def __init__(self, connected: bool = True, reconnecting: bool = False):
        self.is_connected = connected
        self.is_reconnecting = reconnecting
        self.is_closed = False
        self.subscriptions: list[FakeSubscription] = []
        self.subscribed = asyncio.Event()
        self.reconnected = ReconnectSignal()
        self.unsubscribe_error: Exception | None = None

    async def subscribe(self, subject: str, cb=None, **kwargs) -> FakeSubscription:
        assert cb is not None, "the SDK subscribes with a callback"
        subscription = FakeSubscription(self, subject, cb)
        self.subscriptions.append(subscription)
        self.subscribed.set()
        return subscription

    def subjects(self) -> set[str]:
        return {s.subject for s in self.subscriptions}

    async def publish(self, subject: str, data: bytes) -> None:
        for subscription in list(self.subscriptions):
            if subscription.active and subscription.subject == subject:
                await subscription.cb(SimpleNamespace(subject=subject, data=data))

    async def reconnect(self) -> None:
        """The connection dropped and came back: nothing published in between arrived."""
        self.is_connected = True
        self.is_reconnecting = False
        await self.reconnected.fire()
