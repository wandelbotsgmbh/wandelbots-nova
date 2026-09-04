"""A minimal in-memory stand-in for ``nats.NATS`` as the SDK uses it for bus IOs."""

from __future__ import annotations

import asyncio
from types import SimpleNamespace


class FakeSubscription:
    def __init__(self, nats: FakeNats, subject: str, cb):
        self._nats = nats
        self.subject = subject
        self.cb = cb
        self.active = True

    async def unsubscribe(self) -> None:
        self.active = False
        self._nats.subscriptions.remove(self)


class FakeNats:
    """``subscribe(subject, cb=...)`` + ``publish`` delivering to matching callbacks."""

    def __init__(self, connected: bool = True):
        self.is_connected = connected
        self.subscriptions: list[FakeSubscription] = []
        self.subscribed = asyncio.Event()

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
