"""Connection-state helpers for the SDK's NATS client.

nats-py reports three states that matter here: connected, reconnecting and closed. While
reconnecting it queues publishes, requests and subscribes and sends them once the
connection is back, so for the SDK a reconnecting client is a usable one. Only a closed
(or never opened) client is not.
"""

from __future__ import annotations

import asyncio
from typing import Any


def nats_usable(client: Any) -> bool:
    """Whether NATS calls on ``client`` can go through.

    True while the client is connected, and while nats-py is reconnecting (calls are
    queued until the connection is back). False for a missing, never-opened or closed
    client. Attributes are looked up leniently so test doubles only need ``is_connected``;
    ``is_closed`` and ``is_reconnecting`` count only when they are really ``True``.
    """
    if client is None:
        return False
    if getattr(client, "is_closed", False) is True:
        return False
    if getattr(client, "is_connected", False):
        return True
    return getattr(client, "is_reconnecting", False) is True


class ReconnectSignal:
    """Fan-out of nats-py's single ``reconnected_cb``.

    ``wait()`` returns once a reconnect completes after the call was made. Any number of
    waiters may wait at the same time, and each of them sees every reconnect. Consumers
    that keep state derived from pushed messages use it to re-read that state: messages
    published while the connection was down are gone, and core NATS does not replay them.
    """

    def __init__(self) -> None:
        self._generation = 0
        self._condition = asyncio.Condition()

    @property
    def generation(self) -> int:
        """Number of reconnects seen so far."""
        return self._generation

    async def wait(self, since: int | None = None) -> None:
        """Return once a reconnect has completed after ``since`` (default: now).

        Callers that set something up before waiting pass the ``generation`` they read
        beforehand, so a reconnect that lands in between is not missed.
        """
        generation = self._generation if since is None else since
        async with self._condition:
            await self._condition.wait_for(lambda: self._generation != generation)

    async def fire(self) -> None:
        """Wake every waiter; called once per completed reconnect."""
        async with self._condition:
            self._generation += 1
            self._condition.notify_all()
