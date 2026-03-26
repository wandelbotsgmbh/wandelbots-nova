"""Shared execution state for async actions and predicates during trajectory execution.

Provides a thread-safe, observable key-value store that async action handlers
can write to and ``WaitUntilAction`` predicates can read from.  State changes
notify all waiters so that ``wait_for`` returns as soon as a predicate becomes
true.

Example::

    state = ExecutionState()

    # In an async action handler:
    await state.set("part_detected", True)

    # In a WaitUntilAction predicate (evaluated by the executor):
    wait_until(lambda s: s.get("part_detected"))
"""

from __future__ import annotations

import asyncio
from typing import Any, Callable


class ExecutionState:
    """Per-trajectory shared state accessible by async action handlers and predicates.

    The state is backed by a plain ``dict[str, Any]``.  Writes go through the
    async :meth:`set` method which notifies any coroutines blocked in
    :meth:`wait_for`.  Reads via :meth:`get` are synchronous and lock-free
    (safe because the dict is only mutated while holding the condition lock on
    the same event-loop thread).
    """

    def __init__(self) -> None:
        self._state: dict[str, Any] = {}
        self._condition = asyncio.Condition()

    # -- read (sync) ----------------------------------------------------------

    def get(self, key: str, default: Any = None) -> Any:
        """Return the value for *key*, or *default* if not set."""
        return self._state.get(key, default)

    # -- write (async, notifies waiters) --------------------------------------

    async def set(self, key: str, value: Any) -> None:
        """Set *key* to *value* and wake all :meth:`wait_for` waiters."""
        async with self._condition:
            self._state[key] = value
            self._condition.notify_all()

    # -- blocking wait --------------------------------------------------------

    async def wait_for(
        self, predicate: Callable[[dict[str, Any]], bool], timeout: float | None = None
    ) -> bool:
        """Block until *predicate(state_dict)* returns ``True``.

        Args:
            predicate: Callable that receives the full state dict and returns
                a boolean.
            timeout: Maximum seconds to wait.  ``None`` means wait forever.

        Returns:
            ``True`` if the predicate became true, ``False`` on timeout.
        """
        async with self._condition:
            try:
                return await asyncio.wait_for(
                    self._condition.wait_for(lambda: predicate(self._state)), timeout=timeout
                )
            except asyncio.TimeoutError:
                return False

    # -- introspection --------------------------------------------------------

    def snapshot(self) -> dict[str, Any]:
        """Return a shallow copy of the current state dict."""
        return dict(self._state)
