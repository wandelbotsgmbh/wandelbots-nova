"""Evaluate and wait on IO conditions (``PauseOnIO`` / ``StartOnIO`` shaped) in the SDK.

The controller evaluates these conditions itself while a trajectory executes; the SDK needs
the same evaluation to *resume* an IO pause, because the controller never restarts a paused
execution on its own — even after the condition clears it waits for a new
``StartMovementRequest`` (measured 2026-09-03, see
docs/architecture/incoming/pause-on-signal-evaluation.md). :class:`IOConditionWatcher` polls
the IO through the same REST endpoints programs use, for both ``CONTROLLER`` and ``BUS_IO``
origins, and tolerates the transient ``429`` the controller-IO endpoint returns while a write
is in flight.

Controller IOs are observed through the ``stream_io_values`` websocket: the controller-IO REST
endpoint serialises reads against writes and answers ``429 Too Many Requests`` to whichever
call comes second, so a polling watcher would make a program's own ``controller.write()`` fail
(measured). The stream pushes values at the controller step rate without touching that
endpoint. Bus IOs are polled through ``get_bus_io_values`` (a separate service, no such
collision); polling also serves as the fallback when a stream cannot be opened. The
controller's own reaction to an IO change is ~100 ms, so a 100 ms poll is not the bottleneck.
"""

from __future__ import annotations

import asyncio
import contextlib
import logging
from typing import Protocol

from nova import api

logger = logging.getLogger(__name__)

IOScalar = bool | int | float

_DEFAULT_POLL_INTERVAL_SECS = 0.1
_MAX_CONSECUTIVE_READ_ERRORS = 50
_MAX_STREAM_REOPENS = 3


class _StreamUnavailable(Exception):
    """The controller IO value stream could not be kept open."""


class IOCondition(Protocol):
    """Structural type shared by ``api.models.PauseOnIO`` and ``api.models.StartOnIO``."""

    io: api.models.IOValue
    comparator: api.models.Comparator
    io_origin: api.models.IOOrigin


def io_scalar(value: object) -> IOScalar:
    """The plain Python value of an ``IOValue`` (or one of its variants).

    Integer IO values travel as strings on the wire "to avoid precision loss";
    everything else is already a bool or float.
    """
    inner = getattr(value, "actual_instance", value)
    raw = getattr(inner, "value", inner)
    if isinstance(raw, str):
        return int(raw)
    if isinstance(raw, (bool, int, float)):
        return raw
    raise ValueError(f"Not an IO value: {value!r}")


def io_name(value: object) -> str:
    """The IO identifier of an ``IOValue`` (or one of its variants)."""
    inner = getattr(value, "actual_instance", value)
    name = getattr(inner, "io", None)
    if not isinstance(name, str):
        raise ValueError(f"Not an IO value: {value!r}")
    return name


def condition_holds(
    measured: IOScalar, comparator: api.models.Comparator, expected: IOScalar
) -> bool:
    """Evaluate ``measured <comparator> expected`` with the API's comparator semantics.

    "Use the measured I/O as the base value (a) and the expected input/output value
    as the comparator (b): e.g., a > b."
    """
    match comparator:
        case api.models.Comparator.COMPARATOR_EQUALS:
            return measured == expected
        case api.models.Comparator.COMPARATOR_NOT_EQUALS:
            return measured != expected
        case api.models.Comparator.COMPARATOR_GREATER:
            return measured > expected
        case api.models.Comparator.COMPARATOR_GREATER_EQUAL:
            return measured >= expected
        case api.models.Comparator.COMPARATOR_LESS:
            return measured < expected
        case api.models.Comparator.COMPARATOR_LESS_EQUAL:
            return measured <= expected
    raise ValueError(f"Unknown comparator: {comparator!r}")


class IOConditionWatcher:
    """Reads IOs of one controller's cell and waits for conditions on them.

    Args:
        api_client: The gateway (needs ``controller_ios_api`` and ``bus_ios_api``).
        cell: Cell id.
        controller_id: Controller whose IOs a ``CONTROLLER``-origin condition refers to.
        poll_interval_secs: Delay between two reads while waiting.
    """

    def __init__(
        self,
        api_client,
        cell: str,
        controller_id: str,
        *,
        poll_interval_secs: float = _DEFAULT_POLL_INTERVAL_SECS,
    ):
        self._api_client = api_client
        self._cell = cell
        self._controller_id = controller_id
        self._poll_interval_secs = poll_interval_secs

    async def read(self, io: str, origin: api.models.IOOrigin) -> IOScalar:
        """The current value of ``io`` at ``origin``."""
        if origin == api.models.IOOrigin.BUS_IO:
            values = await self._api_client.bus_ios_api.get_bus_io_values(cell=self._cell, ios=[io])
        else:
            values = await self._api_client.controller_ios_api.list_io_values(
                cell=self._cell, controller=self._controller_id, ios=[io]
            )
        for value in values:
            if io_name(value) == io:
                return io_scalar(value)
        raise ValueError(f"IO {io!r} not found at origin {origin}")

    async def holds(self, condition: IOCondition) -> bool:
        """Whether ``condition`` is currently true."""
        measured = await self.read(io_name(condition.io), condition.io_origin)
        return condition_holds(measured, condition.comparator, io_scalar(condition.io))

    async def wait_until(self, condition: IOCondition, *, holds: bool) -> None:
        """Return once ``condition`` evaluates to ``holds``.

        Controller IOs are read from the value stream (see module docstring); bus IOs
        are polled. Transient errors — a stream that closes, or a ``429`` from a REST
        read — are retried; a long run of failures is raised so a broken API never
        turns into a silent hang.
        """
        if condition.io_origin == api.models.IOOrigin.CONTROLLER:
            with contextlib.suppress(_StreamUnavailable):
                await self._wait_until_streamed(condition, holds=holds)
                return
            logger.debug("Controller IO stream unavailable, falling back to polling")
        await self._wait_until_polled(condition, holds=holds)

    async def _wait_until_streamed(self, condition: IOCondition, *, holds: bool) -> None:
        io = io_name(condition.io)
        expected = io_scalar(condition.io)
        failures = 0
        while True:
            try:
                async for response in self._api_client.controller_ios_api.stream_io_values(
                    cell=self._cell, controller=self._controller_id, ios=[io]
                ):
                    for value in response.io_values:
                        if io_name(value) != io:
                            continue
                        if (
                            condition_holds(io_scalar(value), condition.comparator, expected)
                            == holds
                        ):
                            return
            except asyncio.CancelledError:
                raise
            except Exception as error:  # noqa: BLE001 — retried, then given up below
                logger.debug(f"IO value stream failed: {error!r}")
            # The generated stream returns silently when the socket closes; reopen a
            # few times before giving the poll fallback a chance.
            failures += 1
            if failures >= _MAX_STREAM_REOPENS:
                raise _StreamUnavailable
            await asyncio.sleep(self._poll_interval_secs)

    async def _wait_until_polled(self, condition: IOCondition, *, holds: bool) -> None:
        errors = 0
        while True:
            try:
                current = await self.holds(condition)
            except asyncio.CancelledError:
                raise
            except Exception as error:  # noqa: BLE001 — retried, then re-raised below
                errors += 1
                if errors >= _MAX_CONSECUTIVE_READ_ERRORS:
                    raise
                logger.debug(f"IO condition read failed ({errors}), retrying: {error!r}")
            else:
                errors = 0
                if current == holds:
                    return
            await asyncio.sleep(self._poll_interval_secs)

    async def wait_until_released(self, condition: IOCondition) -> None:
        """Return once ``condition`` no longer holds (the pause signal cleared)."""
        await self.wait_until(condition, holds=False)
