"""Evaluate and wait on IO conditions (``PauseOnIO`` / ``StartOnIO`` shaped) in the SDK.

The controller evaluates these conditions itself while a trajectory executes; the SDK needs
the same evaluation to *resume* an IO pause, because the controller never restarts a paused
execution on its own — even after the condition clears it waits for a new
``StartMovementRequest`` (measured 2026-09-03, see
docs/architecture/incoming/pause-on-signal-evaluation.md).

Nothing here polls the API. Controller IOs are observed through the ``stream_io_values``
websocket (the controller-IO REST endpoint serialises reads against writes and answers
``429`` to whichever call comes second, so a polling watcher would make a program's own
``controller.write()`` fail — measured). Bus IOs are observed through NATS: the bus-IO
service pushes every value change on ``nova.v2.cells.{cell}.bus-ios.ios`` (~10 ms after a
write) and its own state on ``nova.v2.cells.{cell}.bus-ios.status`` (``CONNECTED`` when it
comes up; an empty state and ``null`` values when it goes away — measured on the virtual
Profinet). A single REST read establishes the initial state after subscribing, so an edge
between subscribing and the first push is not missed. When a source cannot be observed the
wait fails with :class:`IOConditionUnavailable` rather than degrading to polling.
"""

from __future__ import annotations

import asyncio
import contextlib
import json
import logging
from typing import Any, Protocol

from pydantic import TypeAdapter

from nova import api

logger = logging.getLogger(__name__)

IOScalar = bool | int | float

_MAX_STREAM_REOPENS = 3
_STREAM_REOPEN_DELAY_SECS = 0.5
_BUS_IO_CONNECTED = api.models.BusIOsStateEnum.BUS_IOS_STATE_CONNECTED.value
_io_values_adapter: TypeAdapter[list[api.models.IOValue]] = TypeAdapter(list[api.models.IOValue])


class IOConditionUnavailable(RuntimeError):
    """The source of an IO condition cannot be observed (stream gone, no NATS client)."""


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


def motion_enable_signal(
    io: str, origin: api.models.IOOrigin = api.models.IOOrigin.BUS_IO
) -> api.models.PauseOnIO:
    """A ``pause_on_io`` condition that lets the robot move only while ``io`` reads ``True``.

    This is the fail-safe wiring for a PLC / fieldbus permission signal: the robot pauses
    on path as soon as the signal reads ``False`` — because the PLC dropped it, or because
    the signal path itself is gone (a lost Profinet connection or a broken wire reads
    ``False``, never ``True``). It resumes once the signal reads ``True`` again. Pass it to
    ``execute()`` / ``plan_and_execute()`` as ``pause_on_io``; one signal per motion group,
    or the same signal for every group that must stop together.
    """
    return api.models.PauseOnIO(
        io=api.models.IOBooleanValue(io=io, value=False),
        comparator=api.models.Comparator.COMPARATOR_EQUALS,
        io_origin=origin,
    )


def _bus_io_state_connected(payload: bytes) -> bool:
    """Whether a ``bus-ios.status`` message reports a connected bus."""
    try:
        state = json.loads(payload).get("state")
    except (ValueError, AttributeError):
        return False
    return state == _BUS_IO_CONNECTED


class IOConditionWatcher:
    """Reads IOs of one controller's cell and waits for conditions on them — push only.

    Args:
        api_client: The gateway (needs ``controller_ios_api`` and ``bus_ios_api``).
        cell: Cell id.
        controller_id: Controller whose IOs a ``CONTROLLER``-origin condition refers to.
        nats_client: Connected NATS client for bus-IO conditions. When omitted, the one of
            the current program context is used; without either, bus-IO waits fail with
            :class:`IOConditionUnavailable`.
    """

    def __init__(self, api_client, cell: str, controller_id: str, *, nats_client=None):
        self._api_client = api_client
        self._cell = cell
        self._controller_id = controller_id
        self._nats_client = nats_client

    # -- single reads -----------------------------------------------------------

    async def read(self, io: str, origin: api.models.IOOrigin) -> IOScalar:
        """The current value of ``io`` at ``origin`` (one request, no polling)."""
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

    # -- waits ------------------------------------------------------------------

    async def wait_until(self, condition: IOCondition, *, holds: bool) -> None:
        """Return once ``condition`` evaluates to ``holds``.

        Raises:
            IOConditionUnavailable: The signal source cannot be observed.
        """
        if condition.io_origin == api.models.IOOrigin.BUS_IO:
            await self._wait_until_bus_io(condition, holds=holds)
        else:
            await self._wait_until_streamed(condition, holds=holds)

    async def wait_until_released(self, condition: IOCondition) -> None:
        """Return once ``condition`` no longer holds (the pause signal cleared)."""
        await self.wait_until(condition, holds=False)

    async def wait_until_bus_io_lost(self) -> None:
        """Return once the bus-IO service is not ``CONNECTED`` (or is gone entirely).

        The state is taken from the ``bus-ios.status`` NATS subject after one initial
        request; a service that cannot even be asked counts as lost.
        """
        nats_client = self._require_nats()
        queue: asyncio.Queue[bytes] = asyncio.Queue()

        async def on_status(message) -> None:
            await queue.put(message.data)

        subscription = await nats_client.subscribe(self._subject("status"), cb=on_status)
        try:
            try:
                state = await self._api_client.bus_ios_api.get_bus_io_state(self._cell)
            except Exception as error:  # noqa: BLE001 — a service that cannot be asked is lost
                logger.debug(f"Bus-IO state unavailable: {error!r}")
                return
            if state.state != api.models.BusIOsStateEnum.BUS_IOS_STATE_CONNECTED:
                return
            while True:
                if not _bus_io_state_connected(await queue.get()):
                    return
        finally:
            await subscription.unsubscribe()

    # -- controller IOs: value-stream websocket --------------------------------

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
            except Exception as error:  # noqa: BLE001 — reopened, then given up below
                logger.debug(f"IO value stream failed: {error!r}")
            # The generated stream returns silently when the socket closes; reopen a
            # few times, then fail rather than fall back to polling.
            failures += 1
            if failures >= _MAX_STREAM_REOPENS:
                raise IOConditionUnavailable(
                    f"IO value stream for {io!r} on {self._controller_id!r} keeps closing"
                )
            await asyncio.sleep(_STREAM_REOPEN_DELAY_SECS)

    # -- bus IOs: NATS push -----------------------------------------------------

    async def _wait_until_bus_io(self, condition: IOCondition, *, holds: bool) -> None:
        nats_client = self._require_nats()
        io = io_name(condition.io)
        expected = io_scalar(condition.io)
        queue: asyncio.Queue[tuple[str, bytes]] = asyncio.Queue()

        async def on_values(message) -> None:
            await queue.put(("values", message.data))

        async def on_status(message) -> None:
            await queue.put(("status", message.data))

        # Subscribe first, read once afterwards: an edge in between is then seen
        # either by the read or by a queued message, never lost.
        subscriptions = [
            await nats_client.subscribe(self._subject("ios"), cb=on_values),
            await nats_client.subscribe(self._subject("status"), cb=on_status),
        ]
        try:
            if await self._holds_or_none(condition) == holds:
                return
            while True:
                kind, payload = await queue.get()
                if kind == "values":
                    measured = self._value_from_push(payload, io)
                    if measured is not None and (
                        condition_holds(measured, condition.comparator, expected) == holds
                    ):
                        return
                elif _bus_io_state_connected(payload):
                    # The bus came (back) up: its values are not necessarily
                    # re-published, so ask once.
                    if await self._holds_or_none(condition) == holds:
                        return
        finally:
            for subscription in subscriptions:
                with contextlib.suppress(Exception):
                    await subscription.unsubscribe()

    async def _holds_or_none(self, condition: IOCondition) -> bool | None:
        """``holds()`` for a bus that may be gone: ``None`` when it cannot be read."""
        try:
            return await self.holds(condition)
        except asyncio.CancelledError:
            raise
        except Exception as error:  # noqa: BLE001 — the bus is gone; pushes will tell more
            logger.debug(f"Bus IO read failed, waiting for pushed values: {error!r}")
            return None

    @staticmethod
    def _value_from_push(payload: bytes, io: str) -> IOScalar | None:
        """The value of ``io`` in a ``bus-ios.ios`` message; ``None`` when absent or ``null``."""
        if not payload or payload.strip() == b"null":
            return None
        try:
            values: list[Any] = _io_values_adapter.validate_json(payload)
        except ValueError as error:
            logger.debug(f"Unparseable bus IO message: {error!r}")
            return None
        for value in values:
            if io_name(value) == io:
                return io_scalar(value)
        return None

    def _subject(self, leaf: str) -> str:
        return f"nova.v2.cells.{self._cell}.bus-ios.{leaf}"

    def _require_nats(self):
        nats_client = self._nats_client
        if nats_client is None:
            # Lazily: nova/__init__ imports this package.
            from nova import get_current_program_context

            context = get_current_program_context()
            nats_client = context.nova.nats if context is not None else None
        if nats_client is None or not getattr(nats_client, "is_connected", False):
            raise IOConditionUnavailable(
                "Bus IO conditions need a connected NATS client (create motion groups "
                "through Nova / Cell, or run inside a program)"
            )
        return nats_client
