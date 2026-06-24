"""IO streaming and writing for policy execution.

Provides:
- ``IOStreamCache``: background WebSocket that caches latest IO values
- ``IOStreamManager``: owns the per-controller stream caches for one episode
- ``IOWriter``: deduplicated IO writes with serialized access
"""

from __future__ import annotations

import asyncio
import contextlib
import logging
from typing import TYPE_CHECKING

from nova.cell.io import IOAccess
from novapolicy._sdk import get_api_gateway, get_cell, get_controller_id

if TYPE_CHECKING:
    from collections.abc import Mapping

    from nova.cell.motion_group import MotionGroup
    from novapolicy.types import ValueType

logger = logging.getLogger(__name__)


class IOStreamCache:
    """Caches the latest IO values via a persistent WebSocket stream."""

    def __init__(self, motion_group: MotionGroup, ios: list[str]) -> None:
        self.motion_group = motion_group
        self._io_keys = ios
        self.values: dict[str, object] = {}
        self._task: asyncio.Task[None] | None = None
        self._ready = asyncio.Event()

    async def start(self) -> None:
        """Start the background IO stream listener."""
        if self._task is not None:
            return
        self._task = asyncio.create_task(self._run(), name=f"io-stream-{self.motion_group.id}")
        try:
            await asyncio.wait_for(self._ready.wait(), timeout=5.0)
        except TimeoutError:
            logger.warning(
                "IO stream for %s did not receive initial values within 5s",
                self.motion_group.id,
            )

    async def stop(self) -> None:
        """Cancel the background stream task."""
        if self._task is not None:
            self._task.cancel()
            with contextlib.suppress(asyncio.CancelledError, OSError):
                await self._task
            self._task = None

    async def _run(self) -> None:
        """Subscribe to IO stream and cache values."""
        api_client = get_api_gateway(self.motion_group)
        cell = get_cell(self.motion_group)
        controller_id = get_controller_id(self.motion_group)

        if not self._io_keys:
            self._ready.set()
            return

        stream = None
        try:
            stream = api_client.controller_ios_api.stream_io_values(
                cell=cell,
                controller=controller_id,
                ios=self._io_keys,
            )
            async for response in stream:
                for io_val in response.io_values:
                    self.values[io_val.root.io] = io_val.root.value
                if not self._ready.is_set():
                    self._ready.set()
        except asyncio.CancelledError:
            raise
        except (OSError, RuntimeError) as e:
            logger.warning("IO stream for %s ended: %s", self.motion_group.id, e)
        except Exception:
            logger.exception("IO stream for %s crashed", self.motion_group.id)
        finally:
            if stream is not None:
                with contextlib.suppress(asyncio.CancelledError, OSError, RuntimeError):
                    await stream.aclose()


class IOStreamManager:
    """Owns the per-controller IO stream caches for one execution episode.

    Deduplicates by controller (one stream per controller, even with multiple
    motion groups), exposes a merged latest-value view, and wires each cache
    into its session so guards can read live IO.
    """

    def __init__(
        self, motion_groups: list[MotionGroup], io_by_controller: Mapping[str, list[str]]
    ) -> None:
        self._motion_groups = motion_groups
        self._io_by_controller = io_by_controller
        self._caches: list[IOStreamCache] = []

    async def start(self) -> None:
        """Open one IO stream per controller that has configured IO keys."""
        started_controllers: set[str] = set()
        for mg in self._motion_groups:
            controller_id = get_controller_id(mg)
            if controller_id in started_controllers:
                continue
            io_keys = self._io_by_controller.get(controller_id)
            if not io_keys:
                continue
            started_controllers.add(controller_id)
            cache = IOStreamCache(mg, io_keys)
            self._caches.append(cache)
            await cache.start()

    def wire_to_sessions(self, sessions: Mapping[str, object]) -> None:
        """Give each session a live reference to its controller's IO values."""
        for cache in self._caches:
            session = sessions.get(cache.motion_group.id)
            if session is not None:
                session.set_io_values_ref(cache.values)  # type: ignore[attr-defined]

    async def stop(self) -> None:
        """Close all IO streams."""
        for cache in self._caches:
            await cache.stop()
        self._caches.clear()

    @property
    def all_values(self) -> dict[str, object]:
        """Merged IO values across all caches."""
        merged: dict[str, object] = {}
        for cache in self._caches:
            merged.update(cache.values)
        return merged


class IOWriter:
    """Writes IO values with deduplication and serialized access.

    Only writes values that have actually changed, and serializes writes
    per instance to avoid 429 rate-limit errors from the NOVA API. The
    value→model dispatch and write are delegated to nova's ``IOAccess``.

    TODO(core): ``IOAccess.write`` performs a per-controller
    ``list_io_descriptions`` validation roundtrip (cached after the first
    call). For a real-time policy loop a roundtrip-free write path in core
    would be preferable — tracked as a follow-up task; switch to it once
    available.
    """

    def __init__(self, motion_group: MotionGroup) -> None:
        self._io = IOAccess(
            get_api_gateway(motion_group),
            get_cell(motion_group),
            get_controller_id(motion_group),
        )
        self._last_written: dict[str, ValueType] = {}
        self._lock = asyncio.Lock()

    async def write(self, ios: dict[str, ValueType]) -> None:
        """Write IO values, skipping unchanged ones."""
        async with self._lock:
            for key, value in ios.items():
                if self._last_written.get(key) == value:
                    continue
                try:
                    await self._io.write(key, value)
                    self._last_written[key] = value
                except (OSError, RuntimeError, TypeError, ValueError, KeyError) as e:
                    logger.warning("Failed to write IO %s=%s: %s", key, value, e)
